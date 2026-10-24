"""Read-only saved-plan data, wall-clock animation, and reusable robot artists."""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from .common import JOINTS, LEFT_JOINTS, arm_side, arm_joints
from .visual_mesh import visual_surfaces
from .model import stick_length
from camera_calibration.core import digest
from safe_zone.geometry import Model


class PlanDisplay:
    def __init__(self, path):
        self.path=Path(path).resolve();self.obj=json.loads(self.path.read_text())
        self.req=json.loads((self.path.parent/'backend_request.json').read_text())
        self.side=arm_side(self.req.get('moving_side','right'))
        self.joints=arm_joints(self.side);self.stick_link=self.side+'_drumstick'
        self.other_side='left' if self.side=='right' else 'right'
        if self.obj.get('moving_side',self.side)!=self.side:
            raise ValueError('Plan/request arm mismatch')
        # Old backend exceptions omitted the schema. Permit ONLY historical
        # error reports with no trajectory payload, in memory, without rewriting
        # captures or accepting missing-schema successful/executable plans.
        self.legacy_error_report=(self.obj.get('schema') is None and self.obj.get('status')=='error'
            and isinstance(self.obj.get('error'),str) and bool(self.obj['error'].strip())
            and not self.obj.get('cycle') and not self.obj.get('outward'))
        if ((self.obj.get('schema')!='x5-moveit-result-v1' and not self.legacy_error_report)
                or self.req.get('schema')!='x5-moveit-request-v1' or self.req.get('execute') is not False
                or self.obj.get('motion_authorized') is not False
                or self.obj.get('hardware_execution_available') is not False):
            raise ValueError('Expected a non-executable X5 software preview plan')
        if digest(self.req['urdf'])!=self.obj['model']['calibrated_urdf_sha256']:
            raise ValueError('Saved preview robot model hash mismatch')
        self.model=Model(self.req['urdf']);self.root=ET.parse(self.req['urdf']).getroot()
        for name in ('start',self.other_side,'goal'):
            values=np.asarray(self.req[name],dtype=float)
            if values.shape!=(7,) or not np.isfinite(values).all():
                raise ValueError('Preview requires seven finite '+name+' angles')
        self.curves=self.obj.get('cycle') if self.obj['status']=='planned' else None
        self.curves=self.curves or [dict(time_s=0.,positions_rad=self.req['start'],phase='BLOCKED')]
        self.times=np.array([p['time_s'] for p in self.curves],dtype=float)
        self.values=np.array([p['positions_rad'] for p in self.curves],dtype=float)
        if (self.values.shape!=(len(self.times),7) or not np.isfinite(self.values).all()
                or not np.isfinite(self.times).all() or self.times[0]!=0
                or np.any(np.diff(self.times)<=0)):
            raise ValueError('Invalid saved animation samples/timestamps')
        self.duration=float(self.times[-1])
        self.playable=self.obj['status']=='planned' and self.duration>0
        if self.obj['status']=='planned' and not self.playable:
            raise ValueError('Planned trajectory has no playable samples')
        self.static=dict(zip(arm_joints(self.other_side),self.req[self.other_side]))
        self.static['openarmx_left_finger_joint1']=self.req['left_opening_m']
        self.retimed=None;self.preview_rate=1.
        if self.playable and self.obj.get('timing',{}).get('schema')=='x5-fixed-path-timing-v1':
            from .timing.core import Evaluator
            self.retimed=Evaluator(self.obj['timing'])
            if abs(self.retimed.duration-self.obj['duration_outward_s'])>1e-9:
                raise ValueError('Retiming/preview duration mismatch')

    def check_live_calibration(self,cal_path):
        if self.obj.get('calibration_sha256')!=digest(cal_path):
            raise ValueError('Plan and live camera calibration differ; refusing a misleading overlay')

    @property
    def warning(self):
        if self.obj.get('status')!='planned':
            from .diagnostics import blockage_summary
            return blockage_summary(self.obj)
        if self.obj.get('synthetic'):return 'SYNTHETIC PLAN — not planned against this live kit'
        if self.obj.get('collision_policy')=='observed_obstacles' and not self.req.get('occupied_only_preview'):
            return 'DETECTED-OBSTACLE PLAN — full arm + stick checked against the saved scene; unseen space unverified'
        if not self.obj.get('strict_unknown_space'):
            return 'OCCUPIED-ONLY PLAN — UNKNOWN SPACE WAS NOT CHECKED'
        return 'SAVED PLAN — checked against an older static scene, NOT the live scene'

    def positions(self,t):
        if self.retimed is not None:
            # Evaluate the original curve with its new clock, rather than
            # cutting across curved sections by interpolating preview samples.
            end=self.retimed.duration/self.preview_rate;hold=float(self.obj['hold_at_goal_s'])
            if t<=0 or t>=self.duration:return np.array(self.req['start'])
            if end<=t<=end+hold:return np.array(self.req['goal'])
            outward=t if t<end else 2*end+hold-t
            return self.retimed(outward*self.preview_rate)[0]
        return np.array([np.interp(t,self.times,self.values[:,j]) for j in range(7)])

    def frames(self,t):
        return self.model.transforms(dict(self.static,**dict(zip(self.joints,self.positions(t)))))

    def phase(self,t):
        i=int(np.clip(np.searchsorted(self.times,t,side='right')-1,0,len(self.curves)-1))
        if self.playable:
            end=float(self.obj['outward'][-1]['time_s'])
            if end<=t<=end+float(self.obj['hold_at_goal_s']):return 'HOLD (2 seconds)'
        return self.curves[i]['phase']


class PlaybackClock:
    """Animation time advances by real elapsed time, not by rendered frames."""
    def __init__(self,duration,now=time.monotonic):
        self.duration=float(duration);self.now=now;self.position=0.;self.playing=False
        self.rate=1.;self.loop=False;self.last=now();self.completed_cycles=0

    def sample(self):
        now=self.now();elapsed=max(0.,now-self.last);self.last=now
        if self.playing:
            self.position+=elapsed*self.rate
            if self.position>=self.duration:
                if self.loop and self.duration>0:
                    self.completed_cycles+=int(self.position//self.duration)
                    self.position%=self.duration
                else:
                    self.position=self.duration;self.playing=False;self.completed_cycles+=1
        return self.position

    def play(self):
        self.sample()
        if self.duration<=0:return
        if self.position>=self.duration:self.position=0.
        self.playing=True;self.last=self.now()

    def pause(self):
        self.sample();self.playing=False

    def seek(self,t):
        if not np.isfinite(t):raise ValueError('Animation time must be finite')
        self.position=float(np.clip(t,0,self.duration));self.last=self.now()

    def set_rate(self,rate):
        if not np.isfinite(rate) or rate<=0:raise ValueError('Animation speed must be positive')
        self.sample();self.rate=float(rate)


class RobotArtists:
    """Solid URDF visuals, separate from camera meshing and collision geometry."""
    def __init__(self,ax,plan):
        from .triangle_artist import TriangleCollection
        self.ax=ax;self.plan=plan;self.mesh_entries=[];self._shading=[];self._transforms={}
        self.stick_length=stick_length(plan.root,plan.stick_link)
        for name,triangles,local,colors in visual_surfaces(plan.root):
            poly=TriangleCollection([],alpha=1.,edgecolor='none',linewidths=0,antialiaseds=False)
            ax.add_collection3d(poly);self.mesh_entries.append((name,triangles,local,poly))
            normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
            normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-20)
            self._shading.append((normals,colors))
            if name==plan.stick_link:self.stick_artist=poly
        self.last_positions=None;self.last_tip=None

    def set_visible(self,visible):
        for _,_,_,poly in self.mesh_entries:poly.set_visible(visible)

    def draw(self,t):
        frames=self.plan.frames(t)
        self.last_positions=self.plan.positions(t)
        T=frames[self.plan.stick_link];length=self.stick_length
        self.last_tip=T[:3,3]+T[:3,2]*length/2
        light=np.array([-.3,-.5,1.]);light/=np.linalg.norm(light)
        for i,(name,triangles,local,poly) in enumerate(self.mesh_entries):
            T=frames[name]@local
            if i in self._transforms and np.array_equal(T,self._transforms[i]):continue
            poly.set_verts(triangles@T[:3,:3].T+T[:3,3])
            normals,colors=self._shading[i];shaded=colors.copy()
            # Vendor visual faces include mixed winding; two-sided lighting
            # avoids alternating dark triangles on a single flat housing.
            shaded[:,:3]*=(.8+.2*np.abs((normals@T[:3,:3].T)@light))[:,None]
            poly.set_facecolors(shaded);self._transforms[i]=T.copy()

    def focus_bounds(self):
        # Viewing bounds only, never an obstacle or motion constraint.
        all_points=[]
        for t in np.linspace(0,self.plan.duration,25):
            frames=self.plan.frames(t)
            all_points.extend(T[:3,3] for T in frames.values())
            T=frames[self.plan.stick_link]
            all_points.append(T[:3,3]+T[:3,2]*self.stick_length/2)
        points=np.array(all_points)
        lo=np.minimum(points.min(axis=0)-.25,[-.65,-.9,-.35])
        hi=np.maximum(points.max(axis=0)+.35,[1.3,.8,1.15])
        return np.array([lo,hi])
