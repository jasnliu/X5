"""Read-only saved-plan data, wall-clock animation, and reusable robot artists."""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .common import JOINTS, LEFT_JOINTS
from .model import stl_vertices
from camera_calibration.core import digest
from safe_zone.geometry import Model


class PlanDisplay:
    def __init__(self, path):
        self.path=Path(path).resolve();self.obj=json.loads(self.path.read_text())
        self.req=json.loads((self.path.parent/'backend_request.json').read_text())
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
        for name in ('start','left','goal'):
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
        self.static=dict(zip(LEFT_JOINTS,self.req['left']))
        self.static['openarmx_left_finger_joint1']=self.req['left_opening_m']

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
        return np.array([np.interp(t,self.times,self.values[:,j]) for j in range(7)])

    def frames(self,t):
        return self.model.transforms(dict(self.static,**dict(zip(JOINTS,self.positions(t)))))

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
    """Same generated robot/tool geometry in saved and live-depth previews."""
    def __init__(self,ax,plan,mesh_budget=350):
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        self.ax=ax;self.plan=plan;self.mesh_entries=[]
        for link in plan.root.findall('link'):
            name=link.get('name')
            for collision in link.findall('collision'):
                geom=collision.find('geometry');mesh=geom.find('mesh')
                if mesh is None:continue
                filename=mesh.get('filename').removeprefix('file://')
                if not filename.lower().endswith('.stl'):continue
                triangles=stl_vertices(filename).astype(float)*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
                triangles=triangles[::max(1,int(np.ceil(len(triangles)/mesh_budget)))]
                origin=collision.find('origin');local=np.eye(4)
                if origin is not None:
                    local[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                    local[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
                color='darkorange' if 'right' in name else 'slategray'
                poly=Poly3DCollection([],alpha=.65,facecolor=color,edgecolor='none')
                ax.add_collection3d(poly);self.mesh_entries.append((name,triangles,local,poly))
        self.right_line,=ax.plot([],[],[],color='darkorange',linewidth=3)
        self.left_line,=ax.plot([],[],[],color='slategray',linewidth=2)
        self.stick_line,=ax.plot([],[],[],color='crimson',linewidth=4)
        self.last_positions=None;self.last_tip=None

    def draw(self,t):
        frames=self.plan.frames(t)
        self.last_positions=self.plan.positions(t)
        for side,line in [('right',self.right_line),('left',self.left_line)]:
            pts=np.array([frames[f'openarmx_{side}_link{i}'][:3,3] for i in range(8)]+
                         [frames[f'openarmx_{side}_hand'][:3,3]])
            line.set_data_3d(*pts.T)
        T=frames['right_drumstick'];length=self.plan.obj['model']['stick_length_from_mount_m']
        tips=np.array([[0,0,-length/2],[0,0,length/2]])@T[:3,:3].T+T[:3,3]
        self.stick_line.set_data_3d(*tips.T);self.last_tip=tips[-1]
        for name,triangles,local,poly in self.mesh_entries:
            T=frames[name]@local;poly.set_verts(triangles@T[:3,:3].T+T[:3,3])

    def focus_bounds(self):
        # Viewing bounds only, never an obstacle or motion constraint.
        all_points=[]
        for t in np.linspace(0,self.plan.duration,25):
            frames=self.plan.frames(t)
            all_points.extend(T[:3,3] for T in frames.values())
            T=frames['right_drumstick']
            all_points.append(T[:3,3]+T[:3,2]*self.plan.obj['model']['stick_length_from_mount_m']/2)
        points=np.array(all_points)
        lo=np.minimum(points.min(axis=0)-.25,[-.65,-.9,-.35])
        hi=np.maximum(points.max(axis=0)+.35,[1.3,.8,1.15])
        return np.array([lo,hi])
