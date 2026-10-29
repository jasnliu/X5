"""Independent exact-extrema and geometric comparison; offline saved plans only."""
import json,sys,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.interpolate import PPoly
from drum_planner.common import atomic_json
from drum_planner.timing import core
from drum_planner.fast_mode.kinematics import Kinematics
from drum_planner.experiment.fluidity import reversals,curve_shape


def measure(folder):
    obj=json.loads((folder/'plan.json').read_text());req=json.loads((folder/'backend_request.json').read_text())
    ev=core.Evaluator(obj['timing']);p=ev.path if obj['timing']['kind']=='time_density' else ev.p
    end=1. if obj['timing']['kind']=='time_density' else ev.duration
    q=p(np.linspace(0,end,4001));kin=Kinematics(req['urdf'],obj['moving_side']);fk=kin.fk(q)
    lo=[];hi=[]
    for j in range(7):
        f=PPoly(p.c[:,:,j],p.x,extrapolate=False);roots=f.derivative().roots(extrapolate=False)
        roots=roots[np.isfinite(roots)&(roots>=0)&(roots<=end)]
        v=f(np.r_[p.x[(p.x>=0)&(p.x<=end)],roots]);lo.append(float(v.min()));hi.append(float(v.max()))
    row=dict(duration_s=obj['duration_outward_s'],goal_deg=np.degrees(req['goal']).tolist(),
        minimum_deg=np.degrees(lo).tolist(),maximum_deg=np.degrees(hi).tolist(),
        reversals=reversals(q,np.radians(2.)),total_joint_travel_deg=float(np.degrees(np.abs(np.diff(q,axis=0)).sum())),
        weighted_joint_travel_rad=float((np.abs(np.diff(q,axis=0))*[1.2,1.2,1.2,1.,.35,.35,.35]).sum()),
        wrist=curve_shape(fk['wrist'],.05),tip=curve_shape(fk['tip'],.05),
        continuous_bounds_valid=bool(np.all(np.asarray(lo)>=kin.lower-1e-12) and np.all(np.asarray(hi)<=kin.upper+1e-12)))
    if obj.get('dt1_generation'):
        goal=json.loads(Path(obj['destination_bindings'][0]['goal_recording']).read_text());row['geometry']=goal['target_geometry']
        row['target_world_m']=goal['target_world_m']
    return row,q


def main():
    base=Path(__file__).resolve().parent;rows={};curves={}
    for name in ('baseline_fault','updated_fault','final_fault','old_r1','new_r1','old_r2','new_r2','old_left','new_left'):
        rows[name],curves[name]=measure(base/name/'hardware')
        assert rows[name]['continuous_bounds_valid']
        if name.startswith(('final','new_')) and name!='new_left':assert rows[name]['minimum_deg'][1]>=-1e-10
    for old,new in [('baseline_fault','final_fault'),('old_r1','new_r1'),('old_r2','new_r2')]:
        a=json.loads((base/old/'hardware/backend_request.json').read_text());b=json.loads((base/new/'hardware/backend_request.json').read_text())
        assert a['start']==b['start'] and a['left']==b['left'] and a['scene']==b['scene']
        ar=ET.parse(a['urdf']).getroot();br=ET.parse(b['urdf']).getroot()
        for r in (ar,br):r.find("joint[@name='openarmx_right_joint2']/limit").attrib.pop('lower')
        assert ET.tostring(ar)==ET.tostring(br)
    assert rows['baseline_fault']['target_world_m']==rows['final_fault']['target_world_m']
    atomic_json(base/'quality_report.json',dict(software_only=True,comparison=rows,
        same_scene_start_calibration_geometry=True,only_model_difference='right J2 lower position bound',
        note='Geometric quality measures are heuristics, not a physical or aesthetic certification.'))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,2,figsize=(12,10),sharex=True)
    for j,ax in enumerate(axes.flat[:7]):
        for name,label in [('baseline_fault','Before: infeasible on actual J2'),('final_fault','Corrected operating range')]:
            ax.plot(np.linspace(0,1,len(curves[name])),np.degrees(curves[name][:,j]),label=label)
        ax.set_ylabel(f'J{j+1} (degrees)');ax.grid(alpha=.3)
        if j==1:ax.axhline(0,color='black',ls='--');ax.axhline(-1,color='red',ls=':',label='Reported approximate stop')
    axes.flat[-1].axis('off');axes.flat[0].legend(fontsize=8)
    for ax in axes[-1]:ax.set_xlabel('Geometric path progress (not time)')
    fig.suptitle('Same actual saved scene/start/green target; right J2 limit correction');fig.tight_layout()
    fig.savefig(base/'joint_paths.png',dpi=140);plt.close(fig)
    for name,row in rows.items():print(name,round(row['duration_s'],6),[round(row['minimum_deg'][1],4),round(row['maximum_deg'][1],4)],row['reversals']['count'],round(row['wrist']['length'],4),round(row['tip']['length'],4))


if __name__=='__main__':main()
