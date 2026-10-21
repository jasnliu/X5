import json,sys,pathlib,xml.etree.ElementTree as ET
import numpy as np
ROOT=pathlib.Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import common,planning,workflow,style
from drum_planner.execution import Curve
from drum_planner.trajectory_view import PlanDisplay
D=pathlib.Path(__file__).parent
reports={}
for name in ['saved_scene_plan','saved_scene_12']:
 folder=D/name;p=json.load(open(folder/'plan.json'));r=json.load(open(folder/'backend_request.json'))
 assert p['status']=='planned' and not p['motion_authorized'] and not p['hardware_execution_available']
 assert p['outward'][0]['positions_rad']==r['start']==p['cycle'][-1]['positions_rad']
 assert p['outward'][-1]['positions_rad']==common.endpoint().tolist()==r['goal']
 assert p['hold_at_goal_s']==2 and p['reverse_checked_on_same_static_scene']
 assert p['cycle'][-1]['time_s']==2*p['duration_outward_s']+2
 assert workflow.simulate(p)['motor_commands']==0
 assert p['joint_limit_check']['valid'] and not p['joint_limit_check']['angles_clamped']
 tree=ET.parse(r['urdf']).getroot();curve=Curve(p);peak=[]
 for namej,pp in zip(common.JOINTS,curve.curves):
  limits=tree.find(f"joint[@name='{namej}']/limit");lo=float(limits.get('lower'));hi=float(limits.get('upper'))
  roots=pp.derivative().roots(extrapolate=False);roots=roots[np.isfinite(roots)&(roots>=pp.x[0])&(roots<=pp.x[-1])]
  values=pp(np.r_[pp.x,roots]);assert values.min()>=lo-1e-9 and values.max()<=hi+1e-9
  peaks=[planning.extrema(pp,order)/curve.scale**order for order in [1,2,3]]
  assert np.all(np.array(peaks)<=[.6,1.2,6.]);peak.append(peaks)
 opt=p['optimization'];assert opt['objective']=='broad_route_style_v1' and opt['weights']==style.WEIGHTS
 assert opt['selected_duration_s']<=opt['duration_budget_s']
 selected=opt['candidates'][opt['selected_candidate']-1]
 variants=selected['variants'];v=next(v for v in variants if v['status']=='selected');assert v['passed']
 for c in opt['candidates']:
  for alt in c['variants']:
   if alt.get('style_score',{}).get('total',float('inf'))<v['style_score']['total']-1e-8:
    assert alt['status']=='rejected_collision_check' and not alt['passed']
 check=json.load(open(folder/selected['backend_folder']/('interpolated_check_'+str(int(v['stop_at_corners']))+'_result.json')))
 assert check['status']=='checked' and all(row['valid'] is True for row in check['checks'])
 assert len(check['checks'])==p['sampled_interpolated_checks']==v['checked_samples']
 assert r['scene']['obstacle_padding_m']==.02 and r['drumstick_self_filter_padding_m']==.029
 assert r['filter_drumstick_depth_shadow'] and r['collision_policy']=='observed_obstacles'
 assert p['calibration_sha256']==common.digest(common.DEFAULT_CALIBRATION)
 assert p['recording_sha256']==common.digest(common.DEFAULT_RECORDING)
 display=PlanDisplay(folder/'plan.json');assert display.playable
 reports[name]=dict(passed=True,selected_candidate=opt['selected_candidate'],selected_seed=opt['selected_seed'],
  selected_score=opt['selected_score'],outward_s=p['duration_outward_s'],cycle_s=display.duration,
  reference=opt['fastest_valid_reference'],all_dense_checks_passed=len(check['checks']),
  full_continuous_polynomial_within_limits=True,maximum_derivatives=np.max(peak,axis=0).tolist(),
  exact_start_goal_hold_return=True,motor_commands=0,camera_access=False,can_access=False)
(D/'saved_plan_verification.json').write_text(json.dumps(reports,indent=2));print(json.dumps(reports,indent=2))
