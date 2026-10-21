"""Independent polynomial/old-file-backend verification; no device imports."""
from pathlib import Path
import json,xml.etree.ElementTree as ET
import numpy as np
from scipy.interpolate import PPoly
from drum_planner import common,planning,workflow
from drum_planner.trajectory_view import PlanDisplay
from drum_planner.experiment.config import no_can
no_can();D=Path(Path('/tmp/x5_fluidity_task').read_text().strip())
results=json.loads((D/'ui/results.json').read_text());reports={}
for method,row in results.items():
 path=Path(row['plan']);p=json.loads(path.read_text());r=json.loads((path.parent/'backend_request.json').read_text())
 assert p['experiment_only'] and not p['motion_authorized'] and not p['hardware_execution_available']
 assert not p['actual_current_robot_state_verified'] and not p['can_access'] and p['motor_commands']==0
 assert common.digest(p['provenance']['state'])==p['provenance']['state_sha256']
 assert common.digest(p['provenance']['scene'])==r['scene']['source_scene_sha256']
 display=PlanDisplay(path)
 if p['status']!='planned':
  assert p['status']=='no_feasible_route' and not display.playable
  reports[method]=dict(passed=True,normal_blocked_result=p['details']['reason']);continue
 assert p['outward'][0]['positions_rad']==r['start']==p['cycle'][-1]['positions_rad']
 assert p['outward'][-1]['positions_rad']==common.endpoint().tolist()==r['goal']
 assert p['hold_at_goal_s']==2 and p['reverse_checked_on_same_static_scene']
 assert abs(p['cycle'][-1]['time_s']-(2*p['duration_outward_s']+2))<1e-9
 assert workflow.simulate(p)['motor_commands']==0
 timing=p['timing'];scale=timing['time_scale'];duration=timing['duration_s'];tree=ET.parse(r['urdf']).getroot()
 curves=[PPoly(np.array(c),np.array(x),extrapolate=False) for c,x in zip(timing['coefficients'],timing['polynomial_breaks_unscaled_s'])]
 peaks=[]
 for name,pp in zip(common.JOINTS,curves):
  limit=tree.find(f"joint[@name='{name}']/limit")
  roots=pp.derivative().roots(extrapolate=False);roots=roots[np.isfinite(roots)&(roots>=pp.x[0])&(roots<=pp.x[-1])]
  values=pp(np.r_[pp.x,roots]);assert values.min()>=float(limit.get('lower'))-1e-9 and values.max()<=float(limit.get('upper'))+1e-9
  peak=[planning.extrema(pp,k)/scale**k for k in (1,2,3)];assert np.all(np.array(peak)<=[.6,1.2,6.]);peaks.append(peak)
 grid=np.r_[np.arange(0,duration,.002),duration];q=np.column_stack([c(grid/scale) for c in curves]);q[0]=r['start'];q[-1]=r['goal']
 check=dict(r,check_states=q.tolist(),check_path=True,stop_on_invalid=True)
 result=planning.backend(check,D/'independent_fcl'/method)
 assert result['status']=='checked' and len(result['checks'])==len(q) and all(x['valid'] for x in result['checks']),method
 assert p['joint_limit_check']['valid'] and not p['joint_limit_check']['angles_clamped']
 reports[method]=dict(passed=True,original_file_backend_dense_checks=len(q),polynomial_bounds=True,
    derivative_peaks=np.max(peaks,axis=0).tolist(),exact_start_goal_hold_return=True,outward_s=duration,
    scene_sha256=r['scene']['source_scene_sha256'],motor_commands=0)
 print(method,'verified',len(q),'states',flush=True)
common.atomic_json(D/'independent_verification.json',reports)
print('ALL INDEPENDENT CHECKS PASSED',flush=True)
