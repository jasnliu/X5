import json,sys
from pathlib import Path
import numpy as np
ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import planning,common,workflow
from drum_planner.execution import Curve
from drum_planner.trajectory_view import PlanDisplay
D=Path(__file__).parent;folder=D/'final_saved_scene_plan'
plan=json.loads((folder/'plan.json').read_text());req=json.loads((folder/'backend_request.json').read_text())
old=json.loads((ROOT/'diagnostics/observed_obstacle_policy_20261018T201044Z/live_current_pose/plan/plan.json').read_text())
assert plan['status']=='planned' and plan['motion_authorized'] is False and plan['hardware_execution_available'] is False
assert plan['calibration_sha256']==old['calibration_sha256']==common.digest(common.DEFAULT_CALIBRATION)
assert plan['recording_sha256']==old['recording_sha256']==common.digest(common.DEFAULT_RECORDING)
assert req['start']==plan['start_state']['right_rad']==old['start_state']['right_rad']
assert req['goal']==common.endpoint().tolist()
opt=plan['optimization'];chosen=opt['selected_candidate'];duration=plan['duration_outward_s']
assert opt['searched_candidates']==opt['requested_candidates']==6
assert len({r['seed'] for r in opt['candidates']})==6
assert json.loads((folder/'optimization.json').read_text())==opt
for row in opt['candidates']:
    trial=json.loads((folder/row['backend_folder']/'backend_request.json').read_text())
    for name in ('start','goal','left','left_opening_m','scene','collision_policy','occupied_only_preview','execute','urdf','srdf'):
        assert trial[name]==req[name]
    raw=json.loads((folder/row['backend_folder']/'backend_result.json').read_text())
    for v in row['variants']:
        if 'duration_outward_s' not in v:continue
        points,check,timing=planning.smooth_samples(raw['path_waypoints'],v['stop_at_corners'])
        assert timing['duration_s']==v['duration_outward_s']
        if v['status']=='not_checked_slower_or_equal':assert v['duration_outward_s']>=duration
        elif v['status']=='rejected_collision_check':
            assert v['duration_outward_s']<=duration and not v['passed']
        elif v['status']=='selected':
            assert row['index']==chosen and v['duration_outward_s']==duration
            assert points==plan['outward'] and timing==plan['timing']
            checks=json.loads((folder/row['backend_folder']/f"interpolated_check_{int(v['stop_at_corners'])}_result.json").read_text())
            assert checks['status']=='checked' and len(checks['checks'])==len(check)
            assert all(r['valid'] is True for r in checks['checks'])
curve=Curve(plan)
for order,limit in ((1,.6),(2,1.2),(3,6.)):
    assert max(planning.extrema(c,order)/curve.scale**order for c in curve.curves)<=limit
assert np.allclose(curve.at(0),req['start'],atol=1e-10,rtol=0)
assert np.allclose(curve.at(curve.duration),req['goal'],atol=1e-10,rtol=0)
assert workflow.simulate(plan)['motor_commands']==0
view=PlanDisplay(folder/'plan.json');assert view.playable
for t,q in [(0,req['start']),(duration,req['goal']),(duration+1,req['goal']),(view.duration,req['start'])]:
    assert np.array_equal(view.positions(t),q)
report=dict(passed=True,tests='all candidate reports, final-duration selection, actual FCL check evidence, exact polynomial/limits, endpoints, hold and reverse',
    candidate_count=6,selected_seed=opt['selected_seed'],planning_wall_s=opt['wall_time_s'],
    previous_outward_s=old['duration_outward_s'],selected_outward_s=duration,
    outward_time_reduction_percent=100*(1-duration/old['duration_outward_s']),cycle_duration_s=view.duration,
    full_selected_interpolated_checks=plan['sampled_interpolated_checks'],
    calibration_recording_and_state_unchanged=True,physical_motion_tested=False,live_camera_used=False)
(D/'saved_scene_verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
