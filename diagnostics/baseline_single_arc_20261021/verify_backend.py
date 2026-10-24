"""Real new backend + native FCL, using frozen inputs only. Cannot authorize motion."""
import json,sys,time
from pathlib import Path
import numpy as np
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,atomic_json,digest
from drum_planner.experiment.config import saved_inputs,no_can
from drum_planner.baseline import planner,tasks,paths
from drum_planner.baseline.curve import Curve
from drum_planner.trajectory_view import PlanDisplay


def run(folder):
    no_can();folder=Path(folder);state,scene=saved_inputs()
    obj=planner.plan(folder/'result',state,scene,DEFAULT_CALIBRATION)
    assert obj['status']=='planned' and obj['route_planner']['method']=='single_arc'
    assert not obj['actual_current_robot_state_verified'] and not obj.get('baseline_hardware_session')
    path=folder/'result/plan.json';p=PlanDisplay(path);c=Curve(obj);paths.validate_timing(c)
    grid=np.r_[np.arange(0,c.duration,.002),c.duration];q=c.at_many(grid)
    proof=tasks.hard_check(p.req,q,folder/'hardware_curve_check')
    assert np.max(abs(q[0]-p.req['start']))<1e-10 and np.max(abs(q[-1]-p.req['goal']))<1e-10
    for t in np.linspace(0,c.duration,501):
        assert np.allclose(c.at(t),p.positions(t),atol=1e-12)
        assert np.allclose(c.at(t),p.positions(p.duration-t),atol=1e-12)
    for fraction in (.2,.6,1.):
        tau=c.duration*fraction;spec=paths.recovery(c,c.at(tau),tau,1.)
        tasks.hard_check(p.req,paths.recovery_samples(c,spec),folder/('return_'+str(fraction)))
    # Must remain a software artifact: neither an experiment output nor this
    # offline backend test substitutes for a real Calculate acquisition.
    try:tasks.load_checked(path,DEFAULT_CALIBRATION)
    except ValueError as exc:assert 'newly acquired' in str(exc)
    else:raise AssertionError('Offline artifact accepted as hardware acquisition')
    frozen=json.loads((ROOT/'diagnostics/corner_smoothing_20261021/inputs/right_single_arc.json').read_text())
    assert obj['path_waypoints']==frozen['path_waypoints']
    for key in ('kind','duration_s','path','density'):
        assert obj['timing'][key]==frozen['v1_law'][key],key
    per_call=[]
    for t in np.linspace(0,c.duration,1001):
        begin=time.perf_counter();c.at(float(t));per_call.append(time.perf_counter()-begin)
    result=dict(passed=True,plan=str(path.resolve()),method='single_arc',duration_outward_s=c.duration,
        identical_to_experiment_right_single_arc=True,hardware_curve_proof=proof,
        prefix_returns_checked=[.2,.6,1.],exact_preview_execution_parity=True,
        evaluator_median_ms=float(np.median(per_call)*1000),evaluator_p99_ms=float(np.percentile(per_call,99)*1000),
        software_only=True,can_opened=False,camera_opened=False,physical_motor_commands=0)
    atomic_json(folder/'verification.json',result);print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':run(sys.argv[1])
