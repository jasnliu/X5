"""Fresh software planning on the incident capture and both saved experiments."""
import json,sys
from pathlib import Path
import numpy as np
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can,settings,saved_inputs,METHODS
no_can()
from drum_planner import depth
# Fail even if future code accidentally starts acquisition.
def forbidden(*a,**k):raise AssertionError('No live camera permitted')
depth.DepthCamera=forbidden
from drum_planner.baseline import planner,tasks,paths
from drum_planner.baseline.curve import Curve
from drum_planner.experiment import worker
from drum_planner.trajectory_view import PlanDisplay
from drum_planner.timing import core,motion_quality


def check(path,output):
    p=PlanDisplay(path);obj=p.obj;c=Curve(obj);paths.validate_timing(c)
    q=c.at_many(np.r_[np.arange(0,c.duration,.002),c.duration])
    if p.side=='right':proof=tasks.hard_check(p.req,q,output/'full_curve')
    else:proof=obj['final_curve_check']
    assert np.allclose(q[0],p.req['start'],atol=1e-10) and np.allclose(q[-1],p.req['goal'],atol=1e-10)
    for t in np.linspace(0,c.duration,101):
        assert np.allclose(c.at(t),p.positions(t),atol=1e-11)
        assert np.allclose(c.at(t),p.positions(p.duration-t),atol=1e-11)
    rep=obj.get('route_planner',obj.get('experiment'))['report'];row=rep['candidates'][0]
    quality=motion_quality.law_metrics(obj['timing'])
    if rep['method']=='single_arc':
        assert rep['normalization']['selection_policy']=='continuous_arc_first_v1'
        assert row['selection_class']=='continuous_arc' and not row['stop_at_corners']
        assert quality['internal_stops']==0,quality
    return dict(plan=str(Path(path).resolve()),selected=row['index'],source=row['source'],
                selection_class=row.get('selection_class'),quality=quality,collision_check=proof)


def main(folder):
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False);rows={}
    old=ROOT/'planner_runs/20261021T072213Z-baseline-f0f65f/calculate-34c5a435/plan/result/plan.json'
    oldobj=json.loads(old.read_text());state=Path(oldobj['provenance']['state']);scene=Path(oldobj['provenance']['scene'])
    print('=== FRESH BASELINE: incident start/scene; no hardware acquisition ===',flush=True)
    p=planner.plan(folder/'baseline',state,scene,DEFAULT_CALIBRATION)
    assert p['status']=='planned' and not p['actual_current_robot_state_verified']
    rows['baseline_incident']=check(folder/'baseline/plan.json',folder/'baseline_checks')
    assert rows['baseline_incident']['selected']==12
    assert p['duration_outward_s']<oldobj['duration_outward_s']
    c=Curve(p);r=json.loads((folder/'baseline/backend_request.json').read_text())
    for fraction in (.2,.6,1.):
        tau=c.duration*fraction;spec=paths.recovery(c,c.at(tau),tau,1.)
        tasks.hard_check(r,paths.recovery_samples(c,spec),folder/f'return_{fraction}')
    try:tasks.load_checked(folder/'baseline/plan.json',DEFAULT_CALIBRATION)
    except ValueError:pass
    else:raise AssertionError('Offline plan accepted for hardware')
    print('=== EXPERIMENT: exactly same incident inputs and validated candidate pool ===',flush=True)
    session=folder/'matched_experiment';context=worker.initialize(session,state,scene,DEFAULT_CALIBRATION)
    context['pool_root']=str(folder/'baseline/candidate_pool');atomic_json(session/'context.json',context)
    e=worker.plan(context,session/'plans/single_arc', 'single_arc',settings(),120.)
    assert e['path_waypoints']==p['path_waypoints'] and e['timing']==p['timing']
    assert e['experiment']['report']['pool_reused']
    rows['experiment_incident']=check(session/'plans/single_arc/plan.json',folder/'matched_checks')
    for side in ('right','left'):
        print(f'=== FRESH {side.upper()} EXPERIMENT: all four choices ===',flush=True)
        s,sc=saved_inputs(side);session=folder/('experiment_'+side)
        context=worker.initialize(session,s,sc,DEFAULT_CALIBRATION,side=side)
        outcomes={}
        for method in METHODS:
            target=session/'plans'/method
            result=worker.plan(context,target,method,settings(),120.)
            assert result['status']=='planned',(side,method,result)
            outcomes[method]=dict(status='planned',plan=str(target/'plan.json'))
            row=check(target/'plan.json',folder/(side+'_'+method+'_checks'))
            rows[side+'_'+method]=row
            if side=='right' and method=='single_arc':
                olde=json.loads((ROOT/'planner_runs/20261021T072410Z-experiment-c2a053/plans/single_arc-339171fb/plan.json').read_text())
                assert result['path_waypoints']==olde['path_waypoints'] and result['timing']['path']==olde['timing']['path']
                assert result['timing']['density']==olde['timing']['density']
        atomic_json(session/'results.json',outcomes)
        atomic_json(folder/'partial.json',rows)
    report=dict(passed=True,software_only=True,can_opened=False,camera_opened=False,physical_motor_commands=0,
                same_input_baseline_experiment_exact_parity=True,original_incident_duration_s=oldobj['duration_outward_s'],
                original_incident_quality=motion_quality.law_metrics(oldobj['timing']),rows=rows)
    atomic_json(folder/'report.json',report)
    for name,row in rows.items():print(name,row['selected'],row['source'],row['quality']['duration_s'],row['quality']['internal_stops'],flush=True)
    print('ALL FRESH BACKEND CHECKS PASSED',flush=True)

if __name__=='__main__':main(sys.argv[1])
