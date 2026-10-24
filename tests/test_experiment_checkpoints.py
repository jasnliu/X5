"""Checkpoint orchestration and exact animation; no camera or motor access."""
from test_checkpoint_performance import OPTIMIZED
import copy
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.interpolate import PPoly

from drum_planner.common import ROOT,atomic_json,digest,read_state,goal_recording
from drum_planner.experiment import checkpoints as cp,config,worker,app
from drum_planner.timing import core
from drum_planner.planning import cycle

# These archived scenes predate the camera move. Never relabel them with a new
# live default; new-default routing is covered by test_calibration_default.py.
ARCHIVE_CALIBRATION = ROOT/'calibration_sessions/20261017T211949Z-762f9908-reprocessed/calibration.json'


@pytest.mark.parametrize('side',['right','left'])
def test_freeze_one_preserves_legacy_and_adjacent_only_exclusion(tmp_path,side):
    a=tmp_path/'a.json';b=tmp_path/'b.json'
    a.write_bytes(goal_recording(side).read_bytes());b.write_bytes(a.read_bytes())
    one=cp.freeze([a],side,tmp_path)
    assert set(one)=={'goal_recording','destination_source','destination_sha256'}
    multiple=cp.freeze([a,b,a],side,tmp_path)
    assert len(multiple['checkpoints'])==3
    assert multiple['checkpoint_sha256']==cp.signature(multiple['checkpoints'])
    for paths in ([],[a,b,a,b],[a,a]):
        with pytest.raises(ValueError):cp.freeze(paths,side,tmp_path)


@pytest.fixture(params=['right','left'])
def sequence(tmp_path,request):
    side=request.param;state,scene=config.saved_inputs(side)
    context=worker.initialize(tmp_path/'session',state,scene,ARCHIVE_CALIBRATION,side)
    records=[];original=read_state(state);q=np.array(original[side+'_rad'])
    for i in range(3):
        record=json.loads(goal_recording(side).read_text())
        record['samples'][-1]['positions_rad']=(q+(i+1)*.01).tolist()
        path=tmp_path/f'pose{i}.json';atomic_json(path,record);records.append(path)
    context.update(cp.freeze(records,side,tmp_path/'session'))
    template=json.loads((tmp_path/'session/initial/plan.json').read_text())
    base_req=json.loads((tmp_path/'session/initial/backend_request.json').read_text())
    calls=[]
    def leg(local,folder,method,params,seconds):
        folder=Path(folder);folder.mkdir(exist_ok=True,parents=True)
        state=read_state(local['state']);start=np.array(state[side+'_rad'])
        goal=np.array(worker.goal(local)[1]);duration=1.7;d=goal-start
        coefficients=np.array([6*d/duration**5,-15*d/duration**4,10*d/duration**3,
                               np.zeros(7),np.zeros(7),start])[:,None,:]
        law=dict(schema='x5-fixed-path-timing-v1',kind='joint_polynomial',duration_s=duration,
                 curve=core.encode(PPoly(coefficients,[0.,duration])))
        times=np.linspace(0,duration,21);values=core.Evaluator(law)(times)
        outward=[dict(time_s=float(t),positions_rad=q.tolist(),velocities_rad_s=v.tolist(),
                      accelerations_rad_s2=a.tolist()) for t,q,v,a in zip(times,*values[:3])]
        outward[0]['positions_rad']=start.tolist();outward[-1]['positions_rad']=goal.tolist()
        result=copy.deepcopy(template)
        result.update(status='planned',outward=outward,cycle=cycle(outward),timing=law,
            duration_outward_s=duration,hold_at_goal_s=2.,start_state=state,
            recording_sha256=local['destination_sha256'],final_curve_check={'valid':True,'samples_checked':21},
            sampled_interpolated_checks=21,experiment={'method':method,'report':{},'retiming':{}})
        req=dict(base_req,start=start.tolist(),goal=goal.tolist())
        if 'scene_robot_start' in local:req['scene_robot_start']=local['scene_robot_start']
        atomic_json(folder/'backend_request.json',req);atomic_json(folder/'plan.json',result)
        calls.append((copy.deepcopy(local),req))
        return result
    return context,leg,calls,tmp_path/'result'


def test_sequence_starts_holds_new_return_and_passive_arm(sequence):
    context,leg,calls,folder=sequence;side=context['moving_side'];other='left' if side=='right' else 'right'
    result=cp.plan_sequence(context,folder,'workspace_curve',config.settings(),120,leg)
    assert len(calls)==4 and result['status']=='planned'
    initial=read_state(context['state'])
    assert calls[0][0]['state']==context['state'] and 'scene_robot_start' not in calls[0][1]
    for i,(local,req) in enumerate(calls):
        assert req[other]==initial[other+'_rad']
        if i:
            assert req['start']==calls[i-1][1]['goal']
            assert req['scene_robot_start']==initial[side+'_rad']
            assert 'motor_telemetry' not in read_state(local['state'])
    plan=cp.ExperimentPlanDisplay(folder/'plan.json')
    assert plan.duration==pytest.approx(12.8) # 3*1.7 + 2*2 outward, final hold, new 1.7s return
    assert result['duration_outward_s']==pytest.approx(9.1)
    assert np.all(np.diff(plan.times)>0)
    for segment,evaluator in plan.segments:
        assert np.array_equal(plan.positions(segment['arrival_time_s']),segment['goal'])
        for t in np.linspace(0,evaluator.duration,7)[1:-1]:
            forward=segment['start_time_s']+t
            assert np.allclose(plan.positions(forward),evaluator(t)[0],atol=1e-12)
        for dt in (.001,.7,1.999):
            held=segment['arrival_time_s']+dt
            assert np.array_equal(plan.positions(held),segment['goal'])
            assert 'HOLD' in plan.phase(held)
    return_leg,evaluator=plan.return_leg
    assert calls[-1][1]['start']==calls[-2][1]['goal']
    assert calls[-1][1]['goal']==initial[side+'_rad']
    assert len(result['final_curve_check']['legs'])==4
    assert 'reverse_checked_on_same_static_scene' not in result
    assert result['checkpoint_sequence']['return_policy']==cp.RETURN_POLICY
    for t in np.linspace(0,evaluator.duration,50)[1:-1]:
        absolute=return_leg['start_time_s']+t
        assert np.allclose(plan.positions(absolute),evaluator(t)[0],atol=1e-12)
        assert plan.phase(absolute)=='RETURN TO ORIGINAL START'
    assert sum(p['phase']=='hold_complete' for p in result['cycle'])==1
    assert np.array_equal(plan.positions(0),initial[side+'_rad'])
    assert np.array_equal(plan.positions(plan.duration),initial[side+'_rad'])
    assert not result['motion_authorized'] and not result['hardware_execution_available']


def test_failed_leg_never_presents_partial_route_and_can_retry(sequence):
    context,leg,calls,folder=sequence
    def blocked(*args):
        result=leg(*args)
        if len(calls)==2:result.update(status='no_feasible_route',details={'reason':'test obstacle'})
        return result
    failed=cp.plan_sequence(context,folder,'workspace_curve',config.settings(),5,blocked)
    assert failed['status']=='no_feasible_route' and 'cycle' not in failed and len(calls)==2
    assert failed['details']['failed_checkpoint']==2
    assert not cp.ExperimentPlanDisplay(folder/'plan.json').playable
    calls.clear()
    retried=cp.plan_sequence(context,folder,'workspace_curve',config.settings(),5,leg)
    assert retried['status']=='planned' and cp.ExperimentPlanDisplay(folder/'plan.json').playable


def test_staged_timing_waits_for_all_geometry_and_keeps_endpoint_order(sequence,monkeypatch):
    from drum_planner.timing import pipeline
    context,leg,calls,folder=sequence;ready=[];events=[]
    def staged(*args):
        result=leg(*args);ready.append(copy.deepcopy(result));events.append('route')
        result.update(status='timing_pending',_timing_job={'fixture':len(calls)})
        return result
    def finish(legs):
        assert events==['route']*4 and len(legs)==4
        events.append('timing')
        for path,result in zip([folder/f'checkpoint_{i}' for i in (1,2,3)]+[folder/'return_to_start'],ready):
            atomic_json(path/'plan.json',result)
        return ready
    monkeypatch.setattr(pipeline,'finish_all',finish)
    result=cp.plan_sequence(context,folder,'workspace_curve',config.settings(),120,staged,parallel_timing=True)
    assert result['status']=='planned' and events==['route']*4+['timing']
    assert result['experiment']['timing_workers']==2
    assert cp.ExperimentPlanDisplay(folder/'plan.json').playable


def test_staged_failed_route_never_starts_timing_or_publishes_a_partial_plan(sequence,monkeypatch):
    from drum_planner.timing import pipeline
    context,leg,calls,folder=sequence
    def staged(*args):
        result=leg(*args)
        if len(calls)==2:result.update(status='no_feasible_route',details={'reason':'test blocked'})
        else:result.update(status='timing_pending',_timing_job={'fixture':True})
        return result
    monkeypatch.setattr(pipeline,'finish_all',lambda *_:pytest.fail('No timing for a blocked mission'))
    result=cp.plan_sequence(context,folder,'workspace_curve',config.settings(),120,staged,parallel_timing=True)
    assert result['status']=='no_feasible_route' and '_timing_job' not in result
    assert 'cycle' not in result and not cp.ExperimentPlanDisplay(folder/'plan.json').playable


def test_checkpoint_identity_is_order_sensitive_and_tampering_fails(sequence):
    context,leg,calls,folder=sequence
    assert cp.signature(context['checkpoints'])!=cp.signature(context['checkpoints'][::-1])
    Path(context['checkpoints'][1]['goal_recording']).write_text('{}')
    with pytest.raises(ValueError,match='snapshot changed'):
        cp.plan_sequence(context,folder,'workspace_curve',config.settings(),5,leg)
    assert not calls


def test_offset_roundoff_does_not_duplicate_display_timestamps(sequence):
    context,leg,calls,folder=sequence
    def close_samples(*args):
        result=leg(*args)
        if len(calls)>1:
            result['outward'].insert(1,dict(result['outward'][0],time_s=1e-16))
            atomic_json(Path(args[1])/'plan.json',result)
        return result
    cp.plan_sequence(context,folder,'workspace_curve',config.settings(),5,close_samples)
    plan=cp.ExperimentPlanDisplay(folder/'plan.json')
    assert plan.playable and np.all(np.diff(plan.times)>0)


def test_worker_budget_scales_with_legs_not_memory(tmp_path,monkeypatch):
    from types import SimpleNamespace
    calls=[]
    monkeypatch.setattr(app,'Job',lambda *a,**kw:calls.append(kw) or SimpleNamespace())
    atomic_json(tmp_path/'context.json',{'checkpoints':[{}, {}, {}]})
    app.start_plan(tmp_path,'workspace_curve',config.settings(),120)
    # Four serial legs, each with search cap plus bounded fast-timing/check
    # headroom. No old ten-minute CPU reference qualification per leg.
    assert calls==[{'timeout':600}]
    atomic_json(tmp_path/'context.json',{})
    app.start_plan(tmp_path,'workspace_curve',config.settings(),120)
    assert calls[-1]=={'timeout':150}


def test_leg_cache_identity_includes_original_scene_pose(sequence):
    from drum_planner.experiment.algorithms import pool_key
    context,_,_,folder=sequence
    req=json.loads((folder.parent/'session/initial/backend_request.json').read_text())
    first=pool_key(context,req,config.settings(),120)
    req['scene_robot_start']=req['start']
    second=pool_key(context,req,config.settings(),120)
    req['scene_robot_start']=list(req['start']);req['scene_robot_start'][0]+=.01
    third=pool_key(context,req,config.settings(),120)
    assert len({first,second,third})==3


def test_hardware_timing_generators_and_saved_data_unchanged():
    before=json.loads((ROOT/'diagnostics/checkpoints_20261021/before_hashes.json').read_text())
    changed={'drum_planner/experiment/app.py','drum_planner/experiment/worker.py',
             'drum_planner/experiment/algorithms.py','drum_planner/cpp/experiment_oracle.cpp'}
    # Authorized hardware Workspace curve/checkpoint rollout; new tests cover
    # its exact mission playback and unchanged experiment/timing/input sources.
    changed.update({'drum_planner/baseline/curve.py', 'drum_planner/baseline/transport.py', 'drum_planner/baseline/__init__.py', 'drum_planner/cli.py', 'drum_planner/baseline/computation.py', 'drum_planner/baseline/motion.py', 'drum_planner/baseline/windows.py', 'drum_planner/baseline/work.py', 'drum_planner/baseline/workspace.py', 'drum_planner/baseline/service.py', 'drum_planner/baseline/planner.py', 'drum_planner/baseline/tasks.py'})
    # Approved 5 mm map wiring; test_depth_resolution protects all generation/timing/data.
    changed.update({'drum_planner/depth.py','drum_planner/planning.py','drum_planner/cpp/moveit_plan.cpp','drum_planner/experiment/worker.py','drum_planner/experiment/algorithms.py','drum_planner/baseline/work.py','drum_planner/live_view.py'})
    for path,sha in before.items():
        if path.startswith('tests/') or path in changed|OPTIMIZED:continue
        assert digest(ROOT/path)==sha,path


def test_single_leg_worker_and_pool_identity_unchanged_without_checkpoint_fields():
    reference=ROOT/'diagnostics/checkpoints_20261021/reference'
    # Authorized timing phase extraction replaces the worker byte check.
    # Route generation/ranking and fixed policies remain byte-identical; full
    # numerical/output parity is checked by the performance regression study.
    from test_checkpoint_performance import test_policies_geometry_inputs_and_motor_control_are_unchanged
    test_policies_geometry_inputs_and_motor_control_are_unchanged()
    from drum_planner.experiment import algorithms
    text=Path(algorithms.__file__).read_text().replace(
        "    if 'scene_robot_start' in req:data['scene_robot_start']=req['scene_robot_start']\n",'')
    text=text.replace("from ..depth import scene_voxel_size\n",'').replace("              octree_resolution_m=scene_voxel_size(req.get('scene',{})),\n",'')
    assert text==(reference/'algorithms.py').read_text()


def test_blocked_return_is_not_replaced_by_reverse_and_retry_works(sequence):
    context,leg,calls,folder=sequence
    def blocked(*args):
        result=leg(*args)
        if len(calls)==4:result.update(status='no_feasible_route',details={'reason':'test return obstacle'})
        return result
    result=cp.plan_sequence(context,folder,'single_arc',config.settings(),5,blocked)
    assert result['status']=='no_feasible_route' and 'cycle' not in result
    assert result['details']['failed_leg']=='return_to_start'
    assert not cp.ExperimentPlanDisplay(folder/'plan.json').playable
    calls.clear()
    cp.plan_sequence(context,folder,'single_arc',config.settings(),5,leg)
    assert cp.ExperimentPlanDisplay(folder/'plan.json').playable


def test_reject_stale_or_discontinuous_return(sequence):
    context,leg,_,folder=sequence
    result=cp.plan_sequence(context,folder,'single_arc',config.settings(),5,leg)
    original=copy.deepcopy(result)
    result['checkpoint_sequence']['schema']='x5-experiment-checkpoints-v1'
    atomic_json(folder/'plan.json',result)
    with pytest.raises(ValueError,match='schema'):cp.ExperimentPlanDisplay(folder/'plan.json')
    original['checkpoint_sequence']['return_leg']['goal'][0]+=.1
    atomic_json(folder/'plan.json',original)
    with pytest.raises(ValueError,match='return'):cp.ExperimentPlanDisplay(folder/'plan.json')


def test_final_checkpoint_can_repeat_first_without_reversing_middle(sequence):
    context,leg,calls,folder=sequence
    context['checkpoints'][-1]=copy.deepcopy(context['checkpoints'][0])
    context['checkpoint_sha256']=cp.signature(context['checkpoints'])
    result=cp.plan_sequence(context,folder,'workspace_curve',config.settings(),5,leg)
    assert len(calls)==4 and calls[2][1]['goal']==calls[0][1]['goal']
    assert calls[3][1]['start']==calls[0][1]['goal']
    assert calls[3][1]['goal']==calls[0][1]['start']
    assert len(result['checkpoint_sequence']['segments'])==3
    assert cp.ExperimentPlanDisplay(folder/'plan.json').playable
