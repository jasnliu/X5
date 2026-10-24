"""Current Single arc backend/exact clocks; devices forbidden throughout."""
import copy,json,socket,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,JOINTS,LEFT_JOINTS,atomic_json
from drum_planner import planning
from drum_planner.timing import core
from drum_planner.baseline import planner,paths,motion
from drum_planner.baseline.curve import Curve
from drum_planner.baseline.simulation import SimulatedBus


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    real=socket.socket
    def guard(family=socket.AF_INET,*a,**k):
        if family==socket.AF_CAN:pytest.fail('No physical CAN permitted')
        return real(family,*a,**k)
    monkeypatch.setattr(socket,'socket',guard)
    from drum_planner import depth
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**k:pytest.fail('No camera permitted'))
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')


def fixture(case='right_single_arc'):
    base=ROOT/'diagnostics/corner_smoothing_20261021/selected'/case
    source=json.loads((base/'source.json').read_text());law=json.loads((base/'law.json').read_text())
    return dict(timing=law,outward=core.samples(law,source)),source


@pytest.mark.parametrize('case',['right_single_arc','right_clearance','right_progress','left_single_arc'])
def test_exact_controller_clock_matches_preview_law_and_limits(case):
    obj,_=fixture(case);c=Curve(obj);paths.validate_timing(c)
    grid=np.linspace(0,c.duration,4097)
    assert np.array_equal(c.at_many(grid),core.evaluate(obj['timing'],grid)[0])
    assert np.all(paths.peaks(c)<=core.LIMITS[:,None]+1e-8)
    for t in grid[::200]:assert np.array_equal(c.at(t),core.evaluate(obj['timing'],t)[0])
    with pytest.raises(ValueError):c.at(-.1)
    with pytest.raises(ValueError):c.at(c.duration+.1)


def test_corrupt_law_samples_and_limit_metadata_cannot_authorize_playback():
    obj,_=fixture();bad=copy.deepcopy(obj);bad['outward'][30]['positions_rad'][0]+=.01
    with pytest.raises(ValueError,match='differs'):Curve(bad)
    bad=copy.deepcopy(obj);bad['timing']['duration_s']+=.1
    with pytest.raises(ValueError,match='duration mismatch'):Curve(bad)
    bad=copy.deepcopy(obj);bad['timing']['density']['c']=(np.asarray(bad['timing']['density']['c'])*.5).tolist()
    bad['timing']['duration_s']*=.5;bad['timing']['peaks']=[[0]*7]*3
    ev=core.Evaluator(bad['timing']);grid=np.linspace(0,ev.duration,21)
    bad['outward']=[dict(time_s=float(t),positions_rad=q.tolist()) for t,q in zip(grid,ev(grid)[0])]
    with pytest.raises(ValueError,match='limits'):paths.validate_timing(Curve(bad))


@pytest.mark.parametrize('rate',[.25,.6,1.])
@pytest.mark.parametrize('interruption',[None,'hold','fault'])
def test_retimed_full_cycle_or_held_prefix_only_relaxes_at_start(rate,interruption):
    obj,source=fixture();req=copy.deepcopy(source['request']);req['execution_joint_limits']=[[-4,4]]*7
    clock=SimpleNamespace(t=100.);bus=SimulatedBus(req,now=lambda:clock.t);c=motion.Controller(bus,obj,req,rate=rate,now=lambda:clock.t)
    def tick():clock.t+=.01;c.step()
    def until(phase):
        for _ in range(15000):
            if c.phase==phase:return
            assert c.phase not in motion.TERMINAL,(c.phase,c.error)
            tick()
        pytest.fail('Controller did not reach '+phase)
    c.start();until('OUTWARD')
    if interruption:
        for _ in range(40):tick()
        c.halt('software test',fault=interruption=='fault')
        for _ in range(80):tick()
        assert c.hold_confirmed and bus.releases==0
        assert c.status()['can_return']
        spec=paths.recovery(c.curve,c.held,c.tau,rate)
        spec['proof']={'all_valid':True};spec['left_snapshot']=dict(left=req['left'],left_opening_m=req['left_opening_m'])
        request=c.request_return(True)
        for _ in range(70):tick()
        c.accept_return(spec,request['generation'])
    until('RELAXED')
    assert bus.releases==1 and np.allclose(bus.q,req['start'],atol=1e-10)
    if interruption:assert 'GOAL_HOLD' not in [e['phase'] for e in c.events]
    else:
        phases=[e['phase'] for e in c.events];assert 'GOAL_HOLD' in phases and 'RETURN' in phases


def test_real_startup_display_model_metadata_without_any_devices(tmp_path,monkeypatch):
    from drum_planner.baseline import workspace
    monkeypatch.setattr(workspace,'DisplayTelemetry',lambda *a,**k:pytest.fail('No feedback reader'))
    v=workspace.Workspace(tmp_path,autoscene=False)
    try:
        assert len([w for w in v.windows if w.isVisible()])==3
        model=v.preview.robot.plan
        assert model.stick_link=='right_drumstick' and model.side=='right' and model.joints==JOINTS
        assert v.plan is None and v.job is None and v.owner_info is None
        pose=dict(joints=dict.fromkeys(JOINTS+LEFT_JOINTS,0.)|{'openarmx_left_finger_joint1':0.},stamp=time.monotonic())
        v.set_pose(pose)
        assert v.live.robot.last_tip.shape==(3,) and all(b.isEnabled() for b in v.control.buttons.values())
    finally:v.close()


@pytest.mark.parametrize('rate',[.25,.6,1.])
def test_baseline_preview_speed_matches_exact_execution_and_two_second_hold(tmp_path,rate):
    from drum_planner.trajectory_view import PlanDisplay
    from drum_planner.baseline.windows import PlanWindow
    from drum_planner.timing.benchmark import benchmark_request
    from PyQt5.QtWidgets import QApplication
    app=QApplication.instance() or QApplication([])
    obj,source=fixture();req=benchmark_request(source);T=obj['timing']['duration_s']
    obj.update(schema='x5-moveit-result-v1',status='planned',motion_authorized=False,hardware_execution_available=False,
        moving_side='right',model=source['model'],cycle=planning.cycle(obj['outward']),duration_outward_s=T,hold_at_goal_s=2.)
    atomic_json(tmp_path/'plan.json',obj);atomic_json(tmp_path/'backend_request.json',req)
    p=PlanDisplay(tmp_path/'plan.json');window=PlanWindow(p,1.)
    try:
        window.install_plan(p);window.rate.setValue(rate);display=window.plan;c=Curve(obj)
        assert display.duration==pytest.approx(2*T/rate+2)
        for t in np.linspace(.01,T/rate-.01,23):
            assert np.allclose(display.positions(t),c.at(t*rate),atol=1e-12)
            assert np.allclose(display.positions(display.duration-t),display.positions(t),atol=1e-12)
        for t in [T/rate,T/rate+1,T/rate+2]:assert np.array_equal(display.positions(t),req['goal'])
        assert p.obj==obj and p.preview_rate==1. and p.duration==pytest.approx(2*T+2)
    finally:window.shutdown()


def test_hardware_backend_uses_workspace_curve_and_latest_right_dispatch(monkeypatch,tmp_path):
    from drum_planner.experiment.config import saved_inputs
    state,scene=saved_inputs();obj,source=fixture();calls=[]
    winner=dict(path=source['path_waypoints'],timing=source['timing'])
    report=dict(pool_key='unit fixture',joint_limit_check={'valid':True})
    def generate(req,context,pool,path,params,seconds,log):
        calls.append('workspace_curve');assert req['start']==json.loads(state.read_text())['right_rad']
        assert req['goal']==planning.endpoint().tolist();assert seconds==120.
        return winner,report
    def timing(w,r,req,context,folder,log):
        calls.append('timing');assert req.get('moving_side','right')=='right' and w is winner and r is report
        return obj['outward'],obj['timing'],dict(samples_checked=2000),dict(method='adaptive')
    from drum_planner.timing import dispatch
    monkeypatch.setattr(planner.workspace_curve,'run',generate);monkeypatch.setattr(dispatch,'retime',timing)
    result=planner.plan(tmp_path/'result',state,scene,DEFAULT_CALIBRATION)
    assert calls==['workspace_curve','timing'] and result['route_planner']['method']=='workspace_curve'
    assert not result['experiment_only'] and not result['actual_current_robot_state_verified']
    assert not result['motion_authorized'] and result['motor_commands']==0
    assert result['hold_at_goal_s']==2 and result['reverse_checked_on_same_static_scene']
    assert result['provenance']['state']==str(state.resolve())
    with pytest.raises(FileExistsError):planner.plan(tmp_path/'result',state,scene,DEFAULT_CALIBRATION)


def test_prepare_packet_and_recovery_accept_exact_new_law_without_devices(tmp_path):
    """Exercise production packet plumbing on a marked, temporary offline fixture.
    Revoke its test-only acquisition flags on disk even if an assertion fails.
    """
    from drum_planner.baseline import tasks
    from drum_planner.experiment.config import saved_inputs
    from drum_planner.common import digest,DEFAULT_RECORDING
    from drum_planner.right_zone import metadata
    from drum_planner.timing.benchmark import benchmark_request
    obj,source=fixture();req=benchmark_request(source);state,scene=saved_inputs()
    obj.update(schema='x5-moveit-result-v1',status='planned',synthetic=False,moving_side='right',
        motion_authorized=False,hardware_execution_available=False,baseline_hardware_session=True,
        actual_current_robot_state_verified=True,offline_test_fixture=True,experiment_only=False,
        collision_policy=planning.COLLISION_POLICY,strict_unknown_space=False,hold_at_goal_s=2.,
        reverse_checked_on_same_static_scene=True,acquisition_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        calibration_sha256=digest(DEFAULT_CALIBRATION),recording_sha256=digest(DEFAULT_RECORDING),model=source['model'],
        right_zone_constraint=metadata(req['right_zone']),
        provenance=dict(state=str(state),scene=str(scene),state_sha256=digest(state)),
        duration_outward_s=obj['timing']['duration_s'],cycle=planning.cycle(obj['outward']))
    path=tmp_path/'plan.json';atomic_json(path,obj);atomic_json(tmp_path/'backend_request.json',req)
    try:
        tasks.prepare(tmp_path/'prepared',path,state.parent,DEFAULT_CALIBRATION,allow_replan=False)
        packet,c=tasks.check_packet(tmp_path/'prepared/prepared.json')
        assert packet['proof']['all_valid'] and not packet['motion_authorized']
        assert c.retimed is not None and np.allclose(c.at(c.duration),req['goal'])
        tau=c.duration*.5
        tasks.check_return(tmp_path/'return',tmp_path/'prepared/prepared.json',c.at(tau),tau,1.)
        returned=json.loads((tmp_path/'return/return.json').read_text())
        assert returned['proof']['all_valid'] and returned['tau']<=tau
        obj['experiment_only']=True;atomic_json(path,obj)
        with pytest.raises(ValueError,match='newly acquired'):tasks.load_checked(path,DEFAULT_CALIBRATION)
    finally:
        obj['actual_current_robot_state_verified']=False;obj['baseline_hardware_session']=False
        atomic_json(path,obj)  # invalidates prepared plan hash as well
