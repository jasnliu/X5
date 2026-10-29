"""Regress the actual negative-rest failure. No camera/CAN/motor interfaces."""
import copy,json,math,socket
from pathlib import Path
import numpy as np
import pytest
from scipy.interpolate import PPoly
from drum_planner import model,joint_position_limits as limits,rest_paths
from drum_planner.common import ROOT,atomic_json
from drum_planner.experiment import runtime
from drum_planner.fast_mode.kinematics import Kinematics
from cymbal_destination import goal_limits

REST=np.radians([1.9012127,-.3187004,10.0115883,.0769277,-14.3574935,-3.7035184,28.408294])

@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('No devices/sockets in these tests'))


def curve(coeffs,reference=REST):
    """J2 polynomial in degrees; other joints retain the actual measured rest."""
    c=np.zeros((len(coeffs),1,7));c[-1,0]=reference;c[:,0,1]=np.radians(coeffs)
    p=PPoly(c,[0.,1.]);req=dict(start=p(0).tolist(),goal=p(1).tolist(),
        joint_position_context=dict(policy=limits.policy(),rest_reference_rad=reference.tolist()))
    return p,req


def test_actual_failed_capture_model_accepts_exact_rest_without_clipping(tmp_path):
    folder=ROOT/'planner_runs/20261028T023247Z-hardware-right-ae273b/calculate-7f013e4d/capture'
    state=json.loads((folder/'measured_start.json').read_text());q=state['right_rad'];before=q.copy()
    m=model.generate(tmp_path,reference_right=q);k=Kinematics(tmp_path/'robot.urdf','right')
    assert k.lower[1]==q[1]-limits.REST_MODEL_ROUNDOFF_RAD
    assert np.degrees(q[1])==pytest.approx(-.3187004,abs=1e-6)
    assert np.all(q>=k.lower) and np.all(q<=k.upper)
    assert goal_limits.bounds(k.lower,k.upper)[0][1]==pytest.approx(np.radians(1))
    limits.require_current_model(m,tmp_path/'robot.urdf','right')
    req=dict(start=q,scene_robot_start=q,urdf=str(tmp_path/'robot.urdf'));limits.bind_request(req,m)
    limits.require_request(req,m);atomic_json(tmp_path/'request.json',req)
    runtime.Oracle.check_start_bounds(tmp_path/'request.json')
    assert q==before
    fb=limits.feedback_limits(m,np.c_[k.lower,k.upper],'right')
    assert fb[1][0]==pytest.approx(np.radians(-1))
    assert np.array_equal(np.delete(fb,1,axis=0),np.delete(np.c_[k.lower,k.upper],1,axis=0))
    bad=copy.deepcopy(req);bad['joint_position_context']['rest_reference_rad'][0]+=.01
    with pytest.raises(ValueError,match='reference changed'):limits.require_request(bad,m)


@pytest.mark.parametrize('angle',[-1.,-1.2,-7.,float('nan'),float('inf')])
def test_invalid_rest_is_not_whitelisted_or_clipped(tmp_path,angle):
    q=REST.copy();q[1]=np.radians(angle)
    with pytest.raises(ValueError):model.generate(tmp_path,reference_right=q.tolist())
    assert (np.isnan(q[1]) if np.isnan(angle) else q[1]==np.radians(angle))


def test_continuous_departure_return_and_stationary_rest():
    p,r=curve([1.,np.degrees(REST[1])]);proof=rest_paths.verify(p,r)
    assert proof['continuous_polynomial_checked'] and proof['negative_intervals']
    # Reverse the same polynomial into the EXACT measured rest.
    p,r=curve([-1.,1+np.degrees(REST[1])]);rest_paths.verify(p,r)
    p,r=curve([np.degrees(REST[1])]);rest_paths.verify(p,r)


@pytest.mark.parametrize('coeffs',[
    [1.,-1.,.1],             # positive ends but negative interior
    [.5,-.1,-.3187004],      # below acquired rest before leaving
    [1.,-1.1,.4,-.3187004], # turning toward stop inside margin
    [.1,-.3187004],          # negative goal which is NOT the reference rest
])
def test_continuous_negative_excursions_rejected(coeffs):
    p,r=curve(coeffs)
    with pytest.raises(ValueError):rest_paths.verify(p,r)


def test_negative_pose_at_same_j2_but_other_configuration_is_not_rest():
    p,r=curve([1.,-.3187004]);r['start'][0]+=.01;p.c[-1,0,0]+=.01;r['goal'][0]+=.01
    with pytest.raises(ValueError,match='exact original'):rest_paths.verify(p,r)


def test_left_never_inherits_right_rest_rules(tmp_path):
    a=model.generate(tmp_path/'a',moving_side='left')
    b=model.generate(tmp_path/'b',moving_side='left',reference_right=[float('nan')]*7)
    assert (tmp_path/'a/robot.urdf').read_bytes()==(tmp_path/'b/robot.urdf').read_bytes()
    assert a==b and 'joint_position_policy' not in a
    p,r=curve([0.,-10.]);r['moving_side']='left';assert rest_paths.verify(p,r) is None


def test_start_error_identifies_joint_degrees_and_does_not_launch_native(tmp_path,monkeypatch):
    model.generate(tmp_path/'model')
    req=dict(urdf=str(tmp_path/'model/robot.urdf'),start=REST.tolist())
    atomic_json(tmp_path/'request.json',req)
    monkeypatch.setattr(runtime.subprocess,'Popen',lambda *a,**k:pytest.fail('Preflight must reject before native spawn'))
    with pytest.raises(ValueError,match=r'openarmx_right_joint2 = -0.318700 degrees, outside model'):
        runtime.Oracle(tmp_path/'request.json',tmp_path)
    assert 'NOT clipped' in (tmp_path/'oracle.log').read_text()
    assert json.loads((tmp_path/'request.json').read_text())==req


def test_native_crash_reports_real_stderr_absolute_log_and_closes_process(tmp_path,monkeypatch):
    model.generate(tmp_path/'model',reference_right=REST.tolist())
    req=dict(urdf=str(tmp_path/'model/robot.urdf'),start=REST.tolist());atomic_json(tmp_path/'request.json',req)
    binary=tmp_path/'fake_oracle';binary.write_text('#!/bin/sh\necho "TEST native world failure detail" >&2\nexit 1\n');binary.chmod(0o700)
    monkeypatch.setattr(runtime,'build',lambda:binary)
    closed=[];close=runtime.Oracle.close
    def observe(self):
        close(self);closed.append(self)
    monkeypatch.setattr(runtime.Oracle,'close',observe)
    with pytest.raises(RuntimeError,match='TEST native world failure detail') as exc:runtime.Oracle(tmp_path/'request.json',tmp_path)
    assert str(tmp_path/'oracle.log') in str(exc.value)
    assert len(closed)==1 and closed[0].process.poll()==1 and closed[0].log.closed
    assert closed[0].process.stdin.closed and closed[0].process.stdout.closed


def test_negative_model_floor_never_reduces_dt1_goal_floor():
    lower=np.full(7,-1.4);upper=-lower;lower[1]=np.radians(-.3187004)
    lo,hi=goal_limits.bounds(lower,upper)
    assert lo[1]==pytest.approx(np.radians(1))
    q=(lo+hi)/2;q[1]=lo[1];proof=goal_limits.require(q,lower,upper)
    assert goal_limits.verify(q,proof,(lower,upper))==proof
    q[1]-=1e-5
    with pytest.raises(ValueError,match='reserve'):goal_limits.require(q,lower,upper)


def fake_plan_rig(path):
    from types import SimpleNamespace
    from drum_planner.baseline import service,motion
    from drum_planner.baseline.simulation import SimulatedBus
    holder=SimpleNamespace(info=dict(simulated=True,plan=str(path)),side='right')
    service.Server.prepare_motion(holder)
    obj,req,c=holder.motion_data;clock=SimpleNamespace(t=100.)
    bus=SimulatedBus(req,now=lambda:clock.t);commands=[];send=bus.set_positions
    def record(q):commands.append(np.array(q).copy());send(q)
    bus.set_positions=record
    e=motion.Controller(bus,obj,req,curve=c,now=lambda:clock.t)
    def tick(n=1):
        for _ in range(n):clock.t+=.01;e.step()
    def until(phase):
        for _ in range(20000):
            if e.phase==phase:return
            if e.phase in motion.TERMINAL or e.phase=='FAULT_HOLD':raise AssertionError((e.phase,e.error,e.feedback_error))
            tick()
        raise AssertionError(e.status())
    return SimpleNamespace(obj=obj,req=req,curve=c,bus=bus,engine=e,commands=commands,tick=tick,until=until)


@pytest.mark.parametrize('fixture_name',['latest_dt1','latest_r1','latest_r2','latest_r1_r2','latest_dt1_r2'])
@pytest.mark.parametrize('rate',[.5,1.])
def test_actual_negative_rest_plans_complete_fake_cycle_without_fault(fixture_name,rate):
    folder=ROOT/'diagnostics/hardware_rest_transition_20261028/verified'/fixture_name/'hardware'
    if not (folder/'plan.json').exists():pytest.skip('Run saved-scene native matrix first')
    r=fake_plan_rig(folder/'plan.json');r.engine.rate=rate;r.engine.start();r.until('RELAXED')
    assert not r.engine.error and r.bus.releases==1
    # Playback uses the checked polynomial, not snapped encoder targets. R2's
    # polynomial endpoint differs by 2.6e-18 rad; do not demand a silent snap.
    assert np.array_equal(r.bus.q,r.curve.at(0))
    np.testing.assert_allclose(r.bus.q,r.req['start'],rtol=0,atol=limits.REST_MODEL_ROUNDOFF_RAD)
    assert min(q[1] for q in r.commands)>=r.req['start'][1]-1e-10
    phases=[e['phase'] for e in r.engine.events]
    assert 'FAULT_HOLD' not in phases and 'VERIFIED_START_POSITION' in phases
    assert r.bus.control_side=='right' and all(r.bus.states['left',i][1]==0 for i in range(1,9))


@pytest.fixture
def mock_prepared(tmp_path,monkeypatch):
    """Freshness is explicitly MOCKED in temporary files, never hardware evidence."""
    from drum_planner.baseline import tasks
    from drum_planner.common import DEFAULT_CALIBRATION
    fixture=ROOT/'diagnostics/hardware_rest_transition_20261028/verified/latest_dt1/hardware'
    if not (fixture/'plan.json').exists():pytest.skip('Run saved-scene native matrix first')
    obj=json.loads((fixture/'plan.json').read_text())
    obj.update(experiment_only=False,offline_simulation=False,baseline_hardware_session=True,
        actual_current_robot_state_verified=True,acquisition_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        current_state_check={'TEST_ONLY_FAKE_FRESHNESS':True})
    atomic_json(tmp_path/'plan.json',obj)
    (tmp_path/'backend_request.json').write_bytes((fixture/'backend_request.json').read_bytes())
    capture=Path(obj['provenance']['state']).parent
    tasks.prepare(tmp_path/'checked',tmp_path/'plan.json',capture,DEFAULT_CALIBRATION,allow_replan=False)
    packet,curve=tasks.check_packet(tmp_path/'checked/prepared.json')
    assert packet['proof']['all_valid'] and packet['motion_authorized'] is False
    return tmp_path/'checked/prepared.json',tmp_path/'plan.json'


@pytest.mark.parametrize('progress_ticks',[0,3,60,200])
def test_actual_native_hold_reverse_with_measured_j2_difference(mock_prepared,tmp_path,progress_ticks):
    from drum_planner.baseline import tasks
    prepared,path=mock_prepared;r=fake_plan_rig(path)
    r.engine.start();r.until('OUTWARD');r.tick(progress_ticks)
    # Simulated healthy feedback 0.02 degrees below the exact target, including
    # below the original negative rest. This is not an invented new rest/goal.
    r.bus.q[1]-=np.radians(.02);r.engine.halt();r.tick(70)
    assert r.engine.phase=='HELD' and r.engine.hold_confirmed and not r.bus.releases
    ask=r.engine.request_return(True);r.tick(70)
    old_model=Path(r.req['urdf']).read_bytes()
    tasks.check_return(tmp_path/'return',prepared,ask['held'],ask['tau'],ask['rate'],ask['left_snapshot'])
    spec=json.loads((tmp_path/'return/return.json').read_text())
    assert spec['proof']['all_valid'] and spec['tau']<=ask['tau']
    assert Path(r.req['urdf']).read_bytes()==old_model
    if progress_ticks==0:assert spec['measured_j2_connector']['held_lower_rad']==ask['held'][1]
    assert r.engine.accept_return(spec,ask['generation']);r.until('RELAXED')
    assert r.bus.releases==1 and np.array_equal(r.bus.q,r.req['start'])


def test_prepared_rest_context_tampering_cannot_start(mock_prepared,tmp_path):
    from drum_planner.baseline import tasks
    prepared,_=mock_prepared;packet=json.loads(prepared.read_text())
    packet['request']['joint_position_context']['rest_reference_rad'][1]-=.001
    atomic_json(tmp_path/'tampered.json',packet)
    with pytest.raises(ValueError,match='reference changed'):tasks.check_packet(tmp_path/'tampered.json')
