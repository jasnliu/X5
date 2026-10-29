"""Device-free regression of the actual isolated generated position writer."""
import csv
import io
import json
import multiprocessing as mp
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from camera_playback.generated_stream import (run_controller, reference, StreamSession,
    StreamMotors, Progress, feedback)
from camera_playback.generated_simulation import SimulatedGeneratedMotors, SimulatedStreamMotors
from camera_playback.generated_motion import Motion
from centering.motors import parameter, packet
from safe_zone.encoder import FRAME, joint_to_motor
from tests.test_generated_beat import Clock
from camera_playback.generated_controller import GeneratedController
from drum_planner.baseline.motion import Controller
from drum_planner.baseline.curve import Curve


def make_curve(side='left', delta=None):
    delta=np.linspace(.01,.05,7) if delta is None else np.asarray(delta)
    coefficients=[np.array([6*d,-15*d,10*d,0,0,0])[:,None].tolist() for d in delta]
    return Curve(dict(moving_side=side, outward=[], timing=dict(duration_s=1.,time_scale=1.,
        coefficients=coefficients,polynomial_breaks_unscaled_s=[[0.,1.]]*7)))


def rig_controller(clock, curve=None, step=None):
    c=curve or make_curve();side=c.plan['moving_side']
    values={s:np.zeros(7) for s in ('left','right')}
    values[side]=reference(c,step,0.)[0].copy()
    bus=SimpleNamespace(poll=lambda:None, command=lambda q:values[side].__setitem__(slice(None),q))
    engine=GeneratedController(bus,c,side,step,values,lambda *_:values,now=clock)
    assert type(engine).step is Controller.step
    assert type(engine).send is Controller.send
    return c,engine


def test_fixed_deadlines_exact_curve_and_no_processing_time_drift():
    clock=Clock();c,engine=rig_controller(clock);rows=[]
    def check():clock.advance(.0002)
    def progress(t,tau,q,late):rows.append((clock(),t,tau,q.copy()))
    run_controller(engine,check,progress,clock=clock,wait=clock.advance)
    assert len(rows)>=195
    assert abs(np.median(np.diff([r[0] for r in rows]))-.005)<.0003
    assert rows[-1][1]==c.duration
    for _,t,tau,q in rows:
        assert tau==t;np.testing.assert_array_equal(q,c.at(t))


def test_delayed_tick_skips_missed_targets_without_burst_or_retiming():
    clock=Clock();c,engine=rig_controller(clock);rows=[]
    delayed=False
    def check():
        nonlocal delayed
        if not delayed and .099<clock()<.101:clock.advance(.025);delayed=True
    run_controller(engine,check,lambda t,tau,q,late:rows.append((clock(),t,q.copy())),clock=clock,wait=clock.advance)
    times=np.array([r[0] for r in rows]);assert np.max(np.diff(times))>.02
    assert np.min(np.diff(times))>=.0049
    for _,t,q in rows:np.testing.assert_array_equal(q,c.at(t))


def test_shared_controller_missed_80ms_deadline_stops_before_catchup_command():
    clock=Clock();c,engine=rig_controller(clock);sent=[]
    def progress(*args):sent.append(args);clock.advance(.081)
    with pytest.raises(RuntimeError,match='80 ms'):
        run_controller(engine,lambda:None,progress,clock=clock,wait=clock.advance)
    assert len(sent)==1 and engine.phase=='FAULT_HOLD' and engine.held is not None


def test_common_discontinuity_guard_is_not_bypassed():
    clock=Clock();c,engine=rig_controller(clock)
    engine.begin_segment()
    with pytest.raises(RuntimeError,match='discontinuity'):engine.send(np.ones(7))


@pytest.mark.parametrize('side',['left','right'])
def test_shared_controller_exact_cached_prefix(side):
    from drum_planner.baseline.paths import recovery
    clock=Clock();c=make_curve(side)
    spec=recovery(c,c.at(c.duration),c.duration,.5)
    step=dict(kind='prefix',side=side,spec=spec,duration=spec['bridge_s']+spec['return_s'])
    c,engine=rig_controller(clock,c,step);rows=[]
    run_controller(engine,lambda:None,lambda t,tau,q,late:rows.append((t,tau,q.copy())),clock=clock,wait=clock.advance)
    assert engine.phase=='SETTLING_REST'  # NO auto-release or second outbound leg.
    for t,tau,q in rows:
        expected,original_tau=reference(c,step,t)
        np.testing.assert_array_equal(q,expected);assert tau==original_tau
    np.testing.assert_array_equal(rows[-1][2],c.at(0))


@pytest.mark.parametrize('side',['left','right'])
def test_legacy_checked_connector_retains_its_exact_law(side):
    clock=Clock();c=make_curve(side)
    step=dict(kind='strike_return',side=side,start=c.at(1).tolist(),
              goal=(c.at(1)+.01).tolist(),duration=.8)
    c,engine=rig_controller(clock,c,step);rows=[]
    run_controller(engine,lambda:None,lambda t,tau,q,late:rows.append((t,tau,q.copy())),clock=clock,wait=clock.advance)
    assert engine.phase=='SETTLING_GOAL'
    for t,tau,q in rows:
        expected,_=reference(c,step,t)
        np.testing.assert_array_equal(q,expected);assert tau is None


def test_shared_timing_validation_and_arm_mismatch_are_not_bypassed():
    clock=Clock();c=make_curve();c.limits[:]=1e-6
    with pytest.raises(ValueError,match='velocity/acceleration/jerk'):rig_controller(clock,c)
    c=make_curve('right')
    with pytest.raises(ValueError,match='Arm mismatch'):
        GeneratedController(SimpleNamespace(),c,'left',None,
                            {s:np.zeros(7) for s in ('left','right')},lambda *_:None,now=clock)


@pytest.mark.parametrize('side',['left','right'])
def test_generated_wire_gate_only_selected_positions_and_mode_queries(side):
    b=StreamMotors.__new__(StreamMotors);b.control_side=side;b.special=None
    b.sockets={s:Mock() for s in ('left','right')};b.sockets[side].send.return_value=16
    b.command_file=io.StringIO()
    q=np.linspace(-.2,.2,7);b.command(q)
    assert [c.args[0] for c in b.sockets[side].send.call_args_list]==[
        parameter(i,0x7016,joint_to_motor(side,i,float(v))) for i,v in enumerate(q,1)]
    for frame in [packet(3,1),packet(4,1),parameter(1,0x7005,5,True),parameter(1,0x7017,.8),
                  parameter(1,0x7018,2.),parameter(1,0x701e,10.),parameter(8,0x7016,.1)]:
        with pytest.raises(RuntimeError,match='ONLY'):b._send(side,frame)
    with pytest.raises(RuntimeError):b._send('left' if side=='right' else 'right',parameter(1,0x7016,.1))
    with pytest.raises(RuntimeError,match='Nonfinite'):b._send(side,parameter(1,0x7016,float('nan')))


@pytest.mark.parametrize('side,motor',[('left',6),('right',7)])
def test_return_feeds_existing_mit_owner_in_display_coordinates_without_csp(side,motor):
    b=StreamMotors.__new__(StreamMotors);b.control_side=side
    hold=threading.Event();hold.set();goal=SimpleNamespace(value=0.)
    b.special=dict(motor=motor,goal=goal,hold=hold)
    b.sockets={side:Mock()};b.sockets[side].send.return_value=16;b.command_file=io.StringIO()
    q=np.arange(7)*.03;b.command(q)
    assert goal.value==q[motor-1]
    assert all(FRAME.unpack(c.args[0])[0]&255!=motor for c in b.sockets[side].send.call_args_list)
    with pytest.raises(RuntimeError,match='MIT-owned'):b._send(side,parameter(motor,0x7016,.1))
    hold.clear()
    with pytest.raises(RuntimeError,match='Hold'):b.command(q)


def make_session(tmp_path,side='left'):
    rest={s:np.zeros(7) for s in ('left','right')}
    b=SimulatedGeneratedMotors(rest,streaming=True);b.active=True;b.poll()
    b.stream_geometry={};b.stream_directory=tmp_path
    c=make_curve(side)
    return b,c,b.start_stream(c,side,None)


def wait_ready(s):
    end=time.monotonic()+6
    while not s.owns_feedback:
        update=s.poll()
        assert not update['done'],update
        if time.monotonic()>end:raise AssertionError('spawn readiness timeout')
        time.sleep(.005)


@pytest.mark.parametrize('side',['left','right'])
def test_real_process_200hz_exact_targets_during_gui_pause(tmp_path,side):
    b,c,s=make_session(tmp_path,side)
    try:
        wait_ready(s)
        # Emulate a busy Tk/ROS callback. The writer has an independent GIL/clock.
        until=time.monotonic()+.18
        while time.monotonic()<until:sum(range(100))
        assert s.poll()['elapsed']>=.15
        while True:
            update=s.poll()
            if update['done']:break
            time.sleep(.02)
        assert update['result']['success'],update
        assert update['result']['motor_settings_changed'] is False
        np.testing.assert_array_equal(b.q[side],c.at(c.duration))
        np.testing.assert_array_equal(b.q['right' if side=='left' else 'left'],np.zeros(7))
        rows=list(csv.DictReader((s.directory/'trace.csv').open()))
        assert len(rows)>=185
        dt=np.diff([float(r['monotonic_s']) for r in rows])
        assert .004<np.median(dt)<.006
        for row in rows:
            t=float(row['elapsed_s']);q=np.array([float(row[f'q{i}']) for i in range(1,8)])
            np.testing.assert_array_equal(q,c.at(t))
        assert float(rows[-1]['elapsed_s'])==1.
    finally:s.stop();s.detach();b.active=False;b.close()


def test_cancellation_joins_writer_and_keeps_latest_prefix_progress(tmp_path):
    b,c,s=make_session(tmp_path)
    try:
        wait_ready(s);time.sleep(.075)
        before=s.poll()['tau'];stopped=s.stop()
        assert stopped['done'] and stopped['tau']>=before and stopped['tau']<c.duration
        assert stopped['result']['cancelled'] and stopped['result']['held'] is not None
        held=b.q['left'].copy();time.sleep(.08)
        np.testing.assert_array_equal(b.q['left'],held)
        with pytest.raises(RuntimeError,match='writer'):b.command_arm('left',np.zeros(7))
        s.detach();b.command_arm('left',held,hold=True)
        assert b.active
    finally:s.stop();s.detach();b.active=False;b.close()


def test_gui_heartbeat_loss_holds_without_relax_or_further_progress(tmp_path):
    b,c,s=make_session(tmp_path)
    try:
        wait_ready(s);time.sleep(.65)
        s.process.join(2)
        update=s.poll();assert update['done']
        assert 'heartbeat' in update['result']['error']
        assert update['tau']<c.duration and update['result']['held'] is not None
        assert b.active
    finally:s.stop();s.detach();b.active=False;b.close()


def test_relax_joins_delegate_even_when_feedback_is_stale(tmp_path):
    b,c,s=make_session(tmp_path)
    try:
        wait_ready(s);b.fresh=Mock(side_effect=AssertionError('No relax feedback gate'))
        b.emergency_release()
        assert not b.active and not s.process.is_alive() and b.path_session is None
    finally:s.stop();s.detach();b.active=False;b.close()


def test_spawned_worker_fault_is_reported_not_silently_replaced(tmp_path):
    b,c,s=make_session(tmp_path)
    try:
        wait_ready(s);b.shared[7]=float('nan')  # invalid opposite-arm feedback
        end=time.monotonic()+3
        while not (update:=s.poll())['done']:
            assert time.monotonic()<end;time.sleep(.01)
        assert not update['result']['success'] and 'Nonfinite' in update['result']['error']
        assert b.active
    finally:s.stop();s.detach();b.active=False;b.close()


def test_parent_transport_rejects_competing_writer_at_both_boundaries():
    from camera_playback.generated_transport import GeneratedMotors
    from tests.test_left_center_hold import fake_bus
    b=GeneratedMotors.from_owner(fake_bus(),{s:np.zeros(7) for s in ('left','right')})
    for side in ('left','right'):
        b.path_session=SimpleNamespace(side=side)
        with pytest.raises(RuntimeError,match='worker owns'):
            if side=='left':b.left_drive._send(side,parameter(1,0x7016,0.))
            else:b._send(side,parameter(1,0x7016,0.))


def test_writer_handoff_failure_cannot_veto_emergency_disable():
    from tests.test_generated_stop_return import fake_owner
    b=fake_owner();session=Mock();session.process.is_alive.return_value=False
    session.detach.side_effect=RuntimeError('injected handoff error');b.path_session=session
    with patch('camera_playback.emergency.stop_writer'):
        with pytest.raises(RuntimeError,match='Disable unconfirmed'):b.emergency_release()
    for side in ('left','right'):
        assert b.sockets[side].sock.sock.send.call_count==24


def test_hold_uses_writer_measured_pose_not_older_gui_sample():
    from tests.test_generated_beat import rig
    clock,b,m=rig();m.start();clock.advance();b.poll();m.step()
    actual=np.ones(7)*.03
    update=dict(target=np.ones(7)*.025,tau=.3,result=dict(held=actual.tolist()))
    delegate=Mock();delegate.stop.return_value=update
    m.stream=delegate;m.stream_side='left'
    m.hold()
    np.testing.assert_array_equal(m.targets['left'],actual)
    assert m.progress['left']==.3 and m.stream is None
    delegate.detach.assert_called_once()


def test_saved_curves_and_prepared_return_evaluated_without_refitting():
    from camera_playback.generated_planning import load
    from drum_planner.baseline.paths import recovery_at
    path=Path('diagnostics/generated_beat_controls_20261028/verified/mission.json')
    if not path.exists():pytest.skip('saved software fixture unavailable')
    obj,curves=load(path)
    for side,c in curves.items():
        before=json.dumps(c.plan,sort_keys=True)
        for t in np.linspace(0,c.duration,251):
            q,tau=reference(c,None,t);np.testing.assert_array_equal(q,c.at(t));assert tau==t
        step=next(s for s in obj['return_plan']['steps'] if s['side']==side)
        for t in np.linspace(0,step['duration'],251):
            q,tau=reference(c,step,t);expected,old_tau,_=recovery_at(c,step['spec'],t)
            np.testing.assert_array_equal(q,expected);assert tau==old_tau
        assert json.dumps(c.plan,sort_keys=True)==before
