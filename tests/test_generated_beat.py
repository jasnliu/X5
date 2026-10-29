"""Generated normal beat contracts. All transports and clocks are in memory."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch,Mock
import numpy as np
import pytest
from camera_playback.generated_motion import Motion
from camera_playback.generated_simulation import SimulatedGeneratedMotors
from camera_playback.generated_transport import GeneratedMotors,GeneratedLeftDrive
from camera_playback.generated_worker_hold import PoweredPathHold
from camera_playback.mit_strike import Sample
from camera_playback import generated_planning
from drum_planner.baseline.paths import recovery
from drum_planner.timing import core
from drum_planner.baseline.curve import Curve


class Clock:
    def __init__(self):self.t=0.
    def __call__(self):return self.t
    def advance(self,dt=.02):self.t+=dt


class Line:
    def __init__(self,start,delta):
        self.start=np.array(start);self.delta=np.array(delta);self.duration=1.
        self.limits=np.tile([.6],(3,7))
    def at(self,t):return self.start+self.delta*(10*t**3-15*t**4+6*t**5)


def rig():
    clock=Clock();rest={s:np.arange(7)*.01 for s in ('left','right')}
    curves={s:Line(rest[s],np.full(7,.05 if s=='left' else -.04)) for s in rest}
    bus=SimulatedGeneratedMotors(rest,clock);motion=Motion(bus,curves,clock=clock)
    return clock,bus,motion


def ticks(clock,bus,motion,until,count=400):
    for _ in range(count):
        clock.advance();bus.poll();motion.step()
        if until():return
    raise AssertionError(motion.stage)


def test_start_left_then_right_without_center_or_reverse():
    clock,bus,m=rig();assert not bus.calls
    m.start();ticks(clock,bus,m,lambda:m.stage=='ARRIVED')
    assert [s for _,s in m.events]==['ARMING','SETTLE START','LEFT','SETTLE LEFT','RIGHT','SETTLE RIGHT','ARRIVED']
    commands=[c for c in bus.calls if c[0]=='path'];sides=[c[1] for c in commands]
    assert sides==sorted(sides) and bus.active
    assert m.progress==dict(left=1.,right=1.)
    assert not any(c[0]=='release_at_rest' for c in bus.calls)


def test_start_rejects_moved_rest_before_any_enable():
    clock,bus,m=rig();bus.q['left'][2]+=.02;bus.poll()
    with pytest.raises(RuntimeError,match='moved since capture'):m.start()
    assert not bus.calls


def test_hold_freezes_both_arms_no_auto_return_or_release():
    clock,bus,m=rig();m.start();ticks(clock,bus,m,lambda:m.stage=='LEFT')
    ticks(clock,bus,m,lambda:m.progress['left']>.25)
    held={s:q.copy() for s,q in bus.q.items()};progress=m.progress.copy();m.hold()
    for _ in range(80):clock.advance();bus.poll();m.step()
    assert bus.active and m.stage=='HOLD' and m.progress==progress
    assert all(np.array_equal(bus.q[s],held[s]) for s in held)
    assert not any(c[0]=='release_at_rest' for c in bus.calls)


def test_hold_during_arm_setup_does_not_resume():
    clock,bus,m=rig();m.start();clock.advance();bus.poll();m.step();m.hold()
    for _ in range(50):clock.advance();bus.poll();m.step()
    assert m.setup is None and m.stage=='HOLD' and m.progress=={'left':0.,'right':0.}


def test_scheduler_no_catchup():
    clock,bus,m=rig();m.start();ticks(clock,bus,m,lambda:m.stage=='LEFT')
    clock.advance(.2);bus.poll()
    with pytest.raises(RuntimeError,match='80 ms'):m.step()
    m.hold('deadline');assert m.stage=='HOLD' and bus.active


def test_return_requires_stationary_hold_and_cannot_accept_after_cancel():
    clock,bus,m=rig();m.start();ticks(clock,bus,m,lambda:m.stage=='LEFT')
    ticks(clock,bus,m,lambda:m.progress['left']>.3);m.hold()
    with pytest.raises(RuntimeError,match='stationary'):m.return_input()
    for _ in range(40):clock.advance();bus.poll();m.step()
    spec=m.return_input();m.hold('Cancel return')
    with pytest.raises(RuntimeError,match='cancelled'):m.accept_return(dict(status='ready',steps=[],**spec))


def test_exact_reference_unchanged_by_following_supervisor():
    clock,bus,m=rig();m.start();ticks(clock,bus,m,lambda:m.stage=='LEFT')
    clock.advance(.02);bus.poll();m.step()
    np.testing.assert_array_equal(bus.target['left'],m.curves['left'].at(m.progress['left']))


def test_worker_hold_does_not_complete_strike_or_move_to_center():
    sample=Sample(.7,.0,0.,2,0.)
    held=PoweredPathHold(sample,0.)
    command=held.update(Sample(.7,0.,0.,2,.01),.01,.7,-1.,1.)
    assert command.position==.7 and command.velocity==0.
    with pytest.raises(RuntimeError,match='fresh'):held.update(sample,.1,.7,-1.,1.)
    with pytest.raises(RuntimeError,match='bounds'):held.update(Sample(.7,0.,0.,2,.2),.2,2.,-1.,1.)


def test_worker_path_speed_and_acceleration_remain_bounded():
    hold=PoweredPathHold(Sample(.7,0.,0.,2,0.),0.);q=.7;previous=0.
    for i in range(1,100):
        t=i*.01;cmd=hold.update(Sample(q,previous,0.,2,t),t,0.,-1.,1.)
        assert abs(cmd.velocity)<=.35+1e-12
        assert abs(cmd.velocity-previous)<=.006+1e-12
        q=cmd.position;previous=cmd.velocity


def test_worker_hold_relatched_on_second_emergency():
    import threading
    from camera_playback.hybrid_strike import HybridSession
    s=HybridSession.__new__(HybridSession);s.path_hold=threading.Event();s.path_target=SimpleNamespace(value=0.)
    s.emergency_hold(.5);s.path_target.value=.4;s.emergency_hold(.45)
    assert s.path_target.value==.45 and s.path_hold.is_set()
    with pytest.raises(RuntimeError,match='latched'):s.request('search',.1)


def test_transport_left_cap_is_exact_scoped_packet():
    from centering.motors import parameter,RIGHT_PLAYBACK_SPEED
    view=GeneratedLeftDrive.__new__(GeneratedLeftDrive);view.active=True
    view.sockets={'left':SimpleNamespace(send=Mock(return_value=16))}
    view._send('left',parameter(2,0x7017,RIGHT_PLAYBACK_SPEED))
    assert view.sockets['left'].send.call_count==1
    with patch('camera_playback.left_hold.LeftHoldDrive._send',return_value=None) as original:
        view._send('left',parameter(2,0x7017,.9));original.assert_called_once()


def test_acquisition_uses_both_stationary_arms_and_closes_before_planning(tmp_path):
    from camera_playback import generated_acquisition as a
    from drum_planner.common import JOINTS,LEFT_JOINTS
    worker=Mock(error=None);worker.report.return_value={'motor_commands':0}
    with patch.object(a,'Telemetry',return_value=worker),patch.object(a,'paired_capture') as capture:
        a.capture(tmp_path/'capture')
    assert capture.call_args.kwargs['stationary_joints']==JOINTS+LEFT_JOINTS
    worker.close.assert_called_once()


def test_right_context_keeps_both_original_camera_masks(tmp_path):
    from drum_planner.common import DEFAULT_CALIBRATION,atomic_json
    original=dict(schema='x5-planner-start-state-v1',coordinate_convention='encoder_to_joint',
                  right_rad=[0.]*7,left_rad=[.1]*7,left_gripper_opening_m=0.,right_gripper_closed_motor_joint_rad=float(np.radians(7.)))
    p=tmp_path/'state.json';atomic_json(p,original)
    ctx=generated_planning.context(tmp_path,p,p,p,DEFAULT_CALIBRATION,'right')
    assert ctx['scene_robot_poses']==dict(left=[.1]*7,right=[0.]*7) and ctx['both_tools'] is True


def test_no_fallback_when_left_goals_not_unique(tmp_path):
    (tmp_path/'destinations/left').mkdir(parents=True)
    with patch.object(generated_planning,'ROOT',tmp_path),patch.object(generated_planning,'read_state',return_value={}),\
         patch.object(generated_planning.rgbd,'scene'),patch.object(generated_planning.config,'no_can'):
        with pytest.raises(ValueError,match='exactly ONE'):generated_planning.prepare(tmp_path/'out',tmp_path/'capture')


def test_failed_capture_closes_query_owner(tmp_path):
    from camera_playback import generated_acquisition as a
    worker=Mock(error=None);worker.report.return_value={}
    with patch.object(a,'Telemetry',return_value=worker),patch.object(a,'paired_capture',side_effect=RuntimeError('camera busy')):
        with pytest.raises(RuntimeError,match='camera busy'):a.capture(tmp_path/'capture')
    worker.close.assert_called_once()


def test_real_transport_adapter_setup_only_targets_captured_rest_and_custom_grips():
    from tests.test_left_center_hold import fake_bus
    from centering.motors import Motors,parameter,RIGHT_PLAYBACK_SPEED,RIGHT_GRIPPER_CLOSED
    from camera_playback.left_hold import LEFT_GRIPPER_TARGET
    old=fake_bus();rest={s:np.array([old.states[s,i][0] for i in range(1,8)]) for s in ('left','right')}
    bus=GeneratedMotors.from_owner(old,rest);prepared=[]
    def prepare(drive,motor,q,current):
        side=drive.control_side;prepared.append((side,motor,q))
        bus.states[side,motor]=(q,2,time.monotonic())
        yield
    import time
    with patch.object(Motors,'_prepare_center_motor_confirmed',prepare):list(bus.arm_at_rest())
    assert [s for s,i,q in prepared]==['left']*8+['right']*8
    for s,i,q in prepared:
        assert q==(rest[s][i-1] if i<=7 else LEFT_GRIPPER_TARGET if s=='left' else RIGHT_GRIPPER_CLOSED)
    for side in ('left','right'):
        raw=bus.sockets[side].sock.sock
        assert [c.args[0] for c in raw.send.call_args_list]==[parameter(i,0x7017,RIGHT_PLAYBACK_SPEED) for i in range(1,8)]


def test_failed_feedback_is_not_reused_as_a_healthy_hold_snapshot():
    import time
    from tests.test_left_center_hold import fake_bus
    old=fake_bus();rest={s:[.2]*7 for s in ('left','right')}
    bus=GeneratedMotors.from_owner(old,rest)
    assert bus.fresh()
    bus.feedback_failed_at=time.monotonic()
    assert not bus.fresh()


def test_emergency_requests_both_mit_owners_even_when_gui_feedback_is_stale():
    from camera_playback.generated_app import GeneratedApp
    a=GeneratedApp.__new__(GeneratedApp);a.initialized=True;a.job=None;a.beat_started=True
    a.hihat_calibration=Mock();a.hihat_sync=None;a.hihat=Mock();a.root=Mock()
    right=Mock();left=Mock();a.bus=SimpleNamespace(active=True,right_joint7_session=right,left_joint6_session=left)
    a.motion=Mock();a.motion.hold.return_value='HOLD UNCONFIRMED: stale feedback'
    a.status=Mock();a._refresh_buttons=Mock()
    a.emergency_hold()
    right.emergency_hold.assert_called_once_with();left.emergency_hold.assert_called_once_with()
    a.hihat.release_all.assert_called_once_with();a.motion.hold.assert_called_once()
    assert a.beat_started is False


def test_goal_feedback_only_exempts_the_worker_currently_changing_mode():
    from camera_playback.generated_app import GeneratedApp
    clock,bus,m=rig();m.start();ticks(clock,bus,m,lambda:m.stage=='ARRIVED')
    a=GeneratedApp.__new__(GeneratedApp);a.bus=bus;a.motion=m;a.snare_preparing=True
    bus.left_joint6_session=SimpleNamespace(status=SimpleNamespace(armed=False))
    q,_,stamp=bus.states['left',6];bus.states['left',6]=(q,0,stamp)
    a._goal_feedback()  # This specific drive's confirmed MIT setup owns its transition.
    q,_,stamp=bus.states['left',2];bus.states['left',2]=(q,0,stamp)
    with pytest.raises(RuntimeError,match='not running'):a._goal_feedback()
    bus.states['left',2]=(q,2,stamp)
    bus.left_joint6_session.status.armed=True
    with pytest.raises(RuntimeError,match='not running'):a._goal_feedback()
    q,_,stamp=bus.states['left',6];bus.states['left',6]=(q,2,stamp)
    bus.states['right',8]=(0.,2,stamp)
    with pytest.raises(RuntimeError,match='gripper'):a._goal_feedback()


def test_launcher_keeps_explicit_hold_signal_but_never_time_kills_owner():
    from camera_playback.generated_launch import HeldArmProcess
    from launch.actions import ExecuteProcess,EmitEvent,TimerAction
    from launch.events.process import SignalProcess
    import signal
    request=EmitEvent(event=SignalProcess(signal_number=signal.SIGINT,process_matcher=lambda p:True))
    process=HeldArmProcess(cmd=['/usr/bin/true'])
    with patch.object(ExecuteProcess,'_shutdown_process',return_value=[TimerAction(period=.1,actions=[]),request]):
        assert process._shutdown_process(Mock(),send_sigint=True)==[request]


def test_generated_internal_sync_stop_keeps_worker_ownership_no_auto_center():
    from camera_playback import hybrid_workflow as h
    app=SimpleNamespace(generated_mode=True,emergency_hold=Mock(),hybrid_session=Mock())
    with patch.object(h.snare,'finish') as finish:
        h.request_stop(app,'Timing calibration failed')
    app.emergency_hold.assert_called_once_with('Timing calibration failed')
    finish.assert_not_called();app.hybrid_session.stop.assert_not_called()
