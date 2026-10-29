"""Return after feedback/worker interruption: fake devices only."""
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch
import time
import numpy as np
import pytest

from camera_playback.mit_strike import Joint7Session, Sample, Status
from camera_playback.generated_transport import GeneratedMotors
from tests.test_left_center_hold import fake_bus
from safe_zone.encoder import request_frame


def dead_worker(bus):
    worker = Joint7Session.__new__(Joint7Session)
    worker.closed = False
    worker.process = SimpleNamespace(is_alive=lambda: False, exitcode=0)
    worker.stop_event = SimpleNamespace(is_set=lambda: False)
    worker.receiver = Mock(recv=Mock(side_effect=BlockingIOError))
    worker._status = Status(sample=Sample(.2, 0., 0., 2, time.monotonic()-1.))
    worker.bus = bus
    return worker


def test_stopped_strike_worker_cannot_suppress_live_encoder_queries_forever():
    owner = fake_bus()
    bus = GeneratedMotors.from_owner(owner, {s:np.full(7, .2) for s in ('left','right')})
    bus.right_joint7_session = dead_worker(bus)
    bus.active = bus.left_drive.active = True
    q, mode, _ = bus.states['right', 7]
    bus.states['right', 7] = (q, mode, time.monotonic()-1.)
    with pytest.raises(RuntimeError, match='Encoder feedback lost'):
        bus.poll()
    assert request_frame(7) in [c.args[0] for c in bus.sockets['right'].sock.sock.send.call_args_list]


@pytest.mark.parametrize('alive,age,expected', [(False, .01, False), (True, .2, False), (True, .01, True)])
def test_feedback_ownership_requires_live_process_and_recent_sample(alive, age, expected):
    worker = dead_worker(None)
    worker.process = SimpleNamespace(is_alive=lambda: alive, exitcode=None if alive else 0)
    worker._status = Status(sample=Sample(.2, 0., 0., 2, time.monotonic()-age))
    assert worker.owns_feedback is expected


def test_fresh_queries_clear_feedback_fault_without_reenable_or_mode_change():
    owner=fake_bus();bus=GeneratedMotors.from_owner(owner,{s:np.full(7,.2) for s in ('left','right')})
    bus.right_joint7_session=dead_worker(bus);bus.active=bus.left_drive.active=True
    bus.states={k:(q,state,time.monotonic()-1.) for k,(q,state,at) in bus.states.items()}
    with pytest.raises(RuntimeError,match='Encoder feedback lost'):bus.poll()
    assert bus.feedback_failed_at is not None and not bus.fresh()
    bus.states={k:(q,2,time.monotonic()) for k,(q,state,at) in bus.states.items()}
    bus.poll();assert bus.fresh() and bus.feedback_failed_at is None
    from safe_zone.encoder import FRAME
    for side in ('right','left'):
        for call in bus.sockets[side].sock.sock.send.call_args_list:
            kind=(FRAME.unpack(call.args[0])[0]>>24)&31
            assert kind in (2,17)  # Read-only state/parameter queries.


@pytest.mark.parametrize('side', ['right','left'])
@pytest.mark.parametrize('case', ['healthy','wrong_mode','disabled','stale','drive_fault'])
def test_recovery_worker_uses_only_existing_powered_hold_and_checked_targets(side,case):
    import pickle,threading
    from camera_playback.generated_worker_hold import recover_worker
    from safe_zone.encoder import FRAME
    t=[0.];stop=threading.Event();goal=SimpleNamespace(value=float('nan'))
    heartbeat=SimpleNamespace(value=0.);states=[];frames=[];closed=[]
    motor,sign=(7,1.) if side=='right' else (6,-1.)
    class Channel:
        def __init__(self,interface,**kw):
            assert kw==dict(motor=motor,sign=sign)
            self.sample=Sample(sign*.3,0.,0.,2,0.);self.mode=None
        def receive(self):
            t[0]+=.002;heartbeat.value=t[0]
            if case=='drive_fault':raise RuntimeError('drive fault')
            self.mode=1 if case=='wrong_mode' else 0
            self.sample=replace(self.sample,state=0 if case=='disabled' else 2,
                                at=t[0]-1. if case=='stale' else t[0])
            if t[0]>.1 and case=='healthy':goal.value=.2
            if t[0]>1.5 and case=='healthy':stop.set()
        def send(self,frame):
            cid,_,data=FRAME.unpack(frame);kind=(cid>>24)&31
            assert cid&255==motor and kind in (1,2,17)
            if kind==1:
                raw=int.from_bytes(data[:2],'big')/65535*25.14-12.57
                internal=-sign*raw
                self.sample=replace(self.sample,position=internal)
                frames.append((t[0],sign*internal))
        def close(self):closed.append(True)
    sender=SimpleNamespace(send=lambda b:states.append(pickle.loads(b)),close=lambda:None)
    with patch('time.monotonic',side_effect=lambda:t[0]),patch('time.sleep',side_effect=lambda dt:t.__setitem__(0,t[0]+dt)),patch('signal.signal'):
        recover_worker('fake',side,-1.,1.,goal,stop,heartbeat,sender,Channel)
    assert closed==[True]
    if case=='healthy':
        assert frames and not any(s.error for s in states)
        assert abs(frames[0][1]-.3)<.001 and abs(frames[-1][1]-.2)<.01
        assert all(abs(q-.3)<.001 for at,q in frames if at<.1)
    else:
        assert not frames and states[-1].error


def recovery_bus():
    bus=GeneratedMotors.from_owner(fake_bus(),{s:np.full(7,.2) for s in ('left','right')})
    bus.active=bus.left_drive.active=True;bus.fresh=Mock(return_value=True)
    bus.states={k:(q,2,at) for k,(q,_,at) in bus.states.items()}
    bus.stream_geometry={s:SimpleNamespace(lower=np.full(7,-1.),upper=np.full(7,1.)) for s in ('left','right')}
    return bus


@pytest.mark.parametrize('side,motor',[('right',7),('left',6)])
def test_dead_owner_is_joined_before_recovery_and_healthy_owner_is_unchanged(side,motor):
    bus=recovery_bus();events=[];attr=f'{side}_joint{motor}_session'
    old=SimpleNamespace(status=Status(error='stopped'),process=Mock(is_alive=Mock(return_value=False)))
    old.stop=lambda:(events.append('joined'),setattr(bus,attr,None))
    setattr(bus,attr,old)
    def create(bus,side,lo,hi):
        assert getattr(bus,attr) is None;events.append('created')
        new=SimpleNamespace(status=Status(),process=Mock(is_alive=Mock(return_value=True)),hold_position=.2)
        setattr(bus,attr,new);return new
    with patch('camera_playback.generated_worker_hold.RecoverySession',side_effect=create) as constructor:
        assert not bus.recover_return_workers()
        assert events==['joined','created'] and bus.recover_return_workers()
        constructor.assert_called_once()
    assert not bus.sockets[side].sock.sock.send.called  # No parent setup/enable/disable.


def test_live_or_disabled_failed_owner_is_not_replaced_and_retries_are_bounded():
    bus=recovery_bus();worker=SimpleNamespace(status=Status(error='fault'),process=Mock(is_alive=Mock(return_value=True)),stop=Mock())
    bus.right_joint7_session=worker
    with patch('camera_playback.generated_worker_hold.RecoverySession') as create:
        assert not bus.recover_return_workers();create.assert_not_called();worker.stop.assert_not_called()
        worker.process.is_alive.return_value=False;bus.return_recovery_attempted={'right'}
        with pytest.raises(RuntimeError,match='could not start'):bus.recover_return_workers()
        bus.return_recovery_attempted=set();q,_,at=bus.states['right',7];bus.states['right',7]=(q,0,at)
        with pytest.raises(RuntimeError,match='not powered'):bus.recover_return_workers()
        create.assert_not_called();worker.stop.assert_not_called()


def test_return_click_survives_transient_feedback_failure_but_hold_cancels():
    from tests.test_generated_stop_return import app_stub
    a=app_stub();a.job=None;a.beat_started=False;a.motion.stage='HOLD';a.last_error=None
    a.return_to_rest();assert a.return_requested
    a.fail('Encoder feedback lost');assert a.return_requested
    a.fail('Encoder feedback lost');assert a.return_requested
    a.emergency_hold();assert not a.return_requested
    a.bus.emergency_release.assert_not_called()


def test_repeated_encoder_fault_stops_a_new_return_not_just_updates_label():
    from tests.test_generated_stop_return import app_stub
    a=app_stub();a.job=None;a.motion.stage='RETURN';a.last_error='Encoder feedback lost';a.return_requested=False
    a.fail('Encoder feedback lost');a.motion.hold.assert_called_once()
    a.motion.stop_stream.assert_called_once();a.bus.emergency_release.assert_not_called()


def test_offline_guard_forbids_recovery_hardware_constructor(monkeypatch):
    from camera_playback.generated_worker_hold import RecoverySession
    monkeypatch.setenv('STRIKE_LAB_OFFLINE_ONLY','1')
    with pytest.raises(RuntimeError,match='Physical recovery transport forbidden'):
        RecoverySession(None,'right',-1.,1.)


def test_peripheral_cleanup_error_cannot_block_arm_hold_or_return_request():
    from tests.test_generated_stop_return import app_stub
    a=app_stub();a.job=None;a.beat_started=False;a.motion.stage='LEFT'
    a.hihat_calibration.close.side_effect=RuntimeError('ESP32 unavailable')
    a.return_to_rest()
    assert a.return_requested;a.motion.hold.assert_called_once()
    a.bus.emergency_release.assert_not_called()


def test_pending_return_waits_for_healthy_feedback_then_launches_once():
    from tests.test_generated_stop_return import app_stub
    a=app_stub();a.job=None;a.return_requested=True;a.motion.pending_worker_hold=True
    a.bus.recover_return_workers.return_value=False;a._launch_return=Mock()
    a._service_return();a._launch_return.assert_not_called();a.motion.step.assert_not_called()
    a.bus.recover_return_workers.return_value=True;a.motion.pending_worker_hold=False
    a.bus.settled.return_value=False;a._service_return();a._launch_return.assert_not_called()
    a.bus.settled.return_value=True;a._service_return();a._launch_return.assert_called_once()


def test_missing_cache_queues_offline_planner_with_measured_hold_not_center(tmp_path):
    from tests.test_generated_stop_return import app_stub
    from tests.test_generated_beat import rig
    a=app_stub();_,_,m=rig();a.curves=m.curves;a.mission={};a.mission_path=tmp_path/'mission.json';a.session=tmp_path
    data=dict(held={s:c.at(c.duration).tolist() for s,c in m.curves.items()},progress=dict(left=1.,right=1.))
    a.motion.return_input.return_value=data;a._spawn=Mock();a.return_requested=True
    a._launch_return()
    assert not a.return_requested;a.motion.accept_return.assert_not_called()
    call=a._spawn.call_args;assert call.args[0]=='RETURN' and 'camera_playback.generated_planning' in call.args[1]
    assert 'allow_can' not in call.kwargs  # Default Job containment denies CAN.
    import json
    assert json.loads((call.args[2]/'input.json').read_text())==dict(mission=str(a.mission_path),**data)


def test_return_during_existing_return_is_idempotent():
    from tests.test_generated_stop_return import app_stub
    for stage in ('RETURN','SETTLE RETURN','SETTLE REST','RELAXING'):
        a=app_stub();a.job=None;a.motion.stage=stage;a.return_requested=False
        a.return_to_rest();assert not a.return_requested;a.motion.hold.assert_not_called()


def test_failed_return_calculation_clears_job_and_allows_same_window_retry(tmp_path):
    import json
    from tests.test_generated_stop_return import app_stub
    a=app_stub();a.job_kind='RETURN';a.motion.stage='LOADING PREPARED RETURN'
    a.job=SimpleNamespace(poll=lambda:1,folder=tmp_path,cancelled=False)
    (tmp_path/'error.json').write_text(json.dumps(dict(error='Injected infeasible test route')))
    with pytest.raises(RuntimeError,match='Injected infeasible') as exc:a._poll_job()
    assert a.job is None and a.job_kind is None
    a.last_error=None;a.return_requested=False;a.fail(str(exc.value))
    a.motion.stage='HOLD';a.beat_started=False
    a.return_to_rest();assert a.return_requested
    a.bus.emergency_release.assert_not_called()


def test_cancelled_calculation_never_accepts_a_late_return_result(tmp_path):
    from tests.test_generated_stop_return import app_stub
    a=app_stub();a.job_kind='RETURN';a.motion.stage='HOLD'
    a.job=SimpleNamespace(poll=lambda:0,folder=tmp_path,cancelled=True)
    a._poll_job();a.motion.accept_return.assert_not_called()
    assert a.job is None and a.job_kind is None


def test_new_return_curve_math_never_blocks_hardware_parent_heartbeats():
    from tests.test_generated_beat import rig
    from camera_playback.generated_stream import reference
    clock,bus,m=rig();bus.active=True;bus.streaming_enabled=True;bus.poll()
    m.stage='LOADING PREPARED RETURN'
    step=dict(kind='planned',side='right',duration=1.,start=bus.q['right'].tolist(),plan={})
    result=dict(status='ready',held={s:q.tolist() for s,q in bus.q.items()},progress=m.progress.copy(),steps=[step])
    with patch('drum_planner.baseline.curve.Curve',side_effect=AssertionError('Heavy math belongs in child')):
        m.accept_return(result)
        q,tau=reference(m.curves['right'],step,0.)
    assert m.stage=='RETURN' and m.return_curves=={} and tau is None
    np.testing.assert_array_equal(q,bus.q['right'])
