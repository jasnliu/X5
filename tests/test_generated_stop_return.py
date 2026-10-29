"""Explicit relax and prepared-return regressions; no hardware interfaces."""
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import pytest
from camera_playback.generated_app import GeneratedApp
from camera_playback.generated_transport import GeneratedMotors
from camera_playback.generated_planning import cached_return
from tests.test_generated_beat import rig


def fake_owner():
    from tests.test_left_center_hold import fake_bus
    bus=GeneratedMotors.from_owner(fake_bus(),{s:np.zeros(7) for s in ('left','right')})
    bus.active=bus.left_drive.active=True
    bus.states={};bus.fresh=Mock(side_effect=AssertionError('No feedback gate allowed'))
    bus.settled=Mock(side_effect=AssertionError('No rest gate allowed'))
    return bus


def test_emergency_disables_all_16_drives_without_feedback_or_pose_checks():
    from safe_zone.encoder import FRAME
    bus=fake_owner();r=Mock();l=Mock();bus.right_joint7_session=r;bus.left_joint6_session=l
    with patch('camera_playback.emergency.stop_writer') as stop:bus.emergency_release()
    assert stop.call_count==2
    r.stop_event.set.assert_called_once();l.stop_event.set.assert_called_once()
    for side in ('right','left'):
        frames=[call.args[0] for call in bus.sockets[side].sock.sock.send.call_args_list]
        assert len(frames)==24
        assert [FRAME.unpack(f)[0]&255 for f in frames]==list(range(1,9))*3
        assert all((FRAME.unpack(f)[0]>>24)&31==4 for f in frames)
    assert not bus.active and bus.emergency_latched
    bus.fresh.assert_not_called();bus.settled.assert_not_called()
    with pytest.raises(RuntimeError):bus.command_arm('right',np.zeros(7))


def test_one_failed_writer_or_bus_cannot_prevent_other_arm_disable_and_retry():
    bus=fake_owner();bus.right_joint7_session=Mock()
    raw=bus.sockets['right'].sock.sock
    raw.send.side_effect=OSError('disconnected')
    with patch('camera_playback.emergency.stop_writer',side_effect=RuntimeError('writer failed')):
        with pytest.raises(RuntimeError,match='Disable unconfirmed'):bus.emergency_release()
    assert bus.sockets['left'].sock.sock.send.call_count==24
    assert raw.send.call_count==24 and bus.active
    raw.send.side_effect=None;raw.send.return_value=16;bus.right_joint7_session=None
    bus.emergency_release();assert not bus.active


def app_stub():
    a=GeneratedApp.__new__(GeneratedApp);a.initialized=True;a.emergency_latched=False
    a.motion=Mock(setup=None);a.bus=Mock();a.job=Mock();a.root=Mock();a.status=Mock()
    a.hihat=Mock();a.hihat_calibration=Mock();a.hihat_sync=None;a.snare_sync=None
    a._refresh_buttons=Mock();return a


def test_relax_survives_peripheral_and_cancellation_failures_and_never_holds():
    a=app_stub();a.hihat.release_all.side_effect=OSError('ESP32 disconnected')
    a.job.cancel.side_effect=OSError('planner already exited');a.hihat_calibration.close.side_effect=OSError('log unavailable')
    a.emergency_relax()
    a.bus.emergency_release.assert_called_once();a.motion.hold.assert_not_called()
    assert a.emergency_latched and not a.beat_started and a.motion.emit.call_args.args==('RELAXED',)
    a.emergency_hold();a.motion.hold.assert_not_called()  # A late fault cannot re-enable/retarget.


def test_close_always_exits_even_when_disable_or_ui_reports_error():
    a=app_stub();a.bus.emergency_release.side_effect=RuntimeError('CAN disconnected')
    a.request_safe_close();assert a.closed;a.root.quit.assert_called_once()
    a=app_stub();a._refresh_buttons.side_effect=RuntimeError('UI destroyed')
    with pytest.raises(RuntimeError):a.request_safe_close()
    assert a.closed;a.root.quit.assert_called_once();a.bus.emergency_release.assert_called_once()


def full_cache(curves):
    from drum_planner.baseline.paths import recovery
    with patch('drum_planner.baseline.paths.peaks',return_value=np.ones((3,7))*.1):
        steps=[]
        for side in ('right','left'):
            c=curves[side];spec=recovery(c,c.at(c.duration),c.duration,.5)
            steps.append(dict(kind='prefix',side=side,spec=spec,duration=spec['bridge_s']+spec['return_s']))
    return dict(return_plan=dict(status='ready',steps=steps))


def test_full_return_reuses_upfront_result_with_no_solver_or_native_check():
    _,_,m=rig();obj=full_cache(m.curves);q={s:c.at(c.duration).tolist() for s,c in m.curves.items()}
    with patch('camera_playback.generated_planning.hard_check',side_effect=AssertionError('No end planning')),\
         patch('camera_playback.generated_planning.recovery',side_effect=AssertionError('Already prepared')):
        result=cached_return(obj,m.curves,q,dict(left=1.,right=1.))
    assert result['steps']==obj['return_plan']['steps']
    assert [s['side'] for s in result['steps']]==['right','left']


def test_partial_return_only_crops_proven_geometry_never_commands_untraveled_right():
    _,_,m=rig();obj=full_cache(m.curves);progress=dict(left=.4,right=0.)
    q={s:c.at(progress[s]).tolist() for s,c in m.curves.items()}
    with patch('camera_playback.generated_planning.hard_check',side_effect=AssertionError('No worker')),\
         patch('drum_planner.baseline.paths.peaks',return_value=np.ones((3,7))*.1):
        result=cached_return(obj,m.curves,q,progress)
    assert [s['side'] for s in result['steps']]==['left']
    assert result['steps'][0]['spec']['tau']<=.4
    q['left'][0]+=.1
    with pytest.raises(ValueError,match='measured-start return'):cached_return(obj,m.curves,q,progress)


def test_normal_beat_return_finishes_strokes_then_uses_cache_not_emergency_freeze():
    a=app_stub();a.emergency_latched=False;a.job=None;a.beat_started=True
    a.motion.stage='ARRIVED';a.bus.active=True
    a.bus.right_joint7_session=Mock();a.bus.left_joint6_session=Mock()
    a.return_to_rest()
    a.bus.right_joint7_session.request.assert_called_once_with('finish')
    a.bus.left_joint6_session.finish.assert_called_once()
    a.bus.right_joint7_session.emergency_hold.assert_not_called()
    a.motion.emit.assert_called_once_with('WAIT PRIME')
    assert not a.beat_started


def test_failed_finish_request_cancels_beat_instead_of_leaving_a_worker_swinging():
    a=app_stub();a.job=None;a.beat_started=True;a.motion.stage='ARRIVED';a.bus.active=True
    a.bus.right_joint7_session=Mock();a.bus.right_joint7_session.request.side_effect=RuntimeError('queue full')
    a.emergency_hold=Mock()
    a.return_to_rest();a.emergency_hold.assert_called_once()
    assert 'queue full' in a.emergency_hold.call_args.args[0]


def test_preview_includes_both_return_legs_and_ends_at_rest():
    from camera_playback.generated_preview import CombinedModel
    _,_,m=rig();model=CombinedModel.__new__(CombinedModel)
    model.curves=m.curves;model.return_steps=full_cache(m.curves)['return_plan']['steps']
    model.outward_duration=2.;model.goal_pause=2.
    model.duration=4.+sum(s['duration'] for s in model.return_steps)
    assert 'RIGHT RETURN' in model.phase(4.1)
    assert 'LEFT RETURN' in model.phase(4.+model.return_steps[0]['duration']+.1)
    for side in ('right','left'):
        got=[model.values(model.duration)[f'openarmx_{side}_joint{i}'] for i in range(1,8)]
        np.testing.assert_allclose(got,m.rest[side])
