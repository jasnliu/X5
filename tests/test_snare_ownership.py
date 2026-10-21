"""Startup/return regression: idle path hold is not strike-anchor readiness.

All feedback below is synthetic. These tests never open hardware.
"""
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from camera_playback.app import App
from camera_playback.dual_recording import FAULT, PLAYING
from camera_playback.snare_beat import SnareSession, SnareStatus
from camera_playback.mit_strike import Sample
from tests.test_snare_beat import SnareBeatTests


class SessionView(SnareSession):
    """Exercise the real finished predicate without spawning a motor worker."""
    def __init__(self, status):
        self.current = status
        self.clock = [0., 0., 0., 0., 0.]

    @property
    def status(self):
        return self.current


def idle_status():
    return SnareStatus(at=10., sample=Sample(0., 0., 0., 2, 10.),
                       armed=True, ready=False)


def test_idle_path_hold_can_yield_without_being_at_the_snare_anchor():
    with patch('camera_playback.snare_beat.time.monotonic', return_value=10.):
        assert SessionView(idle_status()).finished


@pytest.mark.parametrize('changes', [
    dict(moving=True), dict(scheduled=True), dict(armed=False),
    dict(error='worker fault'), dict(sample=None), dict(at=9.8),
    dict(sample=Sample(0., 0., 0., 2, 9.97)),
    dict(sample=Sample(0., 0., 0., 0, 10.)),
    dict(sample=Sample(0., 0., 0., 2, 11.)),
    dict(sample=Sample(0., 0., 0., 2, float('nan'))),
])
def test_active_or_unproven_worker_never_yields(changes):
    with patch('camera_playback.snare_beat.time.monotonic', return_value=10.):
        assert not SessionView(replace(idle_status(), **changes)).finished


def test_pending_start_cannot_yield_before_worker_acknowledges_it():
    s = SessionView(idle_status())
    s.clock[2] = 1.
    with patch('camera_playback.snare_beat.time.monotonic', return_value=10.):
        assert not s.finished


@pytest.fixture(scope='module')
def rig():
    SnareBeatTests.setUpClass()
    return SnareBeatTests()


def test_real_controller_start_endpoint_and_center_path_handoffs(rig):
    # This is the physical-run regression: first_joints != strike anchor.
    c = rig.controller(initial=0.)
    q, v = 0., 0.
    s = SessionView(c.status)
    now = 0.
    for goal in (rig.recording.first_joints[5], rig.recording.last_joints[5], 0.):
        for _ in range(2000):
            now += .002
            cmd = c.update(rig.sample(now, q, v), now, float(goal))
            q, v = cmd.position, cmd.velocity
        s.current = c.status
        with patch('camera_playback.snare_beat.time.monotonic', return_value=now):
            assert s.finished
        assert not c.status.moving and c.status.completed == 0
        assert abs(q + goal) < .001
    assert not c.status.ready  # Center must not be confused with strike-ready.


def test_left_path_uses_same_sample_native_velocity_not_command_loop_derivative(rig):
    c = rig.controller(initial=0.)
    assert c.path.bias_gain == 40.
    assert (c.path.kp, c.path.kd) == (160., 4.)
    with patch.object(c.path, 'update', wraps=c.path.update) as update:
        c.update(rig.sample(.002, .01, .025), .002, 0.)
    update.assert_called_once_with(.01, .002, measured_velocity=.025)


def test_finish_during_strike_waits_for_full_return_and_measured_settling(rig):
    c = rig.controller((1,))
    rig.prime(c)
    c.start(1., .6)
    q, v = rig.anchor, 0.
    s = SessionView(c.status)
    stopped = False
    for i in range(60, 1500):
        now = i * .002
        cmd = c.update(rig.sample(now, q, v), now, -rig.anchor)
        q, v = cmd.position, cmd.velocity
        if c.method is not None and not stopped:
            c.finish()
            stopped = True
        if stopped:
            s.current = c.status
            with patch('camera_playback.snare_beat.time.monotonic', return_value=now):
                assert s.finished == (c.status.completed == 1)
            if s.current.completed:
                assert now-c.status.released_at >= c.template.curve.duration
                assert c.status.ready
                assert c.path.bias_gain == 40.
                assert (c.path.kp, c.path.kd) == (160., 4.)
                break
    else:
        pytest.fail('Full reference return did not complete')


@pytest.mark.parametrize('phase,closing,emergency,required', [
    (FAULT, False, False, False),
    (PLAYING, True, False, False),
    (PLAYING, False, True, False),
    ('CENTER RELAX RECENTERING', False, False, False),
    (PLAYING, False, False, True),
])
def test_audio_teardown_does_not_reenter_latched_motor_recovery(phase, closing, emergency, required):
    a = App.__new__(App)
    a.phase, a.close_requested, a.emergency_latched = phase, closing, emergency
    a.bus = Mock(active=True)
    a.hardware_test_mode = False
    a.audio_receiver = Mock(state='stopped', detail='Shared audio listener stopped')
    a.audio_receiver.poll.return_value = []
    a.root = Mock()
    a.sound_status = a.sound_label = Mock()
    a._refresh_buttons = Mock()
    a._program_failure = Mock()
    a.sound_fault_detail = None
    assert a._sound_required_now() == required
    if not required:
        for _ in range(5):
            a.sound_tick()
        a._program_failure.assert_not_called()
        a.bus.disable_all.assert_not_called()
        assert a.sound_fault_detail == 'Shared audio listener stopped'
