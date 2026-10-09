"""Hardware-mode UI state transitions, mocked transports; never opens CAN."""
from dataclasses import replace
import math
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.app import App, SWING_EVENTS, STRIKE_SOUND_WAIT_PHASE
from camera_playback.hybrid_strike import HybridStatus, load_tuning
from camera_playback import hybrid_workflow as hw
from camera_playback.mit_strike import Sample
from camera_playback.strike import StrikePlan


def app_fixture():
    a = App.__new__(App)
    a.phase = 'STRIKE MOVING OUT'
    a.hybrid_enabled = True
    a.hardware = True
    a.test_mode = a.hardware_test_mode = a.recording_only = False
    a.hybrid_stopping = a.hybrid_restoring = False
    a.hybrid_tuning = load_tuning()
    a.hybrid_anchor = np.zeros(7)
    a.lower, a.upper = np.full(7, -2.), np.full(7, 2.)
    targets = []
    for degrees in range(5, 20):
        q = np.zeros(7)
        q[6] = -math.radians(degrees)
        targets.append(q)
    a.strike_plan = StrikePlan(np.zeros(7), tuple(targets))
    a.strike_index = 0
    a.strike_active = True
    a.strike_hit_pending = None
    a.strike_attempt_motion_seen = False
    a.strike_attempt_started_at = a.strike_attempt_returned_at = None
    a.strike_sound_deadline = None
    a.strike_feedback_fast = a.strike_speed_fast = False
    a.continuous_strike_active = False
    a.continuous_stop_requested = False
    a.hybrid_expected_count = 1
    a.hybrid_seen_completed = a.hybrid_seen_bottoms = a.hybrid_main_count = 0
    a.bus = Mock(active=True)
    a.bus.fresh.return_value = True
    a.arm = Mock(return_value=np.zeros(7))
    a.hybrid_session = Mock()
    a.hybrid_session.status = HybridStatus()
    a.hybrid_session.request.return_value = 8
    a.hill_ik = Mock()
    a.status = Mock()
    a.result_status = Mock()
    a.result_label = Mock()
    a.center_relax_button = Mock()
    a.hihat = Mock()
    a.center_goal = np.ones(7)*.2
    a.control = a.setup = a.hold_until = None
    a._refresh_buttons = Mock()
    a._reset_continuous_j7_measurement = Mock()
    a._log_continuous_j7_minimum = Mock()
    return a


class WorkflowTests(unittest.TestCase):
    def test_hardware_search_uses_worker_not_csp_strike_control(self):
        a = app_fixture()
        a._begin_strike_attempt()
        a.hybrid_session.request.assert_called_once_with('search', math.radians(5))
        self.assertIsNone(a.control)
        self.assertIsNone(a.strike_attempt_started_at)
        a.bus.set_positions.assert_not_called()
        a.bus.set_right_joint7_speed.assert_not_called()

    def test_prepare_keeps_camera_anchor_and_intersects_experiment_depth_limit(self):
        a = app_fixture()
        a.hybrid_session = None
        with patch.object(hw, '_path_inside', return_value=True), patch.object(hw, 'HybridSession') as factory:
            hw.prepare(a)
        self.assertEqual(len(a.strike_plan.targets), 8)  # 5..12; beat corridor is 13°.
        self.assertEqual(factory.call_args.args[1], 0.)
        self.assertAlmostEqual(factory.call_args.args[2], -math.radians(13))
        self.assertEqual(a.hybrid_seen_completed, 0)

    def test_no_hit_advances_one_degree_only_after_complete_return_and_sound_wait(self):
        a = app_fixture()
        a.hybrid_session.status = HybridStatus(phase='hold', ready=True, completed=1,
            count=1, peak_drop=math.radians(5), released_at=1., returned_at=1.4,
            sample=Sample(0, 0, .745, 2, 1.4), at=1.4)
        hw.tick(a, 1.4)
        self.assertEqual(a.phase, STRIKE_SOUND_WAIT_PHASE)
        self.assertEqual(a.strike_sound_deadline, 4.4)
        a.hybrid_session.request.assert_not_called()
        a._finish_no_hit_attempt()
        self.assertEqual(a.strike_index, 1)
        a.hybrid_session.request.assert_called_once_with('search', math.radians(6))
        self.assertEqual(a.hybrid_expected_count, 2)

    def test_sound_uses_worker_release_timestamp_and_latches_until_full_return(self):
        a = app_fixture()
        a.hybrid_session.status = HybridStatus(phase='coast', count=1,
            peak_drop=math.radians(4), released_at=10., sample=Sample(-.07, -.5, 0, 2, 10.05), at=10.05)
        a._process_sound_hit({'event_at': 9.})
        self.assertIsNone(a.strike_hit_pending)
        a._process_sound_hit({'event_at': 10.06})
        self.assertEqual(a.strike_hit_pending['degrees'], 5)
        a.hybrid_session.request.assert_not_called()
        a.hybrid_session.status = replace(a.hybrid_session.status, phase='hold', ready=True,
            completed=1, returned_at=10.4, sample=Sample(0, 0, .745, 2, 10.4), at=10.4)
        with patch('camera_playback.app.time.monotonic', return_value=10.4):
            hw.tick(a, 10.4)
        self.assertTrue(a.continuous_strike_active)
        self.assertEqual(a.continuous_strike_degrees, 5.5)
        a.hybrid_session.request.assert_called_once_with('swing', math.radians(5.5), SWING_EVENTS)
        a.hihat.start_sequence.assert_called_once()

    def test_return_before_sound_display_tick_still_rejects_late_onset(self):
        a = app_fixture()
        a.hybrid_session.status = HybridStatus(phase='hold', count=1, completed=1,
            peak_drop=.1, released_at=10., returned_at=10.4, at=10.4)
        a._process_sound_hit({'event_at': 10.6})
        self.assertIsNone(a.strike_hit_pending)

    def test_sound_does_not_start_swing_after_worker_fault(self):
        a = app_fixture()
        a.phase = STRIKE_SOUND_WAIT_PHASE
        a.hybrid_session.status = HybridStatus(phase='fault', count=1, completed=1,
            peak_drop=.1, released_at=10., returned_at=10.4, at=10.4, error='CAN fault')
        a._process_sound_hit({'event_at': 10.2})
        self.assertIsNone(a.strike_hit_pending)
        a.hihat.start_sequence.assert_not_called()
        a.hybrid_session.request.assert_not_called()

    def test_only_main_beat_deadlines_send_hihat_once(self):
        a = app_fixture()
        a.strike_active = False
        a.continuous_strike_active = True
        a.continuous_strike_degrees = 5
        a.continuous_hihat_pending = False
        a.hybrid_session.status = HybridStatus(phase='withdrawal', swing=True,
            count=1, bottoms=1, label='pickup', sample=Sample(-.05, 1, .8, 2, 5.), at=5.)
        hw.tick(a, 5.)
        a.hihat.send_beat.assert_not_called()
        a.hybrid_session.status = replace(a.hybrid_session.status, count=2, bottoms=2,
            main_count=1, main_at=5.2, label='beat 1', main_beat=True,
            sample=Sample(-.05, 1, .8, 2, 5.2), at=5.2)
        hw.tick(a, 5.21)
        hw.tick(a, 5.22)
        a.hihat.send_beat.assert_called_once()
        a.bus.set_positions.assert_not_called()

    def test_stop_waits_for_ack_full_return_join_and_fresh_csp_confirmation(self):
        a = app_fixture()
        a.continuous_strike_active = True
        session = a.hybrid_session
        session.status = HybridStatus(phase='withdrawal', swing=True, count=2,
            at=10., sample=Sample(-.05, .4, .8, 2, 10.))
        a._request_continuous_stop()
        self.assertEqual(a.phase, hw.FINISHING)
        session.request.assert_called_once_with('finish')
        hw.tick(a, 10.)
        session.stop.assert_not_called()
        a.bus.restore_right_joint7_csp.assert_not_called()
        a.bus.restore_right_joint7_csp.return_value = 10.3
        session.status = replace(session.status, request_id=8, ready=True, swing=False,
            phase='hold', at=10.3, sample=Sample(0, 0, .745, 2, 10.3))
        with patch.object(hw.time, 'monotonic', return_value=10.3):
            hw.tick(a, 10.3)
        session.stop.assert_called_once()
        self.assertIsNone(a.hybrid_session)
        a.bus.restore_right_joint7_csp.assert_called_once_with(0.)
        self.assertEqual(a.phase, hw.RESTORING)
        a.begin_stage = Mock()
        a.bus.right_joint7_mode_readback.return_value = 5
        a.bus.right_joint7_operating_state.return_value = (2, 10.2)  # Old enable reply rejected.
        hw.tick(a, 10.31)
        a.begin_stage.assert_not_called()
        a.bus.right_joint7_operating_state.return_value = (2, 10.32)
        hw.tick(a, 10.33)
        a.begin_stage.assert_called_once()
        self.assertEqual(a.begin_stage.call_args.args[0], 'CENTER RELAX RECENTERING')
        a.bus.relax.assert_not_called()  # Existing center completion owns disable.

    def test_stop_cannot_accept_stale_ready_status_before_finish_ack(self):
        a = app_fixture()
        a.hybrid_session.status = HybridStatus(ready=True, request_id=7, at=5,
            sample=Sample(0, 0, .745, 2, 5))
        hw.request_stop(a)
        hw.tick(a, 5)
        a.hybrid_session.stop.assert_not_called()

    def test_fault_joins_before_hold_and_never_sends_center_goal_in_mit(self):
        a = app_fixture()
        timeline = Mock()
        timeline.attach_mock(a.hybrid_session.stop, 'join')
        timeline.attach_mock(a.bus.hold_right_joint7_unknown_mode, 'hold')
        hw.fault(a, 'feedback lost')
        self.assertEqual([c[0] for c in timeline.mock_calls], ['join', 'hold'])
        self.assertEqual(a.phase, hw.FAULT)
        a.bus.set_positions.assert_not_called()
        a.bus.restore_right_joint7_csp.assert_not_called()

    def test_other_modes_never_enter_hybrid_branch(self):
        a = app_fixture()
        a.hybrid_enabled = False
        a._begin_strike_stage = Mock()
        a._begin_strike_attempt()
        a._begin_strike_stage.assert_called_once()
        a.hybrid_session.request.assert_not_called()

    def test_failed_join_retains_worker_ownership_and_forbids_recovery_writes(self):
        a = app_fixture()
        a.hybrid_session.stop.side_effect = RuntimeError('join failed')
        hw.fault(a, 'test failure')
        self.assertIsNotNone(a.hybrid_session)
        a.bus.hold_right_joint7_unknown_mode.assert_not_called()
        a.bus.restore_right_joint7_csp.assert_not_called()
        a.bus.set_positions.assert_not_called()

    def test_missing_csp_confirmation_never_starts_centering(self):
        a = app_fixture()
        a.hybrid_session = None
        a.hybrid_restoring = True
        a.hybrid_csp_at = 1.
        a.hybrid_restore_deadline = 3.
        a.bus.right_joint7_mode_readback.return_value = 0
        a.bus.right_joint7_operating_state.return_value = (2, 3.1)
        a.begin_stage = Mock()
        hw.tick(a, 3.1)
        self.assertEqual(a.phase, hw.FAULT)
        a.begin_stage.assert_not_called()

    def test_soft_failure_during_search_finishes_return_instead_of_csp_recenter(self):
        a = app_fixture()
        a.begin_stage = Mock()
        a._program_failure('ST7 disconnected')
        a.hybrid_session.request.assert_called_once_with('finish')
        a.begin_stage.assert_not_called()
        self.assertEqual(a.phase, hw.FINISHING)


if __name__ == '__main__':
    unittest.main()
