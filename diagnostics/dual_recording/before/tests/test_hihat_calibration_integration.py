"""App gating and actual ESP32 source, without physical hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np

from camera_playback.app import App, HIHAT_CALIBRATION_WAIT_PHASE, HiHatCalibrationHold


class AppGateTests(unittest.TestCase):
    def test_stationary_wait_can_exceed_normal_thirty_second_move_timeout(self):
        control = HiHatCalibrationHold(np.zeros(7), -np.ones(7), np.ones(7), 0.)
        for now in np.arange(0, 100, .05):
            command, error, reached = control.update(np.zeros(7), now)
        self.assertTrue(reached)
        np.testing.assert_array_equal(command, np.zeros(7))

    def test_wait_hold_preserves_stall_detection(self):
        control = HiHatCalibrationHold(np.zeros(7), -np.ones(7), np.ones(7), 0.)
        actual = np.zeros(7); actual[2] = np.deg2rad(10)
        with self.assertRaisesRegex(RuntimeError, 'STALL: motor 3'):
            for now in np.arange(0, 3, .05): control.update(actual, now)

    def test_wait_hold_rejects_invalid_feedback(self):
        control = HiHatCalibrationHold(np.zeros(7), -np.ones(7), np.ones(7), 0.)
        with self.assertRaisesRegex(RuntimeError, 'Invalid encoder'):
            control.update(np.full(7, np.nan), 1.)

    def fixture(self, ready=False):
        a = App.__new__(App)
        a.hihat_calibration = SimpleNamespace(ready=ready, engine=SimpleNamespace(failure=None))
        a.playback_active = True
        a.alignment_active = False
        a._reset_hold_controller = Mock()
        a.status = Mock()
        a.fail = Mock()
        return a

    def test_endpoint_wait_holds_without_beginning_ride_or_alignment(self):
        a = self.fixture()
        a._begin_strike_attempt = Mock()
        a._begin_playback_alignment(1.)
        self.assertEqual(a.phase, HIHAT_CALIBRATION_WAIT_PHASE)
        self.assertFalse(a.playback_active)
        self.assertFalse(a.alignment_active)
        a._reset_hold_controller.assert_called_once()
        a._begin_strike_attempt.assert_not_called()

    def test_picker_return_cannot_start_blocking_preflight_during_calibration(self):
        a = self.fixture()
        a.hihat_calibration.busy = True
        with patch('camera_playback.app.load_smooth_recording') as loader:
            self.assertFalse(a.load_recording('record3.json'))
        loader.assert_not_called()

    def test_reaching_wait_pose_does_not_clear_hold_or_relax(self):
        a = self.fixture()
        a.phase = HIHAT_CALIBRATION_WAIT_PHASE
        hold = a.control = object()
        a.relax = Mock()
        a.complete_stage(2.)
        self.assertIs(a.control, hold)
        a.relax.assert_not_called()

    def test_alignment_and_direct_strike_cannot_bypass_gate(self):
        a = self.fixture()
        a.phase = 'CHECKING PLAYBACK END'
        a.alignment_active = True
        a._alignment_success(23)
        self.assertEqual(a.phase, HIHAT_CALIBRATION_WAIT_PHASE)
        with patch('camera_playback.app.hybrid_control.begin_search') as search:
            a.hybrid_enabled = True
            a._begin_strike_attempt()
        search.assert_not_called()
        a.fail.assert_called_once()

    def test_arm_start_not_blocked_by_unfinished_calibration(self):
        a = self.fixture()
        a.hardware = True
        a.hihat_fault_detail = None
        a.hihat = Mock()
        a.hihat.ready.return_value = True
        self.assertTrue(a._hihat_ready_for_start())

    def test_wait_resumes_only_after_success_with_a_new_alignment_check(self):
        a = self.fixture()
        a.phase = HIHAT_CALIBRATION_WAIT_PHASE
        a.bus = SimpleNamespace(active=True)
        a.single_run_mode = True
        a._sample_continuous_j7_minimum = Mock()
        a._advance_hardware_test_mit = Mock()
        a._maintain_hardware_test_fault_hold = Mock()
        a._maintain_hardware_test_workflow_fault = Mock()
        a.gripper_closed_latched = False
        a.continuous_strike_active = False
        a._begin_playback_alignment = Mock()
        with patch('camera_playback.app.hybrid_control.tick'):
            a.extra_control(10.)
            a._begin_playback_alignment.assert_not_called()
            a.hihat_calibration.ready = True
            a.extra_control(11.)
        a._begin_playback_alignment.assert_called_once_with(11.)

    def test_sync_and_calibration_receive_same_messages_without_polling_twice(self):
        a = App.__new__(App)
        a.hihat_calibration, a.hihat_sync = Mock(), Mock()
        messages = [dict(kind='hit', instrument='hihat', event_at=1.)]
        a._hihat_sound_messages(messages)
        a.hihat_calibration.feed.assert_called_once_with(messages)
        a.hihat_sync.feed.assert_called_once_with('hihat', messages)

    def test_close_waits_for_hihat_return_even_when_arm_is_disabled(self):
        a = App.__new__(App)
        a.bus = SimpleNamespace(active=False)
        a.phase = 'READY'
        a.hihat_calibration = SimpleNamespace(busy=True)
        a.center_relax, a.root = Mock(), Mock()
        a._safe_close_tick()
        a.root.quit.assert_not_called()
        a.root.after.assert_called_once()
        a.hihat_calibration.busy = False
        a._safe_close_tick()
        a.root.quit.assert_called_once()


class FirmwareTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'g++ required for native firmware parser test')
    def test_actual_firmware_parser_and_limits(self):
        source = Path(__file__).with_name('hihat_firmware_harness.cpp')
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / 'firmware_test'
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra',str(source),'-o',str(binary)],
                           check=True,capture_output=True,text=True)
            result = subprocess.run([str(binary)],check=True,capture_output=True,text=True)
            self.assertIn('stall latch passed', result.stdout)


if __name__ == '__main__': unittest.main()
