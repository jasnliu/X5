"""Single-button normal-hardware startup checks. All motor I/O is mocked."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.app import App, SINGLE_RUN_GRIPPER_TIMEOUT
from centering.motors import RIGHT_GRIPPER_CLOSED, RIGHT_GRIPPER_OPEN


def fixture():
    a = App.__new__(App)
    a.single_run_mode = True
    a.single_run_gripper_deadline = None
    a.hardware = True
    a.side = 'right'
    a.test_mode = a.hardware_test_mode = a.recording_only = False
    a.phase = 'READY'
    a.bus = Mock(active=False)
    a.bus.fresh.return_value = True
    a.receiver = Mock(cymbal=True)
    a.receiver.fresh.return_value = True
    a.audio_receiver = Mock()
    a.hihat = Mock()
    a.support_fault_detail = a.sound_fault_detail = a.hihat_fault_detail = None
    a._camera_ready_for_start = Mock(return_value=True)
    a._sound_ready_for_start = Mock(return_value=True)
    a._hihat_ready_for_start = Mock(return_value=True)
    a._clear_alignment_state = Mock()
    a._refresh_hardware_test_controls = Mock()
    a.root = Mock()
    a.status = Mock()
    a.result_status = Mock()
    a.result_label = Mock()
    a.start_button = Mock()
    a.continue_button = Mock()
    a.center_relax_button = Mock()
    a.setup = None
    a.zone = Mock()
    a.zone.contains.return_value = True
    a.planned_tcp = Mock(return_value=np.zeros(3))
    a.tcp = Mock(return_value=np.zeros(3))
    a.center_goal = np.zeros(7)
    a.ik = SimpleNamespace(origin_tcp=np.zeros(3))
    a.playback_trajectory = SimpleNamespace(first_joints=np.ones(7)*.1, tcp_positions=[np.zeros(3)])
    a.gripper_encoder = Mock(return_value=RIGHT_GRIPPER_CLOSED)
    a.begin_stage = Mock(side_effect=lambda phase, q: setattr(a, 'phase', phase))
    a._program_failure = Mock()
    return a


class SingleRunTests(unittest.TestCase):
    def test_run_is_enabled_only_with_all_readiness_and_recording(self):
        for phase in ('READY', 'RELAXED'):
            a = fixture()
            a.phase = phase
            a._refresh_buttons()
            a.start_button.config.assert_called_with(state='disabled')
            a.continue_button.config.assert_called_with(state='normal')
        for gate in ('_camera_ready_for_start', '_sound_ready_for_start', '_hihat_ready_for_start'):
            a = fixture()
            getattr(a, gate).return_value = False
            a._refresh_buttons()
            a.continue_button.config.assert_called_with(state='disabled')
        a = fixture()
        a.playback_trajectory = None
        a._refresh_buttons()
        a.continue_button.config.assert_called_with(state='disabled')
        a = fixture()
        a.bus.fresh.return_value = False
        a._refresh_buttons()
        a.continue_button.config.assert_called_with(state='disabled')

    def test_one_press_latches_closed_gripper_and_preflights_without_duplicate_start(self):
        a = fixture()
        a.start()
        self.assertEqual(a.phase, 'PLAYBACK PREFLIGHTED')
        self.assertTrue(a.gripper_closed_latched)
        a.continue_button.config.assert_called_with(state='disabled')
        a.root.after.assert_called_once_with(20, a._start_preflighted_recording)
        a.start()
        self.assertEqual(a.root.after.call_count, 1)
        a.bus.center.assert_not_called()  # Preflight still precedes actuation.

    def test_preflight_centers_with_closed_gripper_not_open(self):
        a = fixture()
        a.start()
        a._start_preflighted_recording()
        self.assertEqual(a.phase, 'SETTING UP')
        a.bus.center.assert_called_once_with(a.center_goal, RIGHT_GRIPPER_CLOSED)
        self.assertIn('recording follows automatically', a.status.set.call_args.args[0])

    def test_stale_preflight_never_enables_motors(self):
        a = fixture()
        a.start()
        a._hihat_ready_for_start.return_value = False
        a._start_preflighted_recording()
        a.bus.center.assert_not_called()

    def test_center_completion_automatically_moves_to_recording_after_closure(self):
        a = fixture()
        a.phase = 'CENTERING'
        a.gripper_encoder.return_value = RIGHT_GRIPPER_OPEN
        a.centered(10.)
        self.assertEqual(a.phase, 'CENTERING')
        a.begin_stage.assert_not_called()
        a.gripper_encoder.return_value = RIGHT_GRIPPER_CLOSED
        a.centered(10.1)
        a.begin_stage.assert_called_once_with('MOVING TO RECORDING START', a.playback_trajectory.first_joints)
        a.centered(10.2)
        self.assertEqual(a.begin_stage.call_count, 1)
        self.assertIsNone(a.single_run_gripper_deadline)

    def test_gripper_timeout_stops_instead_of_playing_or_waiting_for_loading(self):
        a = fixture()
        a.phase = 'CENTERING'
        a.gripper_encoder.return_value = RIGHT_GRIPPER_OPEN
        a.centered(10.)
        a.centered(10.+SINGLE_RUN_GRIPPER_TIMEOUT)
        a._program_failure.assert_called_once()
        self.assertIn('recording not started', a._program_failure.call_args.args[0])
        a.begin_stage.assert_not_called()

    def test_stale_encoder_or_support_loss_never_starts_recording(self):
        a = fixture()
        a.phase = 'CENTERING'
        a.bus.fresh.return_value = False
        a.centered(10.)
        a.begin_stage.assert_not_called()
        a.bus.fresh.return_value = True
        a._sound_ready_for_start.return_value = False
        a.centered(10.1)
        a._program_failure.assert_called_once()
        a.begin_stage.assert_not_called()

    def test_cancelled_center_does_not_auto_advance(self):
        a = fixture()
        a.phase = 'CENTER RELAX RECENTERING'
        a.centered(10.)
        a.begin_stage.assert_not_called()

    def test_gripper_keepalive_never_opens_and_leaves_setup_to_generator(self):
        a = fixture()
        a.bus.active = True
        a.phase = 'SETTING UP'
        a.setup = object()
        a.gripper_closed_latched = True
        a.last_gripper_command = 0.
        a.alignment_active = False
        for name in ('_sample_continuous_j7_minimum', '_advance_hardware_test_mit',
                     '_maintain_hardware_test_fault_hold', '_maintain_hardware_test_workflow_fault'):
            setattr(a, name, Mock())
        with patch('camera_playback.app.hybrid_control.tick'):
            a.extra_control(10.)
            a.bus.set_gripper.assert_not_called()
            a.phase = 'CENTERING'
            a.setup = None
            a.extra_control(10.1)
        a.bus.set_gripper.assert_called_once_with(RIGHT_GRIPPER_CLOSED)

    def test_other_modes_keep_open_gripper_loading(self):
        for hardware_test in (True, False):
            a = fixture()
            a.single_run_mode = False
            a.hardware_test_mode = hardware_test
            a.side = 'right'
            self.assertEqual(a.initial_gripper_goal(), RIGHT_GRIPPER_OPEN)
            a.phase = 'CENTERING'
            a.gripper_encoder.return_value = RIGHT_GRIPPER_OPEN
            a.centered(10.)
            self.assertEqual(a.phase, 'WAITING FOR LOAD')
            a.begin_stage.assert_not_called()


if __name__ == '__main__':
    unittest.main()
