"""No hardware: regress queue collisions and no-strike center-before-disable."""
from collections import deque
import errno
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.app import App
from camera_playback.playback_transport import PlaybackMotors, RetrySocket
from centering.motors import Motors, packet, parameter, motion_control_packet
from camera_playback.recording_only import refresh_feedback


class TransportTests(unittest.TestCase):
    def test_retries_enobufs_and_eagain_without_dropping_packet(self):
        raw = Mock()
        raw.send.side_effect = [OSError(errno.ENOBUFS, 'full'), OSError(errno.EAGAIN, 'busy'), 16]
        sock = RetrySocket(raw)
        frame = packet(17, 1)
        self.assertEqual(sock.send(frame), 16)
        self.assertEqual(sock.retries, 2)
        self.assertEqual([c.args[0] for c in raw.send.call_args_list], [frame]*3)

    def test_persistent_pressure_is_bounded_and_not_silenced(self):
        raw = Mock()
        raw.send.side_effect = OSError(errno.ENOBUFS, 'full')
        start = time.monotonic()
        with self.assertRaises(OSError): RetrySocket(raw).send(packet(17, 1))
        self.assertLess(time.monotonic()-start, .2)

    def test_fatal_errors_and_short_writes_are_not_retried(self):
        raw = Mock()
        raw.send.side_effect = OSError(errno.ENETDOWN, 'down')
        with self.assertRaises(OSError): RetrySocket(raw).send(packet(17, 1))
        raw.send.assert_called_once()
        with self.assertRaises(RuntimeError): RetrySocket(Mock(send=Mock(return_value=8))).send(packet(17, 1))

    def bus(self):
        now = time.monotonic()
        bus = Motors.__new__(Motors)
        bus.sockets = dict(right=Mock(send=Mock(return_value=16)), left=Mock(send=Mock(return_value=16)))
        bus.locks = []
        bus.states = {(side, i): (0., 2 if side == 'right' else 0, now)
                      for side in ('right', 'left') for i in range(1, 9)}
        bus.control_side, bus.control_gripper, bus.active = 'right', True, True
        bus.isolated_motors = set()
        bus.right_joint7_session = None
        bus.right_joint7_query_period = None
        bus.last_right_joint7_query = bus.last_query = 0.
        return PlaybackMotors.adopt(bus, np.zeros(7), strict_center_relax=True)

    def settled(self, bus, offset=0):
        now = time.monotonic()
        bus.center_history = deque((now-d, np.full(7, offset)) for d in (.64, .60, .3, 0))
        for i in range(1, 8): bus.states['right', i] = (offset, 2, now)

    def test_duplicate_gui_queries_are_suppressed_only_after_worker_is_ready(self):
        bus = self.bus()
        bus.strict_center_relax = False
        bus.playback_session = SimpleNamespace(owns_feedback=False)
        with patch.object(Motors, 'poll'):
            bus.poll()
        self.assertEqual(bus.last_query, 0.)
        bus.playback_session.owns_feedback = True
        # Verify query ownership, not host scheduling/GC within a 10 ms window.
        # An exact clock value is stricter and deterministic under system load.
        with patch.object(Motors, 'poll'), \
                patch('camera_playback.playback_transport.time.monotonic', return_value=123.456):
            bus.poll()
        self.assertEqual(bus.last_query, 123.456)

    def test_no_disable_before_center_or_during_recording(self):
        bus = self.bus()
        self.settled(bus, .5)
        with self.assertRaises(RuntimeError): bus.send_right(packet(4, 1))
        with self.assertRaises(RuntimeError): bus.relax()
        bus.sockets['right'].sock.send.assert_not_called()
        self.settled(bus)
        bus.playback_session = object()
        with self.assertRaises(RuntimeError): bus.relax()
        with self.assertRaises(RuntimeError): bus.send_right(packet(4, 1))

    def test_stale_or_unsettled_center_never_disables(self):
        bus = self.bus()
        self.settled(bus)
        bus.states['right', 1] = (0., 2, time.monotonic()-1.)
        with self.assertRaises(RuntimeError): bus.relax()
        bus.states['right', 1] = (0., 2, time.monotonic())
        bus.center_history.popleft()
        bus.center_history.popleft()
        with self.assertRaises(RuntimeError): bus.relax()

    def test_only_verified_center_can_disable(self):
        bus = self.bus()
        self.settled(bus)
        bus.relax()
        self.assertFalse(bus.active)
        self.assertEqual(bus.sockets['right'].sock.send.call_count, 24)
        self.assertEqual(bus.last_center_evidence['max_error_deg'], 0.)

    def test_normal_hybrid_can_use_mit_but_cannot_relax_away_from_center(self):
        bus = self.bus()
        bus.strict_center_relax = False
        bus.require_center_before_relax = True
        self.settled(bus, .5)
        bus.send_right(motion_control_packet(7, .5, 0., 40., 1.8, 0.))
        with self.assertRaises(RuntimeError): bus.relax()
        with self.assertRaises(RuntimeError): bus.send_right(packet(4, 7))
        self.assertTrue(bus.active)
        self.settled(bus)
        bus.relax()
        self.assertFalse(bus.active)

    def test_major_stall_exception_can_disable_away_from_center(self):
        bus = self.bus()
        bus.strict_center_relax = False
        bus.require_center_before_relax = True
        self.settled(bus, .5)
        bus.emergency_relax_reason = 'STALL: motor 4'
        bus.relax()
        self.assertFalse(bus.active)

    def test_normal_center_return_keeps_mit_mode_and_never_disables_off_center(self):
        bus = self.bus()
        bus.strict_center_relax = False
        bus.require_center_before_relax = True
        bus.modes = {7: (0, time.monotonic())}
        self.settled(bus, .5)
        bus.begin_mit_center_return(.745)
        bus.set_positions(np.zeros(7))
        frames = [call.args[0] for call in bus.sockets['right'].sock.send.call_args_list]
        from safe_zone.encoder import FRAME
        self.assertEqual(len(frames), 6)
        self.assertEqual([FRAME.unpack(f)[0] & 255 for f in frames], list(range(1, 7)))
        with patch.object(Motors, 'poll'):
            bus.poll()
        last = bus.sockets['right'].sock.send.call_args.args[0]
        self.assertEqual((FRAME.unpack(last)[0] >> 24) & 31, 1)  # Powered MIT, not disable.
        with self.assertRaises(RuntimeError): bus.send_right(packet(4, 7))
        self.settled(bus)
        bus.relax()
        self.assertIsNone(bus.mit_center_return)

    def test_initial_disable_of_already_relaxed_drive_is_not_a_relaxation(self):
        bus = self.bus()
        bus.states['right', 1] = (.5, 0, time.monotonic())
        bus.send_right(packet(4, 1))
        bus.sockets['right'].sock.send.assert_called_once()

    def test_recording_only_forbids_mit_and_strike_entrypoints(self):
        bus = self.bus()
        for frame in (motion_control_packet(7, 0., 0., 0., 0., 0.), parameter(7, 0x7005, 0, True)):
            with self.assertRaises(RuntimeError): bus.send_right(frame)
        app = App.__new__(App)
        app.recording_only = True
        with self.assertRaises(RuntimeError): app._prepare_hardware_test_manual_strikes(1, np.zeros(7))
        with self.assertRaises(RuntimeError): app._begin_strike_stage('STRIKE MOVING OUT', np.zeros(7))
        with self.assertRaises(RuntimeError): app._begin_continuous_striking(5, np.zeros(7))

    def test_recording_only_alignment_and_relax_route_through_verified_return(self):
        app = App.__new__(App)
        app.recording_only = True
        app.bus = Mock(active=True)
        with patch('camera_playback.app.recording_only_control.finish') as finish:
            app._begin_playback_alignment(0.)
            self.assertTrue(finish.call_args.kwargs['playback_success'])
            app.relax()
            app.fail('temporary fault')
            app._program_failure('temporary fault')
        app.bus.relax.assert_not_called()
        self.assertEqual(finish.call_count, 4)

    def test_recording_only_does_not_require_a_cymbal_for_preflight(self):
        app = App.__new__(App)
        app.recording_only = True
        app.test_mode = False
        app.support_fault_detail = None
        app.receiver = Mock(cymbal=False, fresh=Mock(return_value=True))
        self.assertTrue(app._camera_ready_for_start())
        self.assertTrue(app._sound_ready_for_start())
        self.assertTrue(app._hihat_ready_for_start())
        self.assertFalse(app._sound_required_now())
        self.assertFalse(app._hihat_required_now())

    def test_geometry_preflight_refresh_never_disables_and_restores_active_check(self):
        bus = Mock(active=True, fresh=Mock(return_value=True))
        checks = []
        bus.poll.side_effect = lambda: checks.append(bus.active)
        refresh_feedback(bus, .004)
        self.assertTrue(bus.active)
        self.assertTrue(checks)
        self.assertFalse(any(checks))
        bus.relax.assert_not_called()
        bus.set_positions.assert_not_called()
        bus.fresh.return_value = False
        with self.assertRaises(RuntimeError): refresh_feedback(bus, .004)
        self.assertTrue(bus.active)
        bus.poll.side_effect = RuntimeError('CAN fault')
        with self.assertRaises(RuntimeError): refresh_feedback(bus, .004)
        self.assertTrue(bus.active)

    def test_recording_only_closes_gripper_then_approaches_without_a_cymbal(self):
        app = App.__new__(App)
        app.recording_only, app.test_mode = True, False
        app.bus = Mock(active=True, fresh=Mock(return_value=True))
        app.phase = 'CLOSING GRIPPER'
        app.continue_button = Mock()
        app._camera_ready_for_start = app._sound_ready_for_start = app._hihat_ready_for_start = Mock(return_value=True)
        for name in ('_sample_continuous_j7_minimum', '_advance_hardware_test_mit',
                     '_maintain_hardware_test_fault_hold', '_maintain_hardware_test_workflow_fault'):
            setattr(app, name, Mock())
        app.gripper_closed_latched = True
        app.last_gripper_command = 0.
        app.gripper_close_until = 0.
        app.status = Mock()
        app.receiver = Mock(cymbal=False, fresh=Mock(return_value=True))
        app.playback_trajectory = SimpleNamespace(first_joints=np.zeros(7))
        app.begin_stage = Mock()
        app.alignment_active = app.continuous_strike_active = False
        app.extra_control(1.)
        self.assertEqual(app.begin_stage.call_args.args[0], 'MOVING TO RECORDING START')


if __name__ == '__main__': unittest.main()
