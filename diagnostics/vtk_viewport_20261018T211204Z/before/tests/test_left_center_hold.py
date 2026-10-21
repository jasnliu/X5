"""Two-arm startup/hold regression checks. No real CAN, motors, or cameras."""
import math
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.app import App
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET, LeftCenterMonitor
from camera_playback.playback_transport import PlaybackMotors
from camera_playback.simulation import SimulatedMotors
from camera_playback.smooth_recording_worker import RecordingMotors
from centering.motors import Motors, packet, parameter
from goal_motion.app import App as JointGoalApp
from safe_zone.encoder import FRAME, EFF, joint_to_motor


def fake_bus():
    bus = Motors.__new__(Motors)
    bus.control_side, bus.control_gripper = 'right', True
    bus.active = False
    bus.states = {(side, i): (.2 if i < 8 else 0., 0, time.monotonic())
                  for side in ('left', 'right') for i in range(1, 9)}
    bus.modes = {}
    bus.sockets = {s: Mock(send=Mock(return_value=16), recv=Mock(side_effect=BlockingIOError))
                   for s in ('left', 'right')}
    bus.locks = []
    bus.isolated_motors = set()
    bus.right_joint7_session = None
    bus.right_joint7_query_period = None
    bus.last_query = bus.last_right_joint7_query = 0.
    return PlaybackMotors.adopt(bus, np.zeros(7), left_hold=True)


def held_states(now):
    return {('left', i): (q, 2, now) for i, q in enumerate([*LEFT_CENTER, LEFT_GRIPPER_TARGET], 1)}


class LeftHoldTests(unittest.TestCase):
    def test_positive_raw_14_16_degree_target_is_left_only(self):
        bus = fake_bus()
        target_frame = parameter(8, 0x7016, -LEFT_GRIPPER_TARGET)
        self.assertEqual(target_frame, parameter(8, 0x7016, math.radians(14.16)))
        bus.left_drive.send_control(target_frame)
        bus.sockets['left'].sock.send.assert_called_once_with(target_frame)
        with self.assertRaisesRegex(RuntimeError, 'Invalid right-motor'):
            bus.send_right(target_frame)
        with self.assertRaisesRegex(RuntimeError, 'Invalid left-motor'):
            bus.left_drive.send_control(parameter(8, 0x7016, -.2))
        bus.states['left', 8] = (LEFT_GRIPPER_TARGET, 2, time.monotonic())
        self.assertAlmostEqual(bus.left_drive._center_motor_target(8, LEFT_GRIPPER_TARGET), math.radians(14.16))

    def test_zero_mm_no_longer_satisfies_left_center(self):
        monitor = LeftCenterMonitor(1.)
        for now in (1., 1.7):
            states = held_states(now)
            states['left', 8] = (0., 2, now)
            monitor.update(states, now)
        self.assertFalse(monitor.ready)

    def test_closed_feedback_uses_raw_positive_sign_and_holds(self):
        monitor = LeftCenterMonitor(1.)
        for now in (1., 1.7, 100.):
            states = held_states(now)
            # The bus stores -raw; positive raw 14.16 degrees must settle.
            states['left', 8] = (-math.radians(14.16), 2, now)
            monitor.update(states, now)
        self.assertTrue(monitor.ready)
        states = held_states(101.)
        states['left', 8] = (math.radians(14.16), 2, 101.)
        with self.assertRaisesRegex(RuntimeError, 'drifted'):
            monitor.update(states, 101.)

    def test_confirm_both_at_live_pose_then_dispatch_goals_in_one_tick(self):
        bus = fake_bus()
        sent = []
        tick = 0
        def send(side, frame):
            sent.append((tick, side, frame))
            cid, _, data = FRAME.unpack(frame)
            motor, kind = cid & 255, (cid >> 24) & 31
            if kind == 17:
                bus.mode_feedback(side, motor, 5, time.monotonic())
            if kind in (3, 4):
                q = bus.states[side, motor][0]
                bus.states[side, motor] = (q, 2 if kind == 3 else 0, time.monotonic())
            return 16
        for side, sock in bus.sockets.items():
            sock.sock.send.side_effect = lambda f, s=side: send(s, f)
        right = np.array([0.] * 6 + [1.4])
        setup = bus.center(right, math.radians(7))
        while True:
            tick += 1
            self.assertLess(tick, 300)
            for key, (q, mode, _) in bus.states.items():
                bus.states[key] = q, mode, time.monotonic()
            try:
                next(setup)
            except StopIteration:
                break
        final = sent[-14:]
        self.assertEqual(len({t for t, _, _ in final}), 1)
        self.assertEqual([s for _, s, _ in final], ['left'] * 7 + ['right'] * 7)
        for (_, side, frame), q, i in zip(final, [*LEFT_CENTER, *right], [*range(1, 8)] * 2):
            self.assertEqual(frame, parameter(i, 0x7016, joint_to_motor(side, i, q)))
        for _, side, frame in sent[:-14]:
            cid, _, data = FRAME.unpack(frame)
            i = cid & 255
            if (cid >> 24) & 31 == 18 and data[:2] == b'\x16\x70' and i <= 7:
                self.assertEqual(frame, parameter(i, 0x7016, joint_to_motor(side, i, .2)))
        self.assertIn(('left', parameter(8, 0x7016, -LEFT_GRIPPER_TARGET)), [(s, f) for _, s, f in sent])
        self.assertFalse(bus.left_center_ready())
        for sock in bus.sockets.values():
            sock.sock.send.reset_mock()
        bus.set_positions(np.ones(7) * .1)
        bus.set_right_joint7_position(.3)
        bus.sockets['left'].sock.send.assert_not_called()
        bus.relax()
        self.assertFalse(bus.active)
        self.assertEqual(bus.sockets['left'].sock.send.call_count, 24)
        self.assertTrue(all(c.args[0] in [packet(4, i) for i in range(1, 9)]
                            for c in bus.sockets['left'].sock.send.call_args_list))

    def test_start_requires_fresh_disabled_both_sides_before_any_writes(self):
        for stale in (False, True):
            bus = fake_bus()
            bus.states['left', 6] = (0., 0 if stale else 2, 0. if stale else time.monotonic())
            with self.assertRaisesRegex(RuntimeError, 'Both arms'):
                next(bus.center(np.zeros(7), 0.))
            for sock in bus.sockets.values():
                sock.sock.send.assert_not_called()

    def test_hold_settle_indefinite_and_drift_fault(self):
        monitor = LeftCenterMonitor(1.)
        monitor.update(held_states(1.), 1.)
        self.assertFalse(monitor.ready)
        monitor.update(held_states(1.7), 1.7)
        self.assertTrue(monitor.ready)
        monitor.update(held_states(1000.), 1000.)
        bad = held_states(1001.)
        bad['left', 5] = (0., 2, 1001.)
        with self.assertRaisesRegex(RuntimeError, 'drifted'):
            monitor.update(bad, 1001.)
        self.assertFalse(monitor.ready)
        monitor.update(bad, 1002.)

    def test_left_setup_failure_never_dispatches_either_center_target(self):
        bus = fake_bus()
        bus.left_drive._prepare_center_motor_confirmed = Mock(
            side_effect=RuntimeError('left enable not confirmed'))
        with self.assertRaisesRegex(RuntimeError, 'left enable not confirmed'):
            list(bus.center(np.zeros(7), 0.))
        bus.sockets['left'].sock.send.assert_not_called()
        for call in bus.sockets['right'].sock.send.call_args_list:
            _, _, data = FRAME.unpack(call.args[0])
            self.assertNotEqual(data[:2], b'\x16\x70')

    def test_left_and_right_mode_readbacks_are_separate(self):
        bus = fake_bus()
        bus.mode_feedback('left', 5, 5, 1.)
        bus.mode_feedback('right', 5, 0, 2.)
        self.assertEqual(bus.left_drive.modes[5], (5, 1.))
        self.assertEqual(bus.modes[5], (0, 2.))

    def test_stall_timeout_gripper_and_feedback_fail_closed(self):
        monitor = LeftCenterMonitor(1.)
        for now in (1., 2.):
            states = held_states(now)
            states['left', 5] = (0., 2, now)
            monitor.update(states, now)
        states['left', 5] = (0., 2, 3.45)
        states = {key: (q, mode, 3.45) for key, (q, mode, _) in states.items()}
        with self.assertRaisesRegex(RuntimeError, 'STALL: left motor 5'):
            monitor.update(states, 3.45)
        monitor = LeftCenterMonitor(1.)
        states = held_states(32.)
        states['left', 8] = (math.radians(2), 2, 32.)
        with self.assertRaisesRegex(RuntimeError, 'timeout'):
            monitor.update(states, 32.)
        with self.assertRaisesRegex(RuntimeError, 'fresh running'):
            LeftCenterMonitor(1.).update(held_states(1.), 2.)

    def test_default_other_programs_remain_left_query_only(self):
        bus = fake_bus()
        bus.left_hold_enabled = False
        self.assertFalse(bus.feedback_controlled('left', 1))
        with self.assertRaisesRegex(RuntimeError, 'query-only'):
            bus.send_left(packet(3, 1))

    def test_powered_left_feedback_accepted_by_gui_and_read_only_worker(self):
        for cls in (PlaybackMotors, RecordingMotors):
            bus = fake_bus()
            if cls is RecordingMotors:
                worker = cls.__new__(cls)
                worker.__dict__.update(bus.__dict__)
                bus = worker
            payload = (32768).to_bytes(2, 'big') + bytes(6)
            frame = FRAME.pack(EFF | 2 << 24 | 2 << 22 | 5 << 8, 8, payload)
            bus.sockets['left'].sock.recv.side_effect = [frame, BlockingIOError()]
            Motors.poll(bus)
            self.assertEqual(bus.states['left', 5][1], 2)
            if cls is RecordingMotors:
                with self.assertRaisesRegex(RuntimeError, 'only'):
                    bus._send('left', parameter(5, 0x7016, 0.))

    def test_partial_disable_failure_not_reported_inactive(self):
        bus = fake_bus()
        bus.active = True
        bus.left_drive.relax = Mock(side_effect=RuntimeError('left CAN down'))
        with self.assertRaisesRegex(RuntimeError, 'disable delivery failed'):
            bus.relax()
        self.assertTrue(bus.active)

    def test_gui_waits_for_both_disabled_feedback_not_just_right(self):
        app = App.__new__(App)
        app.bus = fake_bus()
        app.bus.poll = Mock()
        now = time.monotonic()
        app.relax_at, app.phase = now - .1, 'RELAXING'
        app.side = 'right'
        app.setup = app.control = None
        for name in ('status', 'safety', 'extra_control', 'publish', 'angles',
                     'encoder_text', 'root', 'node', 'start_button', 'fail'):
            setattr(app, name, Mock())
        app.bus.states['left', 5] = (LEFT_CENTER[4], 2, now)
        with patch('goal_motion.app.rclpy.spin_once'):
            JointGoalApp.tick(app)
            self.assertEqual(app.phase, 'RELAXING')
            app.bus.states['left', 5] = (LEFT_CENTER[4], 0, time.monotonic())
            JointGoalApp.tick(app)
        self.assertEqual(app.phase, 'RELAXED')
        app.fail.assert_not_called()
        app.status.set.assert_called_with('Left + right motors disabled')

    def test_simulation_and_app_gate_hold_left_through_right_motion(self):
        with patch('camera_playback.simulation.time.monotonic', return_value=1.) as clock:
            bus = SimulatedMotors(np.zeros(7))
            list(bus.center(np.zeros(7), 0.))
            app = App.__new__(App)
            app.bus, app.status = bus, Mock()
            app.centered(1.)
            self.assertIn('waiting for left', app.status.set.call_args.args[0])
            for now in np.arange(1.05, 4., .05):
                clock.return_value = now
                bus.poll()
            self.assertTrue(bus.left_center_ready())
            np.testing.assert_allclose(bus._left_joints, LEFT_CENTER)
            bus.set_positions(np.ones(7))
            for now in np.arange(4., 8., .05):
                clock.return_value = now
                bus.poll()
            np.testing.assert_allclose(bus._left_joints, LEFT_CENTER)
            self.assertAlmostEqual(math.degrees(bus.motor8_feedback['left']['raw_rad']), 14.16, delta=.022)
            bus.relax()
            self.assertTrue(all(state[1] == 0 for state in bus.states.values()))
            self.assertEqual(app.relaxed_feedback_sides(), ('left', 'right'))


if __name__ == '__main__':
    unittest.main()
