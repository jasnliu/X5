"""Compare restored right return commands with the pre-dual control loop.

Only mocked CAN sockets are used. These tests do not establish physical clearance.
"""
import json
import math
import socket
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.dual_recording import DualRecording, left_geometry
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET
from camera_playback.mit_center_return import MitCenterReturn
from camera_playback.app import App
from camera_playback import hybrid_workflow
from camera_search.app import App as CameraApp
from camera_search.control import CameraGoalControl, ENCODER_LSB_RAD
from centering.motors import RIGHT_GRIPPER_CLOSED
from goal_motion.app import App as JointApp
from safe_zone.encoder import FRAME
from smooth_playback.trajectory import Geometry, ROOT
from tests.test_left_center_hold import fake_bus


def forbid_devices(event, args):
    if event == 'socket.__new__' and args[1] == socket.PF_CAN:
        raise AssertionError('Physical CAN forbidden')
    if event == 'open' and isinstance(args[0], str) and args[0].startswith(('/dev/tty', '/dev/video', '/dev/snd')):
        raise AssertionError('Physical devices forbidden')


sys.addaudithook(forbid_devices)


class RightCenterRestoreTests(unittest.TestCase):
    def workflow(self, geometry, mit):
        bus = fake_bus()
        bus.active = bus.left_drive.active = True
        bus.center_goal = geometry.center.copy()
        bus.poll = Mock()  # Encoder samples below are supplied, never queried.
        bus.disable_centered_side = Mock(side_effect=RuntimeError('Center not settled yet'))
        commands = []
        bus.sockets['right'].sock.send.side_effect = lambda frame: commands.append((time.monotonic(), frame)) or 16
        if mit:
            bus.mit_center_return = MitCenterReturn(.2, geometry.center[6], .3, time.monotonic())
        app = SimpleNamespace(bus=bus, test_mode=True, lower=geometry.lower, upper=geometry.upper,
            center_goal=geometry.center, playback_trajectory=object(),
            ik=Mock(path_between_inside=Mock(return_value=True)),
            setup=None, control=None, hold_until=None, relax_at=None,
            phase='READY', status=Mock(), _stop_smooth_playback=Mock(return_value=True),
            safety=Mock(), complete_stage=Mock(), extra_control=Mock(), start_button=Mock(),
            publish=Mock(), angles=Mock(), encoder_text=Mock(return_value='mock'),
            fail=Mock(side_effect=AssertionError), node=Mock(), root=Mock(), relax=Mock(), tick=Mock())
        app.arm = lambda: np.array([bus.states['right', i][0] for i in range(1, 8)])
        dual = DualRecording.__new__(DualRecording)
        dual.app, dual.g = app, left_geometry(geometry.model)
        dual.session = dual.cleanup_error = None
        dual.returning = False
        app.extra_control = dual.tick
        return app, dual, commands

    def feedback(self, app, right, now):
        for side, values in [('right', [*right, RIGHT_GRIPPER_CLOSED]),
                             ('left', [*LEFT_CENTER, LEFT_GRIPPER_TARGET])]:
            for motor, q in enumerate(values, 1):
                app.bus.states[side, motor] = (q, 2, now)

    def test_exact_old_target_frames_and_cadence_in_csp_and_mit(self):
        g = Geometry()
        start = np.array(json.loads((ROOT/'recordings/record3.json').read_text())['samples'][-1]['positions_rad'])
        for mit in (False, True):
            with self.subTest(mit=mit), patch('time.monotonic', return_value=100.) as clock, \
                    patch('goal_motion.app.rclpy.spin_once'):
                old, _, expected = self.workflow(g, mit)
                new, dual, actual = self.workflow(g, mit)
                self.feedback(old, start, clock.return_value)
                self.feedback(new, start, clock.return_value)
                # This is the exact inherited begin_stage/tick used before the
                # dual coordinator replaced the right return with a fixed goal.
                CameraApp.begin_stage(old, 'CENTER RELAX RECENTERING', g.center)
                old.extra_control = Mock()
                dual.begin_return()
                self.assertIs(type(new.control), CameraGoalControl)
                for index in range(301):
                    clock.return_value = 100. + index*.02
                    q = g.center + (start-g.center)*max(0., 1.-index/175.)
                    self.feedback(old, q, clock.return_value)
                    self.feedback(new, q, clock.return_value)
                    JointApp.tick(old)
                    JointApp.tick(new)
                self.assertEqual(actual, expected)  # Timestamp AND CAN bytes.
                self.assertGreater(len(actual), 100)
                motors = {FRAME.unpack(frame)[0] & 255 for _, frame in actual}
                self.assertEqual(motors, set(range(1, 7 if mit else 8)))
                # No right-arm correction may become a left-arm position write.
                left_frames = new.bus.sockets['left'].sock.send.call_args_list
                self.assertEqual(len(left_frames), 7)  # Original single center dispatch only.
                old.fail.assert_not_called()

    def test_right_keeps_original_one_encoder_count_reached_hysteresis(self):
        g = Geometry()
        with patch('time.monotonic', return_value=100.):
            app, dual, _ = self.workflow(g, False)
            self.feedback(app, g.center, 100.)
            dual.begin_return()
        q = g.center.copy()
        q[0] += math.radians(3.) + ENCODER_LSB_RAD*.5
        controller = app.control
        self.assertFalse(controller.update(q, 100.)[2])
        self.assertTrue(controller.update(q, 100.7)[2])

    def test_blocking_preflight_finishes_before_mit_clock_and_joint_dispatch(self):
        g = Geometry()
        with patch('time.monotonic', return_value=100.) as clock:
            app, dual, commands = self.workflow(g, False)
            self.feedback(app, g.center, 100.)
            app.dual = dual
            app.RETURN_PHASES = App.RETURN_PHASES
            app.hybrid_tuning = ({'load_torque': .745},)
            app._cancel_continuous_striking = Mock()
            app.begin_stage = lambda name, desired: App.begin_stage(app, name, desired)
            app.bus.right_joint7_mode_readback = Mock(return_value=0)
            app.bus.require_center_before_relax = True
            app.test_mode = False
            events = []

            def refresh(bus, seconds=.08):
                events.append('preflight refresh')
                clock.return_value += 1.  # Simulate an expensive, real delay.
                self.feedback(app, g.center, clock.return_value)

            original = app.bus.begin_mit_center_return
            def begin_mit(bias):
                events.append('MIT clock starts')
                original(bias)

            with patch('camera_playback.dual_recording.refresh_feedback', side_effect=refresh), \
                    patch.object(app.bus, 'begin_mit_center_return', side_effect=begin_mit):
                hybrid_workflow.begin_center_return(app)
            self.assertEqual(events[-1], 'MIT clock starts')
            self.assertEqual(len(events), 4)
            self.assertEqual(commands[0][0], app.bus.mit_center_return.at)
            self.assertIs(type(app.control), CameraGoalControl)
            self.assertNotIn('right', dual.return_controls)

    def test_dual_monitor_never_sends_right_updates_and_keeps_old_extra_control(self):
        g = Geometry()
        with patch('time.monotonic', return_value=100.):
            app, dual, commands = self.workflow(g, False)
            self.feedback(app, g.center, 100.)
            dual.begin_return()
            commands.clear()
            self.assertFalse(dual.tick(100.))
            self.assertEqual(commands, [])
            self.assertIs(type(app.control), CameraGoalControl)

    def test_no_more_right_targets_after_its_independent_relax(self):
        g = Geometry()
        with patch('time.monotonic', return_value=100.):
            app, dual, commands = self.workflow(g, False)
            self.feedback(app, g.center, 100.)
            dual.begin_return()
            app.bus.center_disabled['right'] = 99.
            for motor in range(1, 9):
                app.bus.states['right', motor] = (app.bus.states['right', motor][0], 0, 100.)
            commands.clear()
            dual.tick_return(100.)
            self.assertEqual(commands, [])
            self.assertTrue(app.bus.active)
            app.relax.assert_not_called()


if __name__ == '__main__':
    unittest.main()
