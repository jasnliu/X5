"""No hardware: center-first J7 ownership and slow return bounds."""
import math
import unittest
from unittest.mock import Mock
from camera_playback.mit_center_return import MitCenterReturn
from camera_playback import hybrid_workflow as hw
from test_hybrid_workflow import app_fixture


class MitCenterTests(unittest.TestCase):
    def test_reference_reaches_center_without_fast_jump(self):
        c = MitCenterReturn(.6, 1.4, .745, 10.)
        q, previous_v = .6, 0.
        for i in range(1, 251):
            command = c.update(q, 10.+i*.02)
            self.assertLessEqual(abs(command.velocity), .35+1e-9)
            self.assertLessEqual(abs(command.position-q), .35*.02+1e-9)
            self.assertLessEqual(command.position, 1.4)
            self.assertLessEqual(abs(command.torque+command.kp*(command.position-q)
                                     +command.kd*(command.velocity-previous_v)), 3.+1e-8)
            previous_v = (command.position-q)/.02
            q = command.position
        self.assertAlmostEqual(q, 1.4)

    def test_gui_delay_does_not_cause_catch_up_jump(self):
        c = MitCenterReturn(.6, 1.4, .7, 10.)
        command = c.update(.6, 11.)
        self.assertLess(command.position, .601)

    def test_native_velocity_keeps_real_pd_plus_feedforward_inside_torque_limit(self):
        # A quantized/batched position jump is NOT 5 rad/s at the drive.
        # Using that false derivative in the limiter used to add >6 Nm of
        # feedforward even though the drive reports stationary feedback.
        for q in (-.01, .01):
            c = MitCenterReturn(0., 0., 0., 10.)
            command = c.update(q, 10.002, measured_velocity=0.)
            actual_torque = (command.torque+command.kp*(command.position-q)
                             +command.kd*command.velocity)
            self.assertLessEqual(abs(actual_torque), 3.)
            self.assertLess(abs(command.torque), .01)

    def test_invalid_native_velocity_fails_closed(self):
        for v in (float('nan'), float('inf')):
            with self.assertRaisesRegex(RuntimeError, 'velocity'):
                MitCenterReturn(0., 0., 0., 10.).update(0., 10.002, measured_velocity=v)

    def test_optional_left_load_estimator_does_not_change_right_default(self):
        for gain in (10., 60.):
            kwargs = {} if gain == 60. else dict(bias_gain=gain)
            c = MitCenterReturn(0., 0., 0., 10., **kwargs)
            c.update(.01, 10.002, measured_velocity=0.)
            self.assertAlmostEqual(c.bias, -gain*.01*.002)
        for gain in (-1., 61., float('nan')):
            with self.assertRaises(ValueError):
                MitCenterReturn(0., 0., 0., 10., bias_gain=gain)

    def test_precision_left_path_does_not_increase_motion_or_torque_caps(self):
        c = MitCenterReturn(0., .4, 0., 10., bias_gain=40., kp=160., kd=4.)
        reference = MitCenterReturn(0., .4, 0., 10.)
        q = v = 0.
        for i in range(1, 1501):
            cmd = c.update(q, 10.+i*.002, measured_velocity=v)
            original = reference.update(q, 10.+i*.002)
            self.assertLessEqual(abs(cmd.velocity), .35+1e-9)
            # Preserve the complete existing reference, including its final
            # snap to zero velocity at the exact endpoint.
            self.assertEqual((cmd.position, cmd.velocity), (original.position, original.velocity))
            net = cmd.torque+cmd.kp*(cmd.position-q)+cmd.kd*(cmd.velocity-v)
            self.assertLessEqual(abs(net), 3.+1e-9)
            q, v = cmd.position, cmd.velocity
        self.assertAlmostEqual(q, .4)
        # An obstructed/still joint must not receive a higher net torque cap.
        c = MitCenterReturn(0., .4, 0., 10., bias_gain=40., kp=160., kd=4.)
        for i in range(1, 1001):
            cmd = c.update(0., 10.+i*.002, measured_velocity=0.)
            self.assertLessEqual(abs(cmd.torque+cmd.kp*cmd.position+cmd.kd*cmd.velocity), 3.+1e-9)
        right = MitCenterReturn(0., .4, 0., 10.)
        self.assertEqual((right.kp, right.kd, right.bias_gain), (40., 1.8, 60.))

    def test_normal_exit_joins_then_centers_without_mode_switch_or_disable(self):
        a = app_fixture()
        a.single_run_mode = True
        a.bus.right_joint7_mode_readback.return_value = 0
        a._cancel_continuous_striking = Mock()
        a.begin_stage = Mock()
        hw.close(a)
        hw.begin_center_return(a)
        a.bus.begin_mit_center_return.assert_called_once_with(.745)
        a.bus.restore_right_joint7_csp.assert_not_called()
        a.bus.relax.assert_not_called()
        a.begin_stage.assert_called_once_with('CENTER RELAX RECENTERING', a.center_goal)

    def test_unknown_mode_never_moves_or_disables(self):
        a = app_fixture()
        a.bus.right_joint7_mode_readback.return_value = None
        a.begin_stage = Mock()
        with self.assertRaisesRegex(RuntimeError, 'Waiting for J7 mode'):
            hw.begin_center_return(a)
        a.begin_stage.assert_not_called()
        a.bus.begin_mit_center_return.assert_not_called()
        a.bus.relax.assert_not_called()


if __name__ == '__main__':
    unittest.main()
