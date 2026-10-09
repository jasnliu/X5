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
