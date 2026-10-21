"""Offline tests for the isolated right-J7 ten-degree program."""
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from centering.motors import (
    RIGHT_GRIPPER_CLOSED,
    RIGHT_STRIKE_SPEED,
    SPEED,
)
from right_joint7_test.app import (
    App,
    DOWN_PHASE,
    GRIPPER_CLOSED_HOLD_SECONDS,
    RETURN_PHASE,
)
from right_joint7_test.plan import (
    TEST_J7_DELTA_RAD,
    Joint7TestPlan,
    build_joint7_test_plan,
)
from safe_zone.geometry import Model, RIGHT_TCP, Zone


ROOT = Path(__file__).resolve().parents[1]


class Value:
    def __init__(self):
        self.value = ""

    def set(self, value):
        self.value = value

    def get(self):
        return self.value


class RightJoint7TestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = Model(ROOT / "model/openarmx.urdf")
        cls.zone = Zone.load(
            ROOT / "right_zones/zone1.json", cls.model.digest, RIGHT_TCP
        )
        joints = {joint.get("name"): joint for joint in cls.model.joints}
        limits = [
            joints[f"openarmx_right_joint{i}"].find("limit")
            for i in range(1, 8)
        ]
        cls.lower = np.array([float(limit.get("lower")) for limit in limits])
        cls.upper = np.array([float(limit.get("upper")) for limit in limits])
        cls.center = np.zeros(7)
        cls.center[6] = cls.upper[6]

    def test_real_zone_plan_is_exactly_j7_minus_ten_and_back(self):
        plan = build_joint7_test_plan(
            self.model, self.zone, self.lower, self.upper, self.center
        )
        np.testing.assert_array_equal(plan.center_joints, self.center)
        np.testing.assert_array_equal(plan.lowered_joints[:6], self.center[:6])
        self.assertAlmostEqual(
            plan.center_joints[6] - plan.lowered_joints[6],
            TEST_J7_DELTA_RAD,
        )

    def test_plan_rejects_an_outside_sample(self):
        class RejectAllZone:
            def contains(self, _point, _buffer):
                return False

        with self.assertRaisesRegex(ValueError, "outbound path"):
            build_joint7_test_plan(
                self.model, RejectAllZone(), self.lower, self.upper, self.center
            )

    def test_continue_closes_gripper_but_waits_for_confirmed_encoder_hold(self):
        app = App.__new__(App)
        app.hardware = True
        app.phase = "WAITING FOR LOAD"
        app.bus = Mock(active=True)
        app.bus.fresh.return_value = True
        app.zone = SimpleNamespace(contains=Mock(return_value=True))
        app.tcp = Mock(return_value=np.zeros(3))
        app.status = Value()
        app.continue_button = Mock()
        app.control = object()
        app.last_gripper_command = 0.0
        app.strike_speed_fast = False
        app._begin_joint7_test = Mock()

        with patch("right_joint7_test.app.time.monotonic", return_value=10.0):
            app.continue_motion()
        app.bus.set_gripper.assert_called_once_with(RIGHT_GRIPPER_CLOSED)
        self.assertEqual(app.phase, "CLOSING GRIPPER")
        self.assertIsNone(app.control)

        app.gripper_encoder = Mock(return_value=RIGHT_GRIPPER_CLOSED)
        app.extra_control(10.0)
        app._begin_joint7_test.assert_not_called()
        app.extra_control(10.0 + GRIPPER_CLOSED_HOLD_SECONDS - .01)
        app._begin_joint7_test.assert_not_called()
        app.extra_control(10.0 + GRIPPER_CLOSED_HOLD_SECONDS + .01)
        app._begin_joint7_test.assert_called_once_with()

    def test_fast_speed_is_selected_before_downward_target(self):
        app = App.__new__(App)
        app.phase = "CLOSING GRIPPER"
        app.test_plan = Joint7TestPlan(np.zeros(7), np.ones(7))
        app.bus = Mock()
        app.strike_speed_fast = False
        events = []
        app.bus.set_right_joint7_speed.side_effect = lambda value: events.append(
            ("speed", value)
        )
        app._begin_strike_stage = Mock(
            side_effect=lambda name, target: events.append(("target", name))
        )
        app.fail = Mock()
        app._begin_joint7_test()
        self.assertEqual(
            events,
            [("speed", RIGHT_STRIKE_SPEED), ("target", DOWN_PHASE)],
        )
        self.assertTrue(app.strike_speed_fast)
        app.fail.assert_not_called()

    def test_gripper_close_timeout_aborts_without_starting_j7(self):
        app = App.__new__(App)
        app.hardware = True
        app.phase = "CLOSING GRIPPER"
        app.bus = Mock(active=True)
        app.bus.fresh.return_value = True
        app.continue_button = Mock()
        app.gripper_closed_latched = True
        app.last_gripper_command = 10.0
        app.gripper_close_deadline = 11.0
        app.gripper_in_tolerance_since = None
        app.gripper_encoder = Mock(return_value=0.0)
        app.status = Value()
        app.fail = Mock()
        app._begin_joint7_test = Mock()
        app.extra_control(11.01)
        app.fail.assert_called_once()
        self.assertIn("fully close", app.fail.call_args.args[0])
        app._begin_joint7_test.assert_not_called()

    def test_down_completion_immediately_commands_custom_center_return(self):
        center = np.arange(7, dtype=float) / 10.0
        app = App.__new__(App)
        app.phase = DOWN_PHASE
        app.test_plan = Joint7TestPlan(center, center.copy())
        app._begin_strike_stage = Mock()
        app.complete_stage(2.0)
        app._begin_strike_stage.assert_called_once()
        self.assertEqual(app._begin_strike_stage.call_args.args[0], RETURN_PHASE)
        np.testing.assert_array_equal(
            app._begin_strike_stage.call_args.args[1], center
        )

    def test_return_restores_normal_speed_before_disabling(self):
        app = App.__new__(App)
        app.phase = RETURN_PHASE
        app.control = object()
        events = []
        app._restore_strike_speed = Mock(
            side_effect=lambda: events.append("normal-speed") or True
        )
        app.relax = Mock(side_effect=lambda _message: events.append("relax"))
        app.complete_stage(3.0)
        self.assertEqual(events, ["normal-speed", "relax"])
        self.assertIsNone(app.control)

    def test_failure_restores_normal_speed_then_uses_base_failure(self):
        app = App.__new__(App)
        app.bus = Mock(active=True)
        app.strike_speed_fast = True
        app.setup = None
        app.control = None
        app.hold_until = None
        app.phase = DOWN_PHASE
        app.side = "right"
        app.relax_at = None
        app.status = Value()
        app.continue_button = Mock()
        app.fail("test fault")
        app.bus.set_right_joint7_speed.assert_called_once_with(SPEED)
        app.bus.relax.assert_called_once_with()
        self.assertFalse(app.strike_speed_fast)
        self.assertEqual(app.phase, "FAULT")

    def test_launcher_is_isolated_and_uses_separate_ros_domain(self):
        launcher = (ROOT / "launch_right_joint7_test.py").read_text()
        script = (ROOT / "start_right_joint7_test.sh").read_text()
        self.assertIn('"-m", "right_joint7_test.app"', launcher)
        self.assertNotIn("camera_playback", launcher)
        self.assertNotIn("camera_search", launcher)
        self.assertIn("ROS_DOMAIN_ID=93", script)
        self.assertIn("launch_right_joint7_test.py", script)


if __name__ == "__main__":
    unittest.main()
