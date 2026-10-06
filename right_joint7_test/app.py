"""Right-only custom-center, gripper-close, and fast J7 round-trip test."""
from __future__ import annotations

import argparse
import math
import time
import tkinter as tk

import numpy as np
import rclpy

from camera_playback.strike import StrikeControl
from cartesian_goal.app import GRIPPER_COMMAND_INTERVAL, GRIPPER_TOLERANCE
from centering.motors import (
    RIGHT_GRIPPER_CLOSED,
    RIGHT_GRIPPER_OPEN,
    RIGHT_STRIKE_SPEED,
    SPEED,
)
from goal_motion.app import App as JointGoalApp
from safe_zone.geometry import MEMBERSHIP_BUFFER_M

from .plan import Joint7TestPlan, build_joint7_test_plan


GRIPPER_CLOSED_HOLD_SECONDS = 0.20
GRIPPER_CLOSE_TIMEOUT_SECONDS = 3.0
DOWN_PHASE = "MOVING J7 DOWN 10 DEG"
RETURN_PHASE = "RETURNING J7 TO CUSTOM CENTER"


class App(JointGoalApp):
    """A deliberately small right-arm test with no camera or recording."""

    def __init__(self, hardware: bool):
        super().__init__(
            hardware,
            "right",
            center_joint7_at_max=True,
            control_gripper=True,
        )
        self.test_plan: Joint7TestPlan | None = None
        self.gripper_closed_latched = False
        self.gripper_close_deadline = None
        self.gripper_in_tolerance_since = None
        self.last_gripper_command = 0.0
        self.strike_speed_fast = False

        self.root.title(
            "RIGHT arm: Joint 7 ten-degree test"
            + ("" if hardware else " — OFFLINE PREVIEW")
        )
        self.header.config(
            text=("RIGHT arm: custom center → close gripper → "
                  "fast J7 −10° → custom center")
        )
        self.isolation_label.config(
            text=("Right arm only • custom center J1–J6=0°, "
                  f"J7={math.degrees(self.center_goal[6]):.1f}° • "
                  "right gripper −3° open / +7° closed\n"
                  f"Only right J7 uses {RIGHT_STRIKE_SPEED:.1f} rad/s; "
                  "the left arm and left gripper remain relaxed/query-only.")
        )
        self.start_button.config(
            text="START: CENTER RIGHT ARM + OPEN GRIPPER TO −3°"
        )
        self.continue_button = tk.Button(
            self.root,
            text="CONTINUE: CLOSE GRIPPER → FAST J7 −10° → RETURN",
            font=("Sans", 14, "bold"),
            state="disabled",
            command=self.continue_motion,
        )
        self.continue_button.pack(
            fill="x", padx=20, pady=5, after=self.start_button
        )
        self.status.set(
            "Waiting for live encoders" if hardware
            else "Offline preview only — no CAN and no arm movement"
        )

        try:
            self.test_plan = build_joint7_test_plan(
                self.model, self.zone, self.lower, self.upper, self.center_goal
            )
        except ValueError as exc:
            self.status.set("TEST DISABLED: safety preflight rejected: " + str(exc))

    def initial_gripper_goal(self):
        return RIGHT_GRIPPER_OPEN

    def start(self):
        try:
            self.test_plan = build_joint7_test_plan(
                self.model, self.zone, self.lower, self.upper, self.center_goal
            )
        except ValueError as exc:
            self.status.set("TEST DISABLED: safety preflight rejected: " + str(exc))
            return
        if not self.hardware:
            self.status.set(
                "Offline preview only: safe center → J7 −10° → center path; "
                "no CAN opened and no arm moved"
            )
            return
        self.gripper_closed_latched = False
        self.gripper_close_deadline = None
        self.gripper_in_tolerance_since = None
        self.last_gripper_command = 0.0
        self.strike_speed_fast = False
        self.continue_button.config(state="disabled")
        super().start()
        if self.phase == "SETTING UP":
            self.status.set(
                f"Centering right arm at custom J7={math.degrees(self.center_goal[6]):.1f}° "
                "and opening the gripper to −3°"
            )

    def centered(self, now=None):
        if abs(self.gripper_encoder() - RIGHT_GRIPPER_OPEN) > GRIPPER_TOLERANCE:
            self.status.set("Right arm centered — waiting for gripper to reach −3°")
            return
        self.control = None
        self.phase = "WAITING FOR LOAD"
        self.continue_button.config(state="normal")
        self.status.set(
            "Right arm centered with gripper fully open at −3° — "
            "load it, then press Continue"
        )

    def continue_motion(self):
        if not self.hardware or self.phase != "WAITING FOR LOAD":
            return
        if not self.bus.fresh():
            self.status.set("Cannot continue: waiting for fresh encoder feedback")
            return
        if not self.zone.contains(self.tcp(), MEMBERSHIP_BUFFER_M):
            self.status.set("Cannot continue: right gripper TCP is outside zone1")
            return
        try:
            self.bus.set_gripper(RIGHT_GRIPPER_CLOSED)
        except Exception as exc:
            self.fail(str(exc))
            return
        now = time.monotonic()
        self.gripper_closed_latched = True
        self.gripper_close_deadline = now + GRIPPER_CLOSE_TIMEOUT_SECONDS
        self.gripper_in_tolerance_since = None
        self.last_gripper_command = now
        self.control = None
        self.phase = "CLOSING GRIPPER"
        self.continue_button.config(state="disabled")
        self.status.set(
            "Closing gripper to +7° — J7 will not move until encoder closure is confirmed"
        )

    def extra_control(self, now):
        if not self.hardware:
            return
        self.continue_button.config(
            state="normal" if (
                self.phase == "WAITING FOR LOAD" and self.bus.fresh()
            ) else "disabled"
        )
        if not self.bus.active:
            return

        target = None
        if self.gripper_closed_latched:
            target = RIGHT_GRIPPER_CLOSED
        elif self.phase in ("CENTERING", "WAITING FOR LOAD"):
            target = RIGHT_GRIPPER_OPEN
        if (target is not None
                and now - self.last_gripper_command >= GRIPPER_COMMAND_INTERVAL):
            self.bus.set_gripper(target)
            self.last_gripper_command = now

        if self.phase != "CLOSING GRIPPER":
            return
        error = abs(self.gripper_encoder() - RIGHT_GRIPPER_CLOSED)
        if error <= GRIPPER_TOLERANCE:
            if self.gripper_in_tolerance_since is None:
                self.gripper_in_tolerance_since = now
            held = now - self.gripper_in_tolerance_since
            remaining = max(0.0, GRIPPER_CLOSED_HOLD_SECONDS - held)
            self.status.set(
                f"Gripper encoder at +7° — confirming closure for {remaining:.2f} s"
            )
            if held >= GRIPPER_CLOSED_HOLD_SECONDS:
                self._begin_joint7_test()
            return

        self.gripper_in_tolerance_since = None
        remaining = max(0.0, self.gripper_close_deadline - now)
        self.status.set(
            f"Closing gripper to +7°; encoder error "
            f"{math.degrees(error):.2f}° ({remaining:.1f} s timeout)"
        )
        if remaining <= 0.0:
            self.fail("gripper did not fully close to +7° within 3.0 seconds")

    def _begin_joint7_test(self):
        if self.phase != "CLOSING GRIPPER" or self.test_plan is None:
            return
        try:
            self.bus.set_right_joint7_speed(RIGHT_STRIKE_SPEED)
            self.strike_speed_fast = True
            self._begin_strike_stage(DOWN_PHASE, self.test_plan.lowered_joints)
        except Exception as exc:
            self.fail("could not start fast right-J7 motion: " + str(exc))

    def _begin_strike_stage(self, name, desired):
        desired = np.asarray(desired, dtype=float)
        self.control = StrikeControl(
            desired, self.lower, self.upper, time.monotonic()
        )
        self.phase = name
        self.bus.set_positions(desired)
        self.last_command = 0.0
        if name == DOWN_PHASE:
            self.status.set(
                f"Fast right J7 move: −10° at {RIGHT_STRIKE_SPEED:.1f} rad/s"
            )
        else:
            self.status.set(
                "J7 −10° reached — immediately returning to the custom center"
            )

    def complete_stage(self, now=None):
        if self.phase == DOWN_PHASE:
            self._begin_strike_stage(RETURN_PHASE, self.test_plan.center_joints)
            return
        if self.phase == RETURN_PHASE:
            if not self._restore_strike_speed():
                return
            self.control = None
            self.relax(
                "TEST COMPLETE: right J7 moved down 10° and returned to custom center; "
                "right motors disabled"
            )
            return
        super().complete_stage(now)

    def _restore_strike_speed(self):
        if not getattr(self, "strike_speed_fast", False):
            return True
        try:
            if self.bus and self.bus.active:
                self.bus.set_right_joint7_speed(SPEED)
            self.strike_speed_fast = False
            return True
        except Exception as exc:
            JointGoalApp.relax(
                self,
                "STOPPED: could not restore normal right-J7 speed: "
                + str(exc) + "; right motors disabled",
            )
            self.phase = "FAULT"
            return False

    def fail(self, message):
        if not self._restore_strike_speed():
            return
        super().fail(message)

    def safety(self):
        if (self.strike_speed_fast and self.bus and self.bus.active
                and not self.zone.contains(self.tcp(), MEMBERSHIP_BUFFER_M)):
            if not self._restore_strike_speed():
                return
        super().safety()

    def relax(self, message=None):
        self.continue_button.config(state="disabled")
        super().relax(message)
        if not self.bus or not self.bus.active:
            self.strike_speed_fast = False


def main():
    parser = argparse.ArgumentParser(
        description="Right-arm custom-center and ten-degree J7 motion test"
    )
    parser.add_argument("--hardware", action="store_true")
    args = parser.parse_args()
    rclpy.init(args=[])
    try:
        App(args.hardware).run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
