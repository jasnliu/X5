"""In-memory right-arm motor substitute used only by camera-playback --test."""
from __future__ import annotations

import math
import time

import numpy as np
from safe_zone.gripper_feedback import motor8_feedback
from .left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET, LeftCenterMonitor

from centering.motors import (
    MAX_RIGHT_JOINT7_FEEDBACK_HZ,
    RIGHT_GRIPPER_CLOSED,
    RIGHT_GRIPPER_OPEN,
    RIGHT_PLAYBACK_SPEED,
    RIGHT_STRIKE_DOWN_SPEED,
    RIGHT_STRIKE_RETURN_SPEED,
    SPEED,
)


class SimulatedMotors:
    """Small motor-API simulation with no SocketCAN or serial access."""

    def __init__(self, initial_joints, initial_gripper=RIGHT_GRIPPER_OPEN):
        joints = np.asarray(initial_joints, dtype=float)
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise ValueError("Invalid simulated-arm initial joints")
        self.control_side = "right"
        self.control_gripper = True
        self.active = False
        self.closed = False
        self._joints = joints.copy()
        self._targets = joints.copy()
        self._left_joints = np.zeros(7)
        self._left_gripper = 0.
        self._left_targets = LEFT_CENTER.copy()
        self.left_playback_active = False
        self.center_disabled = {}
        self.dual_center_history = None
        self.left_monitor = None
        self._speeds = np.full(7, SPEED, dtype=float)
        self._gripper = float(initial_gripper)
        self._gripper_target = float(initial_gripper)
        self._last_poll = time.monotonic()
        self.right_joint7_feedback_hz = None
        self.states = {}
        self._refresh_states(self._last_poll)

    def _refresh_states(self, now: float) -> None:
        self.motor8_feedback = {
            side: motor8_feedback(round((-q + 12.57) * 65535 / 25.14))
            for side, q in (('left', self._left_gripper), ('right', self._gripper))
        }
        mode = 1 if self.active and 'right' not in self.center_disabled else 0
        self.states = {
            **{("left", index): (float(self._left_joints[index - 1]),
                                 2 if self.active and 'left' not in self.center_disabled else 0, now) for index in range(1, 8)},
            ("left", 8): (self._left_gripper, 2 if self.active and 'left' not in self.center_disabled else 0, now),
            **{("right", index): (float(self._joints[index - 1]), mode, now)
               for index in range(1, 8)},
            ("right", 8): (float(self._gripper), mode, now),
        }

    @staticmethod
    def _advance(current: np.ndarray, target: np.ndarray,
                 speed: np.ndarray, elapsed: float) -> np.ndarray:
        maximum = np.maximum(0.0, speed) * max(0.0, elapsed)
        return current + np.clip(target - current, -maximum, maximum)

    def poll(self) -> None:
        if self.closed:
            raise RuntimeError("Simulated motor controller is closed")
        now = time.monotonic()
        elapsed = min(max(0.0, now - self._last_poll), 0.1)
        self._last_poll = now
        if self.active and 'left' not in self.center_disabled:
            self._left_gripper = float(self._advance(
                np.array([self._left_gripper]), np.array([LEFT_GRIPPER_TARGET]),
                np.array([SPEED]), elapsed,
            )[0])
            self._left_joints = self._advance(
                self._left_joints, self._left_targets, np.full(7, .8 if self.left_playback_active else SPEED), elapsed)
        if self.active and "right" not in self.center_disabled:
            self._joints = self._advance(
                self._joints, self._targets, self._speeds, elapsed
            )
            self._gripper = float(self._advance(
                np.array([self._gripper]), np.array([self._gripper_target]),
                np.array([SPEED]), elapsed,
            )[0])
        self._refresh_states(now)
        if self.active and self.dual_center_history is not None:
            for side, history in self.dual_center_history.items():
                if side in self.center_disabled:
                    continue
                q = self._left_joints if side == 'left' else self._joints
                history.append((now, q.copy()))
                while history and now-history[0][0] > .65:
                    history.popleft()
        if self.active and self.left_monitor is not None:
            self.left_monitor.update(self.states, now)

    def fresh(self) -> bool:
        return not self.closed

    def positions(self) -> dict[str, float]:
        positions = {
            **{f"openarmx_left_joint{i + 1}": float(q)
               for i, q in enumerate(self._left_joints)},
            **{f"openarmx_right_joint{i + 1}": float(self._joints[i])
               for i in range(7)},
            "openarmx_left_finger_joint1": max(0., min(.044, .044 * self._left_gripper / 1.0472)),
        }
        fraction = (
            (RIGHT_GRIPPER_CLOSED - self._gripper)
            / (RIGHT_GRIPPER_CLOSED - RIGHT_GRIPPER_OPEN)
        )
        positions["openarmx_right_finger_joint1"] = (
            .044 * max(0.0, min(1.0, fraction))
        )
        return positions

    def center(self, target=None, gripper_target=None):
        target = np.zeros(7) if target is None else np.asarray(target, dtype=float)
        if target.shape != (7,) or not np.isfinite(target).all():
            raise RuntimeError("Invalid simulated center target")
        if gripper_target is not None:
            gripper_target = float(gripper_target)
            if (not math.isfinite(gripper_target)
                    or not RIGHT_GRIPPER_OPEN <= gripper_target <= RIGHT_GRIPPER_CLOSED):
                raise RuntimeError("Invalid simulated gripper target")
        self.active = True
        self.center_disabled = {}
        self.dual_center_history = None
        self._left_targets = LEFT_CENTER.copy()
        self.left_monitor = LeftCenterMonitor(time.monotonic())
        self._speeds.fill(SPEED)
        self._targets = target.copy()
        self.center_goal = target.copy()
        if gripper_target is not None:
            self._gripper_target = gripper_target
        self._refresh_states(time.monotonic())
        yield

    def left_center_ready(self):
        return (self.left_monitor is not None and np.array_equal(self.left_monitor.goal, LEFT_CENTER)
                and self.left_monitor.ready
                and not self.left_monitor.fault)

    def set_left_hold(self, target):
        self._left_targets = np.asarray(target).copy()
        self.left_playback_active = False
        self.left_monitor = LeftCenterMonitor(time.monotonic(), target)

    def left_hold_ready(self):
        return self.left_monitor is not None and self.left_monitor.ready and not self.left_monitor.fault

    def begin_left_playback(self):
        if not self.left_hold_ready():
            raise RuntimeError('Left start not reached')
        self.left_monitor = None
        self.left_playback_active = True

    def set_left_positions(self, target):
        if not self.left_playback_active:
            raise RuntimeError('No left recording owns targets')
        self._left_targets = np.asarray(target).copy()

    def begin_dual_center(self, right_start=None, left_target=None):
        from collections import deque
        self.dual_center_history = {side: deque() for side in ('left', 'right')}
        if 'left' not in self.center_disabled:
            self.set_left_hold(LEFT_CENTER if left_target is None else left_target)
        if 'right' not in self.center_disabled:
            if right_start is None:
                self.set_positions(self.center_goal)
            else:
                right_start()

    def disable_centered_side(self, side):
        goal = LEFT_CENTER if side == 'left' else self.center_goal
        q = self._left_joints if side == 'left' else self._joints
        history = self.dual_center_history[side]
        if len(history) < 2 or history[-1][0]-history[0][0] < .6:
            raise RuntimeError('Center not settled yet')
        values = np.array([q for _, q in history])
        if (np.max(np.abs(q-goal)) > math.radians(.20)
                or np.max(np.abs(values-goal)) > math.radians(.20)
                or np.max(np.ptp(values, axis=0)) > math.radians(.12)):
            raise RuntimeError('Center not settled yet')
        self.center_disabled[side] = time.monotonic()
        if side == 'left':
            self.left_monitor = None
        self._refresh_states(time.monotonic())

    def set_positions(self, joints) -> None:
        joints = np.asarray(joints, dtype=float)
        if not self.active or joints.shape != (7,) or not np.isfinite(joints).all():
            raise RuntimeError("Simulated right arm is not active or target is invalid")
        self._targets = joints.copy()

    def set_right_joint7_position(self, target) -> None:
        target = float(target)
        if not self.active or not math.isfinite(target) or abs(target) > 3.5:
            raise RuntimeError("Simulated right J7 target is invalid")
        self._targets[6] = target

    def set_gripper(self, target) -> None:
        target = float(target)
        if (not self.active or not math.isfinite(target)
                or not RIGHT_GRIPPER_OPEN <= target <= RIGHT_GRIPPER_CLOSED):
            raise RuntimeError("Invalid simulated right-gripper target")
        self._gripper_target = target

    def set_right_joint7_speed(self, speed) -> None:
        speed = float(speed)
        if not self.active or speed not in (
                SPEED, RIGHT_STRIKE_RETURN_SPEED, RIGHT_STRIKE_DOWN_SPEED):
            raise RuntimeError("Invalid simulated J7 speed")
        self._speeds[6] = speed

    def set_right_joint7_feedback_rate(self, rate_hz=None) -> None:
        if rate_hz is None:
            self.right_joint7_feedback_hz = None
            return
        rate_hz = float(rate_hz)
        if (not self.active or not math.isfinite(rate_hz) or rate_hz <= 0.0
                or rate_hz > MAX_RIGHT_JOINT7_FEEDBACK_HZ):
            raise RuntimeError("Invalid simulated J7 feedback rate")
        self.right_joint7_feedback_hz = rate_hz

    def set_right_arm_speed(self, speed) -> None:
        speed = float(speed)
        if not self.active or speed not in (SPEED, RIGHT_PLAYBACK_SPEED):
            raise RuntimeError("Invalid simulated playback speed")
        self._speeds.fill(speed)

    def relax(self) -> None:
        self.active = False
        self.left_monitor = None
        self.right_joint7_feedback_hz = None
        self._targets = self._joints.copy()
        self._gripper_target = self._gripper
        self._refresh_states(time.monotonic())

    def close(self) -> None:
        self.active = False
        self.closed = True
