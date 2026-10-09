"""Safe joint-7 cymbal-strike planning and fast endpoint monitoring."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from camera_search.control import ENCODER_LSB_RAD
from goal_motion.control import STALL_ERROR, STALL_MOVEMENT
from safe_zone.geometry import MEMBERSHIP_BUFFER_M, RIGHT_TCP


STRIKE_START_DEGREES = 5
STRIKE_INCREMENT_DEGREES = 1
STRIKE_INCREMENT_RAD = math.radians(STRIKE_INCREMENT_DEGREES)
STRIKE_PATH_SAMPLE_RAD = math.radians(1.0)
STRIKE_REACHED_TOLERANCE_RAD = math.radians(2.0)
STRIKE_OTHER_JOINT_TOLERANCE_RAD = math.radians(3.0)
STRIKE_STALL_SECONDS = 0.5
STRIKE_TIMEOUT_SECONDS = 3.0
LIMIT_TOLERANCE_RAD = 3.0 * ENCODER_LSB_RAD
MANUAL_STRIKE_MIN_RAD = 2.0 * ENCODER_LSB_RAD
MANUAL_STRIKE_MIN_DEGREES = math.degrees(MANUAL_STRIKE_MIN_RAD)


@dataclass(frozen=True)
class StrikePlan:
    anchor_joints: np.ndarray
    targets: tuple[np.ndarray, ...]


def _samples(start, end, maximum_step=STRIKE_PATH_SAMPLE_RAD):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start
    steps = max(1, int(math.ceil(float(np.max(np.abs(delta))) / maximum_step)))
    for index in range(steps + 1):
        yield start + delta * (index / steps)


def _tcp(model, joints):
    values = {
        f"openarmx_right_joint{i + 1}": float(joints[i]) for i in range(7)
    }
    return model.transforms(values)[RIGHT_TCP][:3, 3]


def _path_inside(model, zone, start, end) -> bool:
    return all(
        zone.contains(_tcp(model, joints), MEMBERSHIP_BUFFER_M)
        for joints in _samples(start, end)
    )


def build_strike_plan(model, zone, lower, upper, anchor) -> StrikePlan:
    """Build safe one-degree J7 targets beginning five degrees from anchor."""
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    anchor = np.asarray(anchor, dtype=float)
    if any(values.shape != (7,) for values in (lower, upper, anchor)):
        raise ValueError("Invalid cymbal-strike joint vectors")
    if not all(np.isfinite(values).all() for values in (lower, upper, anchor)):
        raise ValueError("Cymbal-strike joint vectors must be finite")
    outside = np.maximum(np.maximum(lower - anchor, anchor - upper), 0.0)
    if np.any(outside > LIMIT_TOLERANCE_RAD):
        joint = int(np.argmax(outside))
        raise ValueError(
            f"Cymbal position J{joint + 1} exceeds its limit by "
            f"{math.degrees(outside[joint]):.4f} deg"
        )
    anchor = np.clip(anchor, lower, upper)
    if not zone.contains(_tcp(model, anchor), MEMBERSHIP_BUFFER_M):
        raise ValueError("Cymbal position is outside right zone1")

    targets = []
    deepest_safe = anchor
    amount_degrees = STRIKE_START_DEGREES
    while True:
        joint7 = float(anchor[6] - math.radians(amount_degrees))
        if joint7 < lower[6] - 1e-12:
            break
        target = anchor.copy()
        target[6] = max(joint7, lower[6])
        # This check is intentionally limited to the actual J7 strike path so
        # the first attempt can start directly after camera acceptance. The
        # aligned anchor/correction paths have already been validated upstream.
        if not _path_inside(model, zone, deepest_safe, target):
            break
        targets.append(target)
        deepest_safe = target
        amount_degrees += STRIKE_INCREMENT_DEGREES
    if not targets:
        raise ValueError(
            "The initial 5-degree J7 strike does not stay inside right zone1"
        )
    return StrikePlan(anchor.copy(), tuple(target.copy() for target in targets))


def build_manual_strike_target(model, zone, lower, upper, anchor,
                               amount_degrees: float) -> np.ndarray:
    """Build one exact, safe J7-down target for a user-entered degree amount."""
    if (isinstance(amount_degrees, bool)
            or not isinstance(amount_degrees, (int, float, np.integer, np.floating))
            or not math.isfinite(float(amount_degrees))):
        raise ValueError("Strike degrees must be a finite number")
    amount_degrees = float(amount_degrees)
    if amount_degrees < MANUAL_STRIKE_MIN_DEGREES:
        raise ValueError(
            f"Strike degrees must be at least {MANUAL_STRIKE_MIN_DEGREES:.3f}° "
            "to exceed two J7 encoder counts"
        )

    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    anchor = np.asarray(anchor, dtype=float)
    if any(values.shape != (7,) for values in (lower, upper, anchor)):
        raise ValueError("Invalid manual-strike joint vectors")
    if not all(np.isfinite(values).all() for values in (lower, upper, anchor)):
        raise ValueError("Manual-strike joint vectors must be finite")
    outside = np.maximum(np.maximum(lower - anchor, anchor - upper), 0.0)
    if np.any(outside > LIMIT_TOLERANCE_RAD):
        joint = int(np.argmax(outside))
        raise ValueError(
            f"Cymbal position J{joint + 1} exceeds its limit by "
            f"{math.degrees(outside[joint]):.4f} deg"
        )
    anchor = np.clip(anchor, lower, upper)
    if not zone.contains(_tcp(model, anchor), MEMBERSHIP_BUFFER_M):
        raise ValueError("Cymbal position is outside right zone1")

    target = anchor.copy()
    target[6] = float(anchor[6] - math.radians(amount_degrees))
    if target[6] < lower[6] or target[6] > upper[6]:
        raise ValueError(
            f"Entered {amount_degrees:g}° strike exceeds the J7 joint limit"
        )
    if not _path_inside(model, zone, anchor, target):
        raise ValueError(
            f"Entered {amount_degrees:g}° strike leaves right zone1"
        )
    return target


class StrikeControl:
    """Exact, no-overshoot strike commands with immediate reached detection."""

    def __init__(self, desired, lower, upper, now: float,
                 reached_tolerance_rad: float = STRIKE_REACHED_TOLERANCE_RAD,
                 timeout_seconds: float = STRIKE_TIMEOUT_SECONDS):
        self.desired = np.asarray(desired, dtype=float).copy()
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.reached_tolerance_rad = float(reached_tolerance_rad)
        self.timeout_seconds = float(timeout_seconds)
        if (self.desired.shape != (7,) or self.lower.shape != (7,) or self.upper.shape != (7,)
                or not np.isfinite(self.desired).all()
                or np.any(self.desired < self.lower) or np.any(self.desired > self.upper)
                or not math.isfinite(self.reached_tolerance_rad)
                or self.reached_tolerance_rad <= 0.0
                or not math.isfinite(self.timeout_seconds)
                or self.timeout_seconds <= 0.0):
            raise ValueError("Invalid cymbal-strike goal")
        self.started = float(now)
        self.history = deque()

    def update(self, actual, now: float):
        actual = np.asarray(actual, dtype=float)
        if actual.shape != (7,) or not np.isfinite(actual).all():
            raise RuntimeError("Invalid encoder feedback")
        error = self.desired - actual
        self.history.append((float(now), actual.copy()))
        while self.history and now - self.history[0][0] > STRIKE_STALL_SECONDS:
            self.history.popleft()
        if self.history and now - self.history[0][0] >= STRIKE_STALL_SECONDS - .05:
            span = np.ptp(np.asarray([joints for _, joints in self.history]), axis=0)
            stalled = (np.abs(error) > STALL_ERROR) & (span < STALL_MOVEMENT)
            if np.any(stalled):
                raise RuntimeError("STALL: motor " + str(int(np.flatnonzero(stalled)[0]) + 1))
        reached = (
            abs(error[6]) <= self.reached_tolerance_rad
            and np.max(np.abs(error[:6])) <= STRIKE_OTHER_JOINT_TOLERANCE_RAD
        )
        if now - self.started > self.timeout_seconds:
            raise RuntimeError("Cymbal strike position timeout")
        return self.desired.copy(), error, reached
