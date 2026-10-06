"""Offline safety preflight for the right-J7 ten-degree test motion."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from safe_zone.geometry import MEMBERSHIP_BUFFER_M, RIGHT_TCP


TEST_J7_DELTA_RAD = math.radians(10.0)
PATH_SAMPLE_RAD = math.radians(1.0)


@dataclass(frozen=True)
class Joint7TestPlan:
    center_joints: np.ndarray
    lowered_joints: np.ndarray


def _tcp(model, joints):
    values = {
        f"openarmx_right_joint{i + 1}": float(joints[i]) for i in range(7)
    }
    return model.transforms(values)[RIGHT_TCP][:3, 3]


def _samples(start, end):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start
    steps = max(1, int(math.ceil(float(np.max(np.abs(delta))) / PATH_SAMPLE_RAD)))
    for index in range(steps + 1):
        yield start + delta * (index / steps)


def build_joint7_test_plan(model, zone, lower, upper, center_goal) -> Joint7TestPlan:
    """Validate the exact custom-center -> J7 -10 deg -> center round trip."""
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    center = np.asarray(center_goal, dtype=float)
    if any(values.shape != (7,) for values in (lower, upper, center)):
        raise ValueError("Invalid right-J7 test joint vectors")
    if not all(np.isfinite(values).all() for values in (lower, upper, center)):
        raise ValueError("Right-J7 test joint vectors must be finite")
    if np.any(center < lower) or np.any(center > upper):
        raise ValueError("Customized right center exceeds a joint limit")

    lowered = center.copy()
    lowered[6] -= TEST_J7_DELTA_RAD
    if np.any(lowered < lower) or np.any(lowered > upper):
        raise ValueError("J7 cannot move 10 degrees below the customized center")

    for name, start, end in (
        ("outbound", center, lowered),
        ("return", lowered, center),
    ):
        for joints in _samples(start, end):
            if not zone.contains(_tcp(model, joints), MEMBERSHIP_BUFFER_M):
                raise ValueError(
                    f"Right-J7 {name} path leaves buffered right zone1"
                )
    return Joint7TestPlan(center.copy(), lowered.copy())
