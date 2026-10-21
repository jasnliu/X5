"""Safe local Cartesian candidates anchored at a recorded endpoint posture."""
from __future__ import annotations

import math
import numpy as np
from scipy.optimize import least_squares, minimize

from camera_search.planner import (
    IK_SEED_INTERIOR_MARGIN_RAD,
    IK_SEED_LIMIT_TOLERANCE_RAD,
)
from cartesian_goal.ik import IKResult, IK_TOLERANCE_M, PATH_PERIOD_S
from safe_zone.geometry import MEMBERSHIP_BUFFER_M


FREE_WITH_FIXED_J2 = np.array([0, 2, 3, 4, 5, 6], dtype=int)


def _path_samples(ik, start, end):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start
    duration = float(np.max(np.abs(delta)) / ik.speed)
    steps = max(1, int(math.ceil(duration / PATH_PERIOD_S)))
    for index in range(steps + 1):
        yield start + delta * (index / steps)


def _path_inside(ik, start, end) -> bool:
    return all(
        ik.zone.contains(ik.position(joints), MEMBERSHIP_BUFFER_M)
        for joints in _path_samples(ik, start, end)
    )


def _transition_and_center_returns_inside(ik, start, end) -> bool:
    for joints in _path_samples(ik, start, end):
        if not ik.zone.contains(ik.position(joints), MEMBERSHIP_BUFFER_M):
            return False
        if not _path_inside(ik, joints, ik.home):
            return False
    return True


def solve_playback_coordinate(ik, offset, previous, fixed_joint2) -> IKResult:
    """Solve a small Cartesian adjustment while retaining the recorded J2."""
    offset = np.asarray(offset, dtype=float)
    previous = np.asarray(previous, dtype=float)
    fixed_joint2 = float(fixed_joint2)
    if (offset.shape != (3,) or previous.shape != (7,)
            or not np.isfinite(offset).all() or not np.isfinite(previous).all()
            or not math.isfinite(fixed_joint2)
            or fixed_joint2 < ik.lower[1] or fixed_joint2 > ik.upper[1]):
        raise ValueError("Invalid playback hill-climber anchor")
    target = ik.origin_tcp + offset
    if not ik.zone.contains(target, MEMBERSHIP_BUFFER_M):
        raise ValueError("Candidate coordinate is outside right zone1")
    free = FREE_WITH_FIXED_J2
    lower = ik.lower[free]
    upper = ik.upper[free]
    raw_seed = previous[free]
    outside = np.maximum(np.maximum(lower - raw_seed, raw_seed - upper), 0.0)
    if np.any(outside > IK_SEED_LIMIT_TOLERANCE_RAD):
        worst = int(np.argmax(outside))
        raise ValueError(
            f"Current J{int(free[worst] + 1)} position is outside IK limits by "
            f"{math.degrees(outside[worst]):.4f} deg, beyond encoder tolerance"
        )
    margin = np.minimum(IK_SEED_INTERIOR_MARGIN_RAD, (upper - lower) / 4.0)
    seed = np.clip(raw_seed, lower + margin, upper - margin)

    def expand(values):
        joints = previous.copy()
        joints[free] = values
        joints[1] = fixed_joint2
        return joints

    fit = least_squares(
        lambda values: ik.position(expand(values)) - target,
        seed,
        bounds=(lower, upper),
        max_nfev=600,
        ftol=1e-11,
        xtol=1e-11,
        gtol=1e-11,
    )
    best = expand(fit.x)
    error = float(np.linalg.norm(ik.position(best) - target))
    if error > IK_TOLERANCE_M:
        raise ValueError("Candidate coordinate is not reachable within 2 mm with recorded J2 fixed")
    refined = minimize(
        lambda values: .5 * float((expand(values) - previous) @ (expand(values) - previous)),
        fit.x,
        method="SLSQP",
        bounds=list(zip(lower, upper)),
        constraints={"type": "eq", "fun": lambda values: ik.position(expand(values)) - target},
        options={"maxiter": 500, "ftol": 1e-12},
    )
    if refined.success:
        candidate = expand(refined.x)
        candidate_error = float(np.linalg.norm(ik.position(candidate) - target))
        if candidate_error <= IK_TOLERANCE_M:
            best, error = candidate, candidate_error
    if (not _transition_and_center_returns_inside(ik, previous, best)
            or not _path_inside(ik, best, previous)):
        raise ValueError("Candidate transition, anchor return, or center return leaves right zone1")
    return IKResult(best, target, error)
