"""Preflight the fixed goal and +Y search before any motor is enabled."""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import least_squares, minimize

from camera_search.control import ENCODER_LSB_RAD
from cartesian_goal.ik import IKResult, IK_TOLERANCE_M, PATH_PERIOD_S
from safe_zone.geometry import MEMBERSHIP_BUFFER_M

INITIAL_OFFSET = np.array([0.25, 0.0, 0.35], dtype=float)
Y_STEP_M = 0.01
MAX_SCAN_POINTS = 1000
# Live right-arm testing showed joint 2 sitting at its centered encoder value
# while small negative targets accumulated at the 10 mm scan points.  The arm
# has ample Cartesian redundancy, so keep joint 2 at the same zero used by the
# requested customized center and produce the +Y motion with the other joints.
SEARCH_FREE_JOINTS = np.array([0, 2, 3, 4, 5, 6], dtype=int)
# A commanded joint at a URDF limit can be reported a few encoder counts past
# that limit.  SciPy rejects such a value as an initial point before doing any
# optimization.  Permit only this quantization-sized discrepancy, and clip only
# the numerical seed; the unmodified measured joints remain the path/safety
# anchor below.  Anything farther out still indicates a real limit violation.
IK_SEED_LIMIT_TOLERANCE_RAD = 3.0 * ENCODER_LSB_RAD
IK_SEED_INTERIOR_MARGIN_RAD = 1e-9


@dataclass(frozen=True)
class SearchPlan:
    results: tuple[IKResult, ...]
    first_outside_offset: np.ndarray


def scan_offsets(ik, zone, start=INITIAL_OFFSET, step=Y_STEP_M) -> tuple[list[np.ndarray], np.ndarray]:
    start = np.asarray(start, dtype=float)
    if start.shape != (3,) or not np.isfinite(start).all() or not math.isfinite(step) or step <= 0:
        raise ValueError("Invalid camera-search coordinate configuration")
    offsets = []
    for index in range(MAX_SCAN_POINTS):
        offset = start.copy()
        offset[1] = start[1] + index * step
        target = ik.origin_tcp + offset
        if not zone.contains(target, MEMBERSHIP_BUFFER_M):
            if not offsets:
                raise ValueError("Initial camera-search coordinate is outside right zone1")
            return offsets, offset
        offsets.append(offset)
    raise ValueError("Camera-search zone boundary was not found")


def _path_samples(ik, start, end):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start
    duration = float(np.max(np.abs(delta)) / ik.speed)
    steps = max(1, int(math.ceil(duration / PATH_PERIOD_S)))
    for index in range(steps + 1):
        travel = min(duration, index * duration / steps) * ik.speed
        yield start + np.sign(delta) * np.minimum(np.abs(delta), travel)


def _path_inside(ik, start, end) -> bool:
    for joints in _path_samples(ik, start, end):
        if not ik.zone.contains(ik.position(joints), MEMBERSHIP_BUFFER_M):
            return False
    return True


def _transition_and_immediate_returns_inside(ik, start, end) -> bool:
    """Detection may request recentering at any sampled point in a transition."""
    for joints in _path_samples(ik, start, end):
        if not ik.zone.contains(ik.position(joints), MEMBERSHIP_BUFFER_M):
            return False
        if not _path_inside(ik, joints, ik.home):
            return False
    return True


def solve_search_coordinate(ik, offset, previous) -> IKResult:
    """Solve one safe coordinate from the current settled joint anchor.

    Joint 2 remains at the customized center.  The candidate transition, its
    reverse back to the anchor, and an emergency return to center from every
    transition sample must all remain in the buffered right-arm zone.
    """
    offset = np.asarray(offset, dtype=float)
    previous = np.asarray(previous, dtype=float)
    if (offset.shape != (3,) or previous.shape != (7,)
            or not np.isfinite(offset).all() or not np.isfinite(previous).all()):
        raise ValueError("Invalid hill-climber coordinate or joint anchor")
    target = ik.origin_tcp + offset
    if not ik.zone.contains(target, MEMBERSHIP_BUFFER_M):
        raise ValueError("Candidate coordinate is outside right zone1")
    free = SEARCH_FREE_JOINTS
    lower = ik.lower[free]
    upper = ik.upper[free]
    raw_seed = previous[free]
    outside = np.maximum(np.maximum(lower - raw_seed, raw_seed - upper), 0.0)
    if np.any(outside > IK_SEED_LIMIT_TOLERANCE_RAD):
        worst = int(np.argmax(outside))
        joint_number = int(free[worst] + 1)
        raise ValueError(
            f"Current J{joint_number} position is outside IK limits by "
            f"{math.degrees(outside[worst]):.4f} deg, beyond encoder tolerance"
        )
    margin = np.minimum(IK_SEED_INTERIOR_MARGIN_RAD, (upper - lower) / 4.0)
    optimizer_seed = np.clip(raw_seed, lower + margin, upper - margin)

    def expand(values):
        joints = ik.home.copy()
        joints[free] = values
        joints[1] = ik.home[1]
        return joints

    fit = least_squares(
        lambda values: ik.position(expand(values)) - target,
        optimizer_seed,
        bounds=(lower, upper),
        max_nfev=600,
        ftol=1e-11,
        xtol=1e-11,
        gtol=1e-11,
    )
    best = expand(fit.x)
    error = float(np.linalg.norm(ik.position(best) - target))
    if error > IK_TOLERANCE_M:
        raise ValueError("Candidate coordinate is not reachable within 2 mm with joint 2 centered")
    refined = minimize(
        lambda values: .5 * float((expand(values) - previous) @ (expand(values) - previous)),
        fit.x,
        method="SLSQP",
        bounds=list(zip(ik.lower[free], ik.upper[free])),
        constraints={"type": "eq", "fun": lambda values: ik.position(expand(values)) - target},
        options={"maxiter": 500, "ftol": 1e-12},
    )
    if refined.success:
        refined_joints = expand(refined.x)
        refined_error = float(np.linalg.norm(ik.position(refined_joints) - target))
        if refined_error <= IK_TOLERANCE_M:
            best = refined_joints
            error = refined_error
    if (not _transition_and_immediate_returns_inside(ik, previous, best)
            or not _path_inside(ik, best, previous)):
        raise ValueError("Candidate transition, anchor return, or center return leaves right zone1")
    return IKResult(best, target, error)


def build_search_plan(ik, zone, start=INITIAL_OFFSET, step=Y_STEP_M) -> SearchPlan:
    offsets, first_outside = scan_offsets(ik, zone, start, step)
    first = solve_search_coordinate(ik, offsets[0], ik.home)
    results = [first]
    for offset in offsets[1:]:
        result = solve_search_coordinate(ik, offset, results[-1].joints)
        results.append(result)
    return SearchPlan(tuple(results), first_outside)
