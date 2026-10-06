"""Load, smooth, validate, and monitor right-arm motion recordings."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np

from camera_search.control import ENCODER_LSB_RAD
from cartesian_goal.ik import PATH_PERIOD_S
from goal_motion.control import STALL_ERROR, STALL_MOVEMENT, STALL_SECONDS
from motion_recording.recording import (
    GRIPPER_NAME,
    JOINT_NAMES,
    MAX_SAMPLES,
    SCHEMA,
)
from safe_zone.geometry import MEMBERSHIP_BUFFER_M, RIGHT_TCP


MAX_RECORDING_BYTES = 80_000_000
LIMIT_TOLERANCE_RAD = 3.0 * ENCODER_LSB_RAD
# Use one uniform time multiplier for the whole recording.  Only the fitted
# curve's peak velocity participates: acceleration estimates from high-rate
# encoder noise deliberately do not slow playback.  A small margin below the
# configured firmware speed helps the physical joints keep up.
PLAYBACK_SPEED_FRACTION = 0.90
SMOOTHING_HALF_WINDOW_S = 0.10
SMOOTHING_MAX_RADIUS = 10
SMOOTH_PATH_CHECK_INTERVAL_S = 0.01
MAX_SMOOTH_PATH_CHECKS = 500_000
TRACKING_ERROR_RAD = math.radians(15.0)
TRACKING_ERROR_SECONDS = 1.0
RETURN_CHECK_ANCHORS = 31


def _finite_float(value, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{label} must be a finite number") from None
    if not math.isfinite(result):
        raise ValueError(f"{label} must be a finite number")
    return result


def _joint_path_samples(start, end, speed):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start
    duration = float(np.max(np.abs(delta)) / speed)
    steps = max(1, int(math.ceil(duration / PATH_PERIOD_S)))
    for index in range(steps + 1):
        yield start + delta * (index / steps)


def _path_inside(model, zone, start, end, speed) -> bool:
    for joints in _joint_path_samples(start, end, speed):
        values = {f"openarmx_right_joint{i + 1}": float(joints[i]) for i in range(7)}
        tcp = model.transforms(values)[RIGHT_TCP][:3, 3]
        if not zone.contains(tcp, MEMBERSHIP_BUFFER_M):
            return False
    return True


@dataclass(frozen=True)
class PlaybackTrajectory:
    source: Path
    joints: np.ndarray
    source_times: np.ndarray
    tcp_positions: np.ndarray
    gripper_openings: np.ndarray
    fitted_joints: np.ndarray
    curve_coefficients: np.ndarray
    time_scale: float

    @property
    def sample_count(self) -> int:
        return int(len(self.joints))

    @property
    def original_duration_s(self) -> float:
        return float(self.source_times[-1])

    @property
    def duration_s(self) -> float:
        return self.original_duration_s * self.time_scale

    @property
    def first_joints(self) -> np.ndarray:
        return self.joints[0].copy()

    @property
    def last_joints(self) -> np.ndarray:
        return self.joints[-1].copy()

    def joints_at(self, elapsed_s: float) -> tuple[np.ndarray, bool]:
        if self.sample_count == 1 or self.duration_s <= 0.0:
            return self.last_joints, True
        source_time = max(0.0, float(elapsed_s)) / self.time_scale
        if source_time >= self.source_times[-1]:
            return self.last_joints, True
        return self.joints_at_source_time(source_time), False

    def joints_at_source_time(self, source_time: float) -> np.ndarray:
        """Evaluate the fitted C2 joint curve in the recording's time base."""
        if self.sample_count == 1 or self.original_duration_s <= 0.0:
            return self.last_joints
        source_time = min(max(0.0, float(source_time)), self.original_duration_s)
        segment = int(np.searchsorted(self.source_times, source_time, side="right") - 1)
        segment = min(max(0, segment), len(self.curve_coefficients) - 1)
        local_time = source_time - self.source_times[segment]
        coefficients = self.curve_coefficients[segment]
        return (
            coefficients[0]
            + local_time * (
                coefficients[1]
                + local_time * (coefficients[2] + local_time * coefficients[3])
            )
        )


def _locally_smooth_joints(times: np.ndarray, joints: np.ndarray) -> np.ndarray:
    """Use a small local best-fit line to suppress encoder stair-step noise."""
    count = len(joints)
    if count < 3:
        return joints.copy()
    median_period = float(np.median(np.diff(times)))
    radius = min(
        SMOOTHING_MAX_RADIUS,
        max(1, int(math.ceil(SMOOTHING_HALF_WINDOW_S / median_period))),
    )
    fitted = joints.copy()
    sigma = max(SMOOTHING_HALF_WINDOW_S / 2.0, median_period)
    for index in range(1, count - 1):
        first = max(0, index - radius)
        last = min(count, index + radius + 1)
        offsets = times[first:last] - times[index]
        values = joints[first:last]
        weights = np.exp(-0.5 * np.square(offsets / sigma))
        sum_0 = float(np.sum(weights))
        sum_1 = float(np.sum(weights * offsets))
        sum_2 = float(np.sum(weights * offsets * offsets))
        determinant = sum_0 * sum_2 - sum_1 * sum_1
        if determinant <= np.finfo(float).eps:
            continue
        value_sum = np.sum(weights[:, None] * values, axis=0)
        slope_sum = np.sum((weights * offsets)[:, None] * values, axis=0)
        estimate = (sum_2 * value_sum - sum_1 * slope_sum) / determinant
        # A local fit must not invent a joint extreme absent from its window.
        fitted[index] = np.clip(estimate, np.min(values, axis=0), np.max(values, axis=0))
    # Starting and ending poses are operationally significant and remain exact.
    fitted[0] = joints[0]
    fitted[-1] = joints[-1]
    return fitted


def _clamped_cubic_coefficients(times: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Return C2 cubic coefficients with zero start/end velocity."""
    count = len(values)
    if count < 2:
        return np.empty((0, 4, values.shape[1]), dtype=float)
    intervals = np.diff(times)
    secants = np.diff(values, axis=0) / intervals[:, None]

    lower_diagonal = np.zeros(count, dtype=float)
    diagonal = np.zeros(count, dtype=float)
    upper_diagonal = np.zeros(count, dtype=float)
    right_hand_side = np.zeros((count, values.shape[1]), dtype=float)

    diagonal[0] = 2.0 * intervals[0]
    upper_diagonal[0] = intervals[0]
    right_hand_side[0] = 6.0 * secants[0]  # zero initial velocity
    for index in range(1, count - 1):
        before = intervals[index - 1]
        after = intervals[index]
        lower_diagonal[index] = before
        diagonal[index] = 2.0 * (before + after)
        upper_diagonal[index] = after
        right_hand_side[index] = 6.0 * (secants[index] - secants[index - 1])
    lower_diagonal[-1] = intervals[-1]
    diagonal[-1] = 2.0 * intervals[-1]
    right_hand_side[-1] = -6.0 * secants[-1]  # zero final velocity

    # Thomas algorithm: linear in the number of samples and all seven joints
    # share the same tridiagonal system.
    for index in range(1, count):
        factor = lower_diagonal[index] / diagonal[index - 1]
        diagonal[index] -= factor * upper_diagonal[index - 1]
        right_hand_side[index] -= factor * right_hand_side[index - 1]
    second_derivatives = np.empty_like(right_hand_side)
    second_derivatives[-1] = right_hand_side[-1] / diagonal[-1]
    for index in range(count - 2, -1, -1):
        second_derivatives[index] = (
            right_hand_side[index]
            - upper_diagonal[index] * second_derivatives[index + 1]
        ) / diagonal[index]

    coefficients = np.empty((count - 1, 4, values.shape[1]), dtype=float)
    for index, interval in enumerate(intervals):
        coefficients[index, 0] = values[index]
        coefficients[index, 1] = (
            secants[index]
            - interval * (2.0 * second_derivatives[index] + second_derivatives[index + 1]) / 6.0
        )
        coefficients[index, 2] = second_derivatives[index] / 2.0
        coefficients[index, 3] = (
            second_derivatives[index + 1] - second_derivatives[index]
        ) / (6.0 * interval)
    return coefficients


def _curve_dynamic_maxima(times: np.ndarray, coefficients: np.ndarray) -> tuple[float, float]:
    """Return exact maximum absolute velocity and acceleration of cubic segments."""
    maximum_velocity = 0.0
    maximum_acceleration = 0.0
    for index, coefficient in enumerate(coefficients):
        interval = float(times[index + 1] - times[index])
        _a, b, c, d = coefficient
        velocities = [b, b + 2.0 * c * interval + 3.0 * d * interval * interval]
        with np.errstate(divide="ignore", invalid="ignore"):
            critical = -c / (3.0 * d)
        inside = np.isfinite(critical) & (critical > 0.0) & (critical < interval)
        if np.any(inside):
            critical_velocity = np.zeros_like(b)
            local_critical = critical[inside]
            critical_velocity[inside] = (
                b[inside] + 2.0 * c[inside] * local_critical
                + 3.0 * d[inside] * local_critical * local_critical
            )
            velocities.append(critical_velocity)
        maximum_velocity = max(maximum_velocity, *(float(np.max(np.abs(v))) for v in velocities))
        acceleration_start = 2.0 * c
        acceleration_end = 2.0 * c + 6.0 * d * interval
        maximum_acceleration = max(
            maximum_acceleration,
            float(np.max(np.abs(acceleration_start))),
            float(np.max(np.abs(acceleration_end))),
        )
    return maximum_velocity, maximum_acceleration


def _curve_validation_times(
        times: np.ndarray, maximum_velocity: float, motor_speed: float) -> np.ndarray:
    if len(times) == 1 or times[-1] <= 0.0:
        return times.copy()
    interval = SMOOTH_PATH_CHECK_INTERVAL_S
    if maximum_velocity > 0.0:
        # Also limit the largest possible joint change between FK checks to the
        # same spatial resolution used by the existing linear path validator.
        interval = min(interval, motor_speed * PATH_PERIOD_S / maximum_velocity)
    estimated_checks = int(math.ceil(times[-1] / interval)) + len(times) + 1
    if estimated_checks > MAX_SMOOTH_PATH_CHECKS:
        raise ValueError(
            "Smoothed recording would require too many safe-zone checks; "
            "crop it into a shorter recording"
        )
    regular = np.arange(0.0, times[-1], interval)
    return np.unique(np.concatenate((regular, times, [times[-1]])))


def load_playback_trajectory(path, model, zone, lower, upper, center_goal, speed,
                             progress=None, playback_speed=None) -> PlaybackTrajectory:
    """Validate a recording and every commanded/representative recovery path."""
    def report(detail: str) -> None:
        if progress is not None:
            progress(detail)

    playback_speed = float(speed if playback_speed is None else playback_speed)
    if not math.isfinite(playback_speed) or playback_speed <= 0.0:
        raise ValueError("Playback speed must be positive and finite")

    report("Reading JSON and checking the recording format")
    source = Path(path).expanduser().resolve()
    if source.suffix.lower() != ".json":
        raise ValueError("Select a .json right-arm recording")
    if not source.is_file():
        raise ValueError("Recording file does not exist")
    if source.stat().st_size > MAX_RECORDING_BYTES:
        raise ValueError("Recording file is too large")
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("Recording JSON could not be read: " + str(exc)) from None
    if not isinstance(payload, dict) or payload.get("schema") != SCHEMA:
        raise ValueError(f"Recording must use schema {SCHEMA}")
    if payload.get("arm") != "right":
        raise ValueError("Recording is not for the right arm")
    if payload.get("model_sha256") != model.digest:
        raise ValueError("Recording robot-model hash does not match this workspace")
    if payload.get("joint_order") != list(JOINT_NAMES) or payload.get("gripper_name") != GRIPPER_NAME:
        raise ValueError("Recording joint order is incompatible")
    expected_units = {
        "time_unit": "s",
        "joint_position_unit": "rad",
        "tcp_position_unit": "m",
        "gripper_opening_unit": "m",
    }
    if any(payload.get(key) != value for key, value in expected_units.items()):
        raise ValueError("Recording units are incompatible")
    samples = payload.get("samples")
    if (not isinstance(samples, list) or not 1 <= len(samples) <= MAX_SAMPLES
            or payload.get("sample_count") != len(samples)):
        raise ValueError("Recording sample count is invalid")

    report(f"Parsing {len(samples)} recorded samples")
    times = []
    positions = []
    stored_tcp = []
    grippers = []
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise ValueError(f"Recording sample {index} is invalid")
        stamp = _finite_float(sample.get("time_s"), f"sample {index} time")
        raw_positions = sample.get("positions_rad")
        raw_tcp = sample.get("tcp_position_m")
        if not isinstance(raw_positions, list) or len(raw_positions) != 7:
            raise ValueError(f"Recording sample {index} must contain seven joint positions")
        if not isinstance(raw_tcp, list) or len(raw_tcp) != 3:
            raise ValueError(f"Recording sample {index} must contain a three-value TCP position")
        times.append(stamp)
        positions.append([
            _finite_float(value, f"sample {index} joint {joint + 1}")
            for joint, value in enumerate(raw_positions)
        ])
        stored_tcp.append([
            _finite_float(value, f"sample {index} TCP") for value in raw_tcp
        ])
        grippers.append(_finite_float(
            sample.get("gripper_opening_m"), f"sample {index} gripper opening"
        ))

    times = np.asarray(times, dtype=float)
    joints = np.asarray(positions, dtype=float)
    stored_tcp = np.asarray(stored_tcp, dtype=float)
    grippers = np.asarray(grippers, dtype=float)
    if abs(times[0]) > 1e-6 or (len(times) > 1 and np.any(np.diff(times) <= 0.0)):
        raise ValueError("Recording sample times must start at zero and strictly increase")
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    below = lower - joints
    above = joints - upper
    outside = np.maximum(np.maximum(below, above), 0.0)
    worst = np.unravel_index(int(np.argmax(outside)), outside.shape)
    if outside[worst] > LIMIT_TOLERANCE_RAD:
        raise ValueError(
            f"Recording sample {worst[0]} J{worst[1] + 1} exceeds its limit by "
            f"{math.degrees(outside[worst]):.4f} deg"
        )
    joints = np.clip(joints, lower, upper)
    if np.any(grippers < 0.0) or np.any(grippers > .044):
        raise ValueError("Recording gripper opening is outside 0 to 44 mm")

    report(
        f"Recalculating {len(joints)} recorded TCP positions and checking right zone1"
    )
    tcp_positions = []
    for index, joint_values in enumerate(joints):
        values = {f"openarmx_right_joint{i + 1}": float(joint_values[i]) for i in range(7)}
        tcp = model.transforms(values)[RIGHT_TCP][:3, 3]
        if not zone.contains(tcp, MEMBERSHIP_BUFFER_M):
            raise ValueError(f"Recording sample {index} leaves right zone1")
        if np.linalg.norm(tcp - stored_tcp[index]) > .002:
            raise ValueError(f"Recording sample {index} TCP metadata does not match its joints")
        tcp_positions.append(tcp)
    tcp_positions = np.asarray(tcp_positions)

    report("Fitting a smooth best-fit joint trajectory")
    fitted_joints = _locally_smooth_joints(times, joints)
    curve_coefficients = _clamped_cubic_coefficients(times, fitted_joints)
    maximum_velocity, _maximum_acceleration = _curve_dynamic_maxima(
        times, curve_coefficients
    )
    provisional = PlaybackTrajectory(
        source=source,
        joints=joints,
        source_times=times,
        tcp_positions=tcp_positions,
        gripper_openings=grippers,
        fitted_joints=fitted_joints,
        curve_coefficients=curve_coefficients,
        time_scale=1.0,
    )
    validation_times = _curve_validation_times(times, maximum_velocity, float(speed))
    curve_joints = np.asarray([
        provisional.joints_at_source_time(stamp) for stamp in validation_times
    ])
    below = lower - curve_joints
    above = curve_joints - upper
    outside = np.maximum(np.maximum(below, above), 0.0)
    worst = np.unravel_index(int(np.argmax(outside)), outside.shape)
    if outside[worst] > 1e-9:
        raise ValueError(
            f"Smoothed playback path near {validation_times[worst[0]]:.3f} s "
            f"makes J{worst[1] + 1} exceed its limit by "
            f"{math.degrees(outside[worst]):.4f} deg"
        )

    center_goal = np.asarray(center_goal, dtype=float)
    report(
        f"Checking center-to-start and {len(curve_joints)} points on the smooth path"
    )
    if not _path_inside(model, zone, center_goal, joints[0], speed):
        raise ValueError("Center-to-recording-start path leaves right zone1")
    for index, joint_values in enumerate(curve_joints):
        values = {
            f"openarmx_right_joint{joint + 1}": float(joint_values[joint])
            for joint in range(7)
        }
        tcp = model.transforms(values)[RIGHT_TCP][:3, 3]
        if not zone.contains(tcp, MEMBERSHIP_BUFFER_M):
            raise ValueError(
                f"Smoothed playback path leaves right zone1 near "
                f"{validation_times[index]:.3f} s"
            )
    report("Checking the recording endpoint return to center")
    if not _path_inside(model, zone, joints[-1], center_goal, speed):
        raise ValueError("Recording-end-to-center path leaves right zone1")

    # Fault recovery uses the same direct center command as the existing camera
    # program. Check every pose for ordinary recordings and a bounded, evenly
    # spaced set for very long files.
    anchor_count = min(len(curve_joints), RETURN_CHECK_ANCHORS)
    anchors = np.unique(np.linspace(0, len(curve_joints) - 1, anchor_count, dtype=int))
    report(
        f"Checking {len(anchors)} representative recovery-to-center paths"
    )
    for index in anchors:
        if not _path_inside(model, zone, curve_joints[index], center_goal, speed):
            raise ValueError(
                f"Center return from smooth playback time "
                f"{validation_times[index]:.3f} s leaves right zone1"
            )

    report("Calculating velocity-only playback timing")
    time_scale = max(
        1.0,
        maximum_velocity / (playback_speed * PLAYBACK_SPEED_FRACTION),
    )
    return PlaybackTrajectory(
        source=source,
        joints=joints,
        source_times=times,
        tcp_positions=tcp_positions,
        gripper_openings=grippers,
        fitted_joints=fitted_joints,
        curve_coefficients=curve_coefficients,
        time_scale=time_scale,
    )


class PlaybackFollower:
    """Monitor dynamic playback references without inventing off-path corrections."""

    def __init__(self):
        self.history = deque()
        self.tracking_error_since = None

    def update(self, actual, desired, now: float) -> np.ndarray:
        actual = np.asarray(actual, dtype=float)
        desired = np.asarray(desired, dtype=float)
        if (actual.shape != (7,) or desired.shape != (7,)
                or not np.isfinite(actual).all() or not np.isfinite(desired).all()):
            raise RuntimeError("Invalid playback joint state")
        error = desired - actual
        self.history.append((float(now), actual.copy()))
        while self.history and now - self.history[0][0] > STALL_SECONDS:
            self.history.popleft()
        if self.history and now - self.history[0][0] >= STALL_SECONDS - .05:
            span = np.ptp(np.array([joints for _, joints in self.history]), axis=0)
            stalled = (np.abs(error) > STALL_ERROR) & (span < STALL_MOVEMENT)
            if np.any(stalled):
                raise RuntimeError("STALL: motor " + str(int(np.flatnonzero(stalled)[0]) + 1))
        if np.max(np.abs(error)) > TRACKING_ERROR_RAD:
            if self.tracking_error_since is None:
                self.tracking_error_since = now
            elif now - self.tracking_error_since >= TRACKING_ERROR_SECONDS:
                raise RuntimeError("Recorded-path tracking error exceeded 15 degrees for 1 second")
        else:
            self.tracking_error_since = None
        return desired.copy()
