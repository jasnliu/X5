"""Validated, atomic JSON persistence for right-arm encoder trajectories."""
from __future__ import annotations

from datetime import datetime, timezone
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile


SCHEMA = "openarmx-right-motion-recording-v1"
JOINT_NAMES = tuple(f"openarmx_right_joint{i}" for i in range(1, 8))
GRIPPER_NAME = "openarmx_right_finger_joint1"
# Query a fresh state batch every 20 ms when the CAN devices can keep up.  The
# recorder stores the actual monotonic timestamp of every completed batch, so a
# slow response never gets duplicated merely to satisfy this target rate.
NOMINAL_SAMPLE_RATE_HZ = 50.0
MAX_SAMPLES = 100_000
# Motor feedback spans 25.14 radians over 16 bits. Match the playback loader's
# three-count tolerance so a recording accepted here is not later rejected for
# harmless quantization at an exact URDF boundary.
ENCODER_LSB_RAD = 25.14 / 65535
JOINT_LIMIT_TOLERANCE_RAD = 3.0 * ENCODER_LSB_RAD


@dataclass(frozen=True)
class JointLimitViolation:
    joint_number: int
    measured_rad: float
    limit_rad: float
    direction: str
    excess_rad: float


def joint_limit_violation(state, lower, upper) -> JointLimitViolation | None:
    """Return the first right-arm joint outside its replayable URDF range."""
    if len(lower) != len(JOINT_NAMES) or len(upper) != len(JOINT_NAMES):
        raise ValueError("Seven right-arm joint limits are required")
    for index, name in enumerate(JOINT_NAMES):
        value = float(state[name])
        low = float(lower[index])
        high = float(upper[index])
        if not all(math.isfinite(number) for number in (value, low, high)):
            raise ValueError("Joint state or limit is not finite")
        if value < low - JOINT_LIMIT_TOLERANCE_RAD:
            return JointLimitViolation(index + 1, value, low, "below", low - value)
        if value > high + JOINT_LIMIT_TOLERANCE_RAD:
            return JointLimitViolation(index + 1, value, high, "above", value - high)
    return None


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def default_filename(now: datetime | None = None) -> str:
    now = now or datetime.now()
    return now.strftime("right_motion_%Y%m%d_%H%M%S.json")


def confined_json_path(recordings_dir, selected) -> Path:
    """Force a dialog selection into the recorder's own folder."""
    directory = Path(recordings_dir).expanduser().resolve()
    name = Path(selected).name
    if not name:
        raise ValueError("A recording filename is required")
    path = directory / name
    if path.suffix.lower() != ".json":
        path = path.with_name(path.name + ".json")
    return path


class MotionRecording:
    """An in-memory, timestamped right-arm trajectory ready for JSON export."""

    def __init__(self, model_sha256: str):
        if not isinstance(model_sha256, str) or len(model_sha256) != 64:
            raise ValueError("A SHA-256 robot-model digest is required")
        self.model_sha256 = model_sha256
        self.reset()

    def reset(self) -> None:
        self.samples = []
        self.active = False
        self.dirty = False
        self.started_at_utc = None
        self._first_stamp = None
        self._last_stamp = None

    @property
    def duration_s(self) -> float:
        if self._first_stamp is None or self._last_stamp is None:
            return 0.0
        return max(0.0, self._last_stamp - self._first_stamp)

    def start(self, started_at_utc: str | None = None) -> None:
        if self.active:
            raise RuntimeError("Recording is already active")
        self.reset()
        self.active = True
        self.dirty = True
        self.started_at_utc = started_at_utc or utc_now()

    def add(self, stamp: float, state, tcp_position_m) -> None:
        if not self.active:
            raise RuntimeError("Recording is not active")
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError("Sample time must be finite")
        if len(self.samples) >= MAX_SAMPLES:
            raise ValueError(f"Recording reached the {MAX_SAMPLES}-sample limit")
        positions = [float(state[name]) for name in JOINT_NAMES]
        gripper = float(state[GRIPPER_NAME])
        tcp = [float(value) for value in tcp_position_m]
        if len(tcp) != 3 or not all(math.isfinite(value) for value in positions + [gripper] + tcp):
            raise ValueError("Recording sample contains invalid values")
        if self._last_stamp is not None and stamp <= self._last_stamp:
            raise ValueError("Recording sample times must increase")
        if self._first_stamp is None:
            self._first_stamp = stamp
        self._last_stamp = stamp
        self.samples.append({
            "time_s": round(stamp - self._first_stamp, 6),
            "positions_rad": positions,
            "gripper_opening_m": gripper,
            "tcp_position_m": tcp,
        })

    def stop(self) -> None:
        self.active = False

    def as_dict(self) -> dict:
        if not self.samples:
            raise ValueError("The recording has no samples")
        return {
            "schema": SCHEMA,
            "arm": "right",
            "source": "query-only disabled-motor encoder feedback",
            "model_sha256": self.model_sha256,
            "time_unit": "s",
            "joint_position_unit": "rad",
            "tcp_position_unit": "m",
            "gripper_opening_unit": "m",
            "joint_order": list(JOINT_NAMES),
            "gripper_name": GRIPPER_NAME,
            "nominal_sample_rate_hz": NOMINAL_SAMPLE_RATE_HZ,
            "started_at_utc": self.started_at_utc,
            "duration_s": round(self.duration_s, 6),
            "sample_count": len(self.samples),
            "samples": self.samples,
        }

    def save(self, path) -> Path:
        if self.active:
            raise RuntimeError("Stop recording before saving")
        destination = Path(path).expanduser().resolve()
        if destination.suffix.lower() != ".json":
            raise ValueError("Motion recordings must use a .json filename")
        destination.parent.mkdir(parents=True, exist_ok=True)
        payload = self.as_dict()
        fd, temporary = tempfile.mkstemp(
            dir=destination.parent, prefix=".right-motion-", suffix=".tmp"
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, allow_nan=False)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, destination)
            self.dirty = False
            return destination
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
