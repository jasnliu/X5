"""Validated, atomic JSON persistence for one-arm encoder trajectories."""
from __future__ import annotations

from datetime import datetime, timezone
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile
from safe_zone.gripper_feedback import motor8_feedback


SCHEMAS = {
    "left": "openarmx-left-motion-recording-v1",
    "right": "openarmx-right-motion-recording-v1",
}


def joint_names(side: str) -> tuple[str, ...]:
    if side not in SCHEMAS:
        raise ValueError("Arm side must be left or right")
    return tuple(f"openarmx_{side}_joint{i}" for i in range(1, 8))


def gripper_name(side: str) -> str:
    if side not in SCHEMAS:
        raise ValueError("Arm side must be left or right")
    return f"openarmx_{side}_finger_joint1"


# Backward-compatible right-arm names used by the right-arm playback programs.
SCHEMA = SCHEMAS["right"]
JOINT_NAMES = joint_names("right")
GRIPPER_NAME = gripper_name("right")
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


def joint_limit_violation(state, lower, upper, names=JOINT_NAMES) -> JointLimitViolation | None:
    """Return the first selected-arm joint outside its replayable URDF range."""
    if len(lower) != len(names) or len(upper) != len(names):
        raise ValueError("Seven arm joint limits are required")
    for index, name in enumerate(names):
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


def default_filename(now: datetime | None = None, side: str = "right") -> str:
    if side not in SCHEMAS:
        raise ValueError("Arm side must be left or right")
    now = now or datetime.now()
    return now.strftime(f"{side}_motion_%Y%m%d_%H%M%S.json")


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
    """An in-memory, timestamped one-arm trajectory ready for JSON export."""

    def __init__(self, model_sha256: str, side: str = "right"):
        if not isinstance(model_sha256, str) or len(model_sha256) != 64:
            raise ValueError("A SHA-256 robot-model digest is required")
        if side not in SCHEMAS:
            raise ValueError("Arm side must be left or right")
        self.model_sha256 = model_sha256
        self.side = side
        self.joint_names = joint_names(side)
        self.gripper_name = gripper_name(side)
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
        positions = [float(state[name]) for name in self.joint_names]
        gripper = float(state[self.gripper_name])
        tcp = [float(value) for value in tcp_position_m]
        if len(tcp) != 3 or not all(math.isfinite(value) for value in positions + [gripper] + tcp):
            raise ValueError("Recording sample contains invalid values")
        raw = getattr(state, 'motor8_feedback', {}).get(self.side)
        if raw is not None:
            raw = motor8_feedback(raw['encoder_count'])
        if self._last_stamp is not None and stamp <= self._last_stamp:
            raise ValueError("Recording sample times must increase")
        if self._first_stamp is None:
            self._first_stamp = stamp
        self._last_stamp = stamp
        sample = {
            "time_s": round(stamp - self._first_stamp, 6),
            "positions_rad": positions,
            "gripper_opening_m": gripper,
            "tcp_position_m": tcp,
        }
        if raw is not None:
            sample['motor8_encoder_count'] = raw['encoder_count']
            sample['motor8_raw_rad'] = raw['raw_rad']
        self.samples.append(sample)

    def stop(self) -> None:
        self.active = False

    def as_dict(self) -> dict:
        if not self.samples:
            raise ValueError("The recording has no samples")
        return {
            "schema": SCHEMAS[self.side],
            "arm": self.side,
            "source": "query-only disabled-motor encoder feedback",
            "model_sha256": self.model_sha256,
            "time_unit": "s",
            "joint_position_unit": "rad",
            "tcp_position_unit": "m",
            "gripper_opening_unit": "m",
            "joint_order": list(self.joint_names),
            "gripper_name": self.gripper_name,
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
            dir=destination.parent, prefix=f".{self.side}-motion-", suffix=".tmp"
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
