"""Validated loading, preview interpolation, cropping, and in-place saving.

This module is deliberately independent of Tk and ROS so the destructive part
of the editor can be tested without opening either UI window.
"""
from __future__ import annotations

from bisect import bisect_left, bisect_right
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import json
import math
import os
from pathlib import Path
import stat
import tempfile

from .recording import MAX_SAMPLES, SCHEMAS, gripper_name, joint_names
from safe_zone.gripper_feedback import motor8_feedback


MAX_RECORDING_BYTES = 80_000_000


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


def _shift_utc_timestamp(value: str, seconds: float) -> str:
    """Shift the recorder's ISO-8601 UTC timestamp, retaining millisecond form."""
    if not isinstance(value, str) or not value:
        raise ValueError("Recording start time is invalid")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        raise ValueError("Recording start time is invalid") from None
    if parsed.tzinfo is None:
        raise ValueError("Recording start time must include a UTC offset")
    shifted = parsed.astimezone(timezone.utc) + timedelta(seconds=float(seconds))
    return shifted.isoformat(timespec="milliseconds").replace("+00:00", "Z")


@dataclass(frozen=True)
class PreviewState:
    """Interpolated visual-only state at one source time."""

    time_s: float
    positions_rad: tuple[float, ...]
    gripper_opening_m: float
    tcp_position_m: tuple[float, ...]


class EditableRecording:
    """A validated recorder JSON file with sample-aligned crop operations."""

    def __init__(self, source: Path, payload: dict, times: tuple[float, ...], arm: str):
        self.source = source
        self.payload = payload
        self.samples = payload["samples"]
        self.times = times
        self.arm = arm
        self.joint_names = joint_names(arm)
        self.gripper_name = gripper_name(arm)

    @classmethod
    def load(cls, path, model_sha256: str) -> "EditableRecording":
        source = Path(path).expanduser().resolve()
        if source.suffix.lower() != ".json":
            raise ValueError("Select a .json arm recording")
        if not source.is_file():
            raise ValueError("Recording file does not exist")
        if source.stat().st_size > MAX_RECORDING_BYTES:
            raise ValueError("Recording file is too large")
        try:
            payload = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("Recording JSON could not be read: " + str(exc)) from None

        arm = payload.get("arm") if isinstance(payload, dict) else None
        if arm not in SCHEMAS or payload.get("schema") != SCHEMAS[arm]:
            choices = " or ".join(SCHEMAS.values())
            raise ValueError(f"Recording must use schema {choices}")
        names = joint_names(arm)
        gripper = gripper_name(arm)
        if payload.get("model_sha256") != model_sha256:
            raise ValueError("Recording robot-model hash does not match this workspace")
        if (payload.get("joint_order") != list(names)
                or payload.get("gripper_name") != gripper):
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

        times = []
        for index, sample in enumerate(samples):
            if not isinstance(sample, dict):
                raise ValueError(f"Recording sample {index} is invalid")
            stamp = _finite_float(sample.get("time_s"), f"sample {index} time")
            positions = sample.get("positions_rad")
            tcp = sample.get("tcp_position_m")
            if not isinstance(positions, list) or len(positions) != len(names):
                raise ValueError(f"Recording sample {index} must contain seven joint positions")
            if not isinstance(tcp, list) or len(tcp) != 3:
                raise ValueError(f"Recording sample {index} must contain a three-value TCP position")
            for joint, value in enumerate(positions):
                _finite_float(value, f"sample {index} joint {joint + 1}")
            for value in tcp:
                _finite_float(value, f"sample {index} TCP")
            gripper = _finite_float(
                sample.get("gripper_opening_m"), f"sample {index} gripper opening"
            )
            if gripper < 0.0 or gripper > .044:
                raise ValueError("Recording gripper opening is outside 0 to 44 mm")
            if 'motor8_encoder_count' in sample or 'motor8_raw_rad' in sample:
                raw = motor8_feedback(sample.get('motor8_encoder_count'))
                if _finite_float(sample.get('motor8_raw_rad'), 'motor 8 raw angle') != raw['raw_rad']:
                    raise ValueError('Motor 8 raw angle does not match the recorded wire count')
            times.append(stamp)

        if abs(times[0]) > 1e-6 or any(
                current <= previous for previous, current in zip(times, times[1:])):
            raise ValueError("Recording sample times must start at zero and strictly increase")
        duration = _finite_float(payload.get("duration_s"), "recording duration")
        if abs(duration - times[-1]) > 1e-5:
            raise ValueError("Recording duration does not match its final sample")
        _shift_utc_timestamp(payload.get("started_at_utc"), 0.0)
        return cls(source, payload, tuple(times), arm)

    @property
    def sample_count(self) -> int:
        return len(self.samples)

    @property
    def duration_s(self) -> float:
        return self.times[-1]

    def nearest_index(self, time_s: float, minimum: int = 0,
                      maximum: int | None = None) -> int:
        """Return the closest real sample, constrained to the supplied range."""
        maximum = self.sample_count - 1 if maximum is None else maximum
        if not (0 <= minimum <= maximum < self.sample_count):
            raise ValueError("Invalid sample-index range")
        target = min(max(float(time_s), self.times[minimum]), self.times[maximum])
        position = bisect_left(self.times, target, minimum, maximum + 1)
        if position <= minimum:
            return minimum
        if position > maximum:
            return maximum
        before = position - 1
        if target - self.times[before] <= self.times[position] - target:
            return before
        return position

    def state_at(self, time_s: float) -> PreviewState:
        """Linearly interpolate between samples for smooth RViz-only playback."""
        target = min(max(float(time_s), 0.0), self.duration_s)
        upper = bisect_right(self.times, target)
        if upper == 0:
            lower = upper = 0
        elif upper >= self.sample_count:
            lower = upper = self.sample_count - 1
        else:
            lower = upper - 1
        first = self.samples[lower]
        if lower == upper:
            fraction = 0.0
            second = first
        else:
            second = self.samples[upper]
            span = self.times[upper] - self.times[lower]
            fraction = (target - self.times[lower]) / span

        def interpolate(left, right):
            return float(left) + fraction * (float(right) - float(left))

        positions = tuple(interpolate(a, b) for a, b in zip(
            first["positions_rad"], second["positions_rad"]
        ))
        tcp = tuple(interpolate(a, b) for a, b in zip(
            first["tcp_position_m"], second["tcp_position_m"]
        ))
        gripper = interpolate(
            first["gripper_opening_m"], second["gripper_opening_m"]
        )
        return PreviewState(target, positions, gripper, tcp)

    def raw_gripper_at(self, time_s):
        """Exact nearest recorded sample, never an interpolated width estimate."""
        sample = self.samples[self.nearest_index(time_s)]
        if 'motor8_encoder_count' not in sample:
            return None  # Clipped legacy widths cannot recover raw feedback.
        return motor8_feedback(sample['motor8_encoder_count'])

    def cropped_payload(self, start_index: int, end_index: int) -> dict:
        """Build a schema-compatible payload retaining both boundary samples."""
        if not (0 <= start_index <= end_index < self.sample_count):
            raise ValueError("Crop boundaries are invalid")
        result = deepcopy(self.payload)
        base_time = self.times[start_index]
        kept = deepcopy(self.samples[start_index:end_index + 1])
        for sample, original_time in zip(kept, self.times[start_index:end_index + 1]):
            sample["time_s"] = round(original_time - base_time, 6)
        if any(current["time_s"] <= previous["time_s"]
               for previous, current in zip(kept, kept[1:])):
            raise ValueError("Cropped sample times lost their strict ordering")
        result["samples"] = kept
        result["sample_count"] = len(kept)
        result["duration_s"] = kept[-1]["time_s"]
        result["started_at_utc"] = _shift_utc_timestamp(
            self.payload["started_at_utc"], base_time
        )
        return result

    def overwrite_crop(self, start_index: int, end_index: int) -> Path:
        """Atomically replace the selected source path; never create a new recording."""
        payload = self.cropped_payload(start_index, end_index)
        destination = self.source
        original_mode = stat.S_IMODE(destination.stat().st_mode)
        fd, temporary = tempfile.mkstemp(
            dir=destination.parent, prefix=f".{self.arm}-motion-edit-", suffix=".tmp"
        )
        try:
            os.fchmod(fd, original_mode)
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, allow_nan=False)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, destination)
            return destination
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
