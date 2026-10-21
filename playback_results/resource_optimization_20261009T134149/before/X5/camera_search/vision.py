"""Pure helpers for selecting and scoring camera detections."""
from __future__ import annotations

from dataclasses import dataclass
import math
import statistics
from typing import Any, Iterable


CYMBAL_CLASS_ID = 0
DRUMSTICK_CLASS_ID = 1


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def valid_point(value: Any) -> tuple[float, float] | None:
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or not all(_finite_number(item) for item in value)):
        return None
    return float(value[0]), float(value[1])


def valid_box(value: Any) -> tuple[float, float, float, float] | None:
    if (not isinstance(value, (list, tuple)) or len(value) != 4
            or not all(_finite_number(item) for item in value)):
        return None
    left, top, right, bottom = map(float, value)
    if right <= left or bottom <= top:
        return None
    return left, top, right, bottom


def _confidence(detection: dict[str, Any]) -> float:
    value = detection.get("confidence", 0.0)
    return float(value) if _finite_number(value) else 0.0


def select_cymbal(detections: Iterable[dict[str, Any]]) -> dict[str, Any] | None:
    candidates = [
        detection for detection in detections
        if detection.get("class_id") == CYMBAL_CLASS_ID
        and valid_box(detection.get("box")) is not None
    ]
    return max(candidates, key=_confidence) if candidates else None


def select_observed_stick(detections: Iterable[dict[str, Any]]) -> dict[str, Any] | None:
    """Choose the strongest class-1 object with its directly observed YOLO tip."""
    candidates = [
        detection for detection in detections
        if detection.get("class_id") == DRUMSTICK_CLASS_ID
        and valid_box(detection.get("box")) is not None
        and valid_point(detection.get("tip")) is not None
        and detection.get("tip_source") == "yolo_pose"
        and detection.get("tip_status") == "observed"
    ]
    return max(candidates, key=_confidence) if candidates else None


def cymbal_grid_geometry(detections: Iterable[dict[str, Any]]) -> dict[str, tuple[int, ...]] | None:
    """Return equal thirds and the center ninth of the strongest cymbal box."""
    detection = select_cymbal(detections)
    if detection is None:
        return None
    box = valid_box(detection.get("box"))
    assert box is not None
    left, top, right, bottom = map(lambda value: int(round(value)), box)
    if right <= left or bottom <= top:
        return None
    x_lines = (
        int(round(left + (right - left) / 3.0)),
        int(round(left + 2.0 * (right - left) / 3.0)),
    )
    y_lines = (
        int(round(top + (bottom - top) / 3.0)),
        int(round(top + 2.0 * (bottom - top) / 3.0)),
    )
    return {
        "outer": (left, top, right, bottom),
        "x_lines": x_lines,
        "y_lines": y_lines,
        "target": (x_lines[0], y_lines[0], x_lines[1], y_lines[1]),
    }


@dataclass(frozen=True)
class VisualObservation:
    tip: tuple[float, float]
    cymbal_box: tuple[float, float, float, float]
    target_box: tuple[float, float, float, float]


def observation_from_message(message: dict[str, Any]) -> VisualObservation | None:
    """Extract a directly observed stick tip and simultaneous cymbal target."""
    if message.get("kind") != "frame" or message.get("required") is not True:
        return None
    tip = valid_point(message.get("tip_point"))
    cymbal_box = valid_box(message.get("cymbal_box"))
    target_box = valid_box(message.get("target_box"))
    if tip is None or cymbal_box is None or target_box is None:
        return None
    return VisualObservation(tip, cymbal_box, target_box)


def median_observation(samples: Iterable[VisualObservation]) -> VisualObservation:
    """Component-wise median suppresses short-lived box and keypoint jitter."""
    values = tuple(samples)
    if not values:
        raise ValueError("At least one valid visual observation is required")

    def median(items):
        return float(statistics.median(items))

    return VisualObservation(
        tuple(median(sample.tip[index] for sample in values) for index in range(2)),
        tuple(median(sample.cymbal_box[index] for sample in values) for index in range(4)),
        tuple(median(sample.target_box[index] for sample in values) for index in range(4)),
    )


def normalized_target_distance(observation: VisualObservation) -> float:
    """Distance from tip to center cell, normalized by full cymbal box size."""
    x, y = observation.tip
    left, top, right, bottom = observation.target_box
    cymbal_left, cymbal_top, cymbal_right, cymbal_bottom = observation.cymbal_box
    width = cymbal_right - cymbal_left
    height = cymbal_bottom - cymbal_top
    if width <= 0.0 or height <= 0.0:
        raise ValueError("Cymbal box must have positive dimensions")
    dx = left - x if x < left else x - right if x > right else 0.0
    dy = top - y if y < top else y - bottom if y > bottom else 0.0
    return math.hypot(dx / width, dy / height)


def median_target_distance(samples: Iterable[VisualObservation]) -> float:
    return normalized_target_distance(median_observation(samples))
