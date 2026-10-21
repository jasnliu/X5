"""Visible pink-target checks and playback-derived Cartesian direction prior."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from camera_search.hill_climb import HILL_DIRECTIONS
from camera_search.vision import VisualObservation, median_observation


PRIOR_WINDOW_SECONDS = 2.0
MIN_PRIOR_SAMPLES = 4
MIN_PRIOR_MOTION_M = .003
MIN_PRIOR_SCORE_RANGE = .005
DEFAULT_DIRECTION_ORDER = tuple(name for name, _ in HILL_DIRECTIONS)


def point_inside_box(point, box) -> bool:
    x, y = point
    left, top, right, bottom = box
    return left <= x <= right and top <= y <= bottom


def inside_outer_goal(observation: VisualObservation) -> bool:
    return point_inside_box(observation.tip, observation.target_box)


def normalized_center_distance(observation: VisualObservation) -> float:
    """Smooth score toward the center, including while inside the outer box."""
    x, y = observation.tip
    left, top, right, bottom = observation.target_box
    cymbal_left, cymbal_top, cymbal_right, cymbal_bottom = observation.cymbal_box
    width = cymbal_right - cymbal_left
    height = cymbal_bottom - cymbal_top
    if width <= 0.0 or height <= 0.0:
        raise ValueError("Cymbal box must have positive dimensions")
    center_x = (left + right) / 2.0
    center_y = (top + bottom) / 2.0
    return math.hypot((x - center_x) / width, (y - center_y) / height)


def median_outer_result(samples) -> tuple[VisualObservation, float, bool]:
    """Return robust center score and visible-pink-zone membership."""
    observation = median_observation(samples)
    return observation, normalized_center_distance(observation), inside_outer_goal(observation)


@dataclass(frozen=True)
class PlaybackVisualSample:
    time_s: float
    tcp_offset_m: np.ndarray
    center_score: float


def infer_direction_order(samples) -> tuple[tuple[str, ...], str]:
    """Rank all six directions; learned evidence reorders but never removes one."""
    values = tuple(samples)
    if len(values) < MIN_PRIOR_SAMPLES:
        return DEFAULT_DIRECTION_ORDER, "standard direction order (insufficient playback observations)"
    end_time = values[-1].time_s
    recent = tuple(value for value in values if value.time_s >= end_time - PRIOR_WINDOW_SECONDS)
    if len(recent) < MIN_PRIOR_SAMPLES:
        recent = values[-MIN_PRIOR_SAMPLES:]
    positions = np.asarray([value.tcp_offset_m for value in recent], dtype=float)
    scores = np.asarray([value.center_score for value in recent], dtype=float)
    if (positions.ndim != 2 or positions.shape[1] != 3
            or not np.isfinite(positions).all() or not np.isfinite(scores).all()):
        return DEFAULT_DIRECTION_ORDER, "standard direction order (invalid playback evidence)"
    if np.max(np.ptp(positions, axis=0)) < MIN_PRIOR_MOTION_M:
        return DEFAULT_DIRECTION_ORDER, "standard direction order (too little ending motion)"
    if np.ptp(scores) < MIN_PRIOR_SCORE_RANGE:
        return DEFAULT_DIRECTION_ORDER, "standard direction order (camera score was nearly unchanged)"
    delta_position = np.diff(positions, axis=0)
    delta_score = np.diff(scores)
    moving = np.linalg.norm(delta_position, axis=1) >= .0005
    if np.count_nonzero(moving) < 2:
        return DEFAULT_DIRECTION_ORDER, "standard direction order (too little paired motion)"
    gradient, *_ = np.linalg.lstsq(delta_position[moving], delta_score[moving], rcond=None)
    predictions = {
        name: float(gradient @ vector) for name, vector in HILL_DIRECTIONS
    }
    original_rank = {name: index for index, name in enumerate(DEFAULT_DIRECTION_ORDER)}
    order = tuple(sorted(
        DEFAULT_DIRECTION_ORDER,
        key=lambda name: (predictions[name], original_rank[name]),
    ))
    detail = "playback-guided order " + ", ".join(order)
    return order, detail
