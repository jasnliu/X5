"""Hill-climber state with a playback-derived first-choice direction order."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from camera_search.hill_climb import HILL_DIRECTIONS, HILL_STEPS_M


_DIRECTION_BY_NAME = {name: vector for name, vector in HILL_DIRECTIONS}


@dataclass
class GuidedHillClimber:
    anchor_offset: np.ndarray
    anchor_joints: np.ndarray
    direction_order: tuple[str, ...]
    anchor_score: float | None = None
    step_index: int = 0
    direction_index: int = 0
    improved_in_cycle: bool = False

    def __post_init__(self):
        self.anchor_offset = np.asarray(self.anchor_offset, dtype=float).copy()
        self.anchor_joints = np.asarray(self.anchor_joints, dtype=float).copy()
        self.direction_order = tuple(self.direction_order)
        if self.anchor_offset.shape != (3,) or self.anchor_joints.shape != (7,):
            raise ValueError("Invalid guided hill-climber anchor")
        if (len(self.direction_order) != len(_DIRECTION_BY_NAME)
                or set(self.direction_order) != set(_DIRECTION_BY_NAME)):
            raise ValueError("Guided direction order must contain all six Cartesian directions")

    @property
    def step(self) -> float:
        return HILL_STEPS_M[self.step_index]

    @property
    def direction_name(self) -> str:
        return self.direction_order[self.direction_index]

    def candidate_offset(self) -> np.ndarray:
        return self.anchor_offset + _DIRECTION_BY_NAME[self.direction_name] * self.step

    def accepts(self, score: float) -> bool:
        if self.anchor_score is None:
            raise ValueError("Anchor must be scored before testing candidates")
        return float(score) < float(self.anchor_score)

    def accept(self, offset, joints, score: float) -> None:
        self.anchor_offset = np.asarray(offset, dtype=float).copy()
        self.anchor_joints = np.asarray(joints, dtype=float).copy()
        self.anchor_score = float(score)
        self.improved_in_cycle = True

    def reject_or_skip(self) -> bool:
        self.direction_index += 1
        if self.direction_index < len(self.direction_order):
            return True
        self.direction_index = 0
        if self.improved_in_cycle:
            self.improved_in_cycle = False
            return True
        if self.step_index + 1 < len(HILL_STEPS_M):
            self.step_index += 1
            return True
        return False
