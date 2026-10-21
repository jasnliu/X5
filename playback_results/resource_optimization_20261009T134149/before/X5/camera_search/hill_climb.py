"""Deterministic Cartesian coordinate hill-climber state."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


HILL_STEPS_M = (0.010, 0.005, 0.0025)
HILL_DIRECTIONS = (
    ("+X", np.array([1.0, 0.0, 0.0])),
    ("-X", np.array([-1.0, 0.0, 0.0])),
    ("+Y", np.array([0.0, 1.0, 0.0])),
    ("-Y", np.array([0.0, -1.0, 0.0])),
    ("+Z", np.array([0.0, 0.0, 1.0])),
    ("-Z", np.array([0.0, 0.0, -1.0])),
)


@dataclass
class HillClimber:
    anchor_offset: np.ndarray
    anchor_joints: np.ndarray
    anchor_score: float | None = None
    step_index: int = 0
    direction_index: int = 0
    improved_in_cycle: bool = False

    def __post_init__(self):
        self.anchor_offset = np.asarray(self.anchor_offset, dtype=float).copy()
        self.anchor_joints = np.asarray(self.anchor_joints, dtype=float).copy()
        if self.anchor_offset.shape != (3,) or self.anchor_joints.shape != (7,):
            raise ValueError("Invalid hill-climber anchor")

    @property
    def step(self) -> float:
        return HILL_STEPS_M[self.step_index]

    @property
    def direction_name(self) -> str:
        return HILL_DIRECTIONS[self.direction_index][0]

    def candidate_offset(self) -> np.ndarray:
        return self.anchor_offset + HILL_DIRECTIONS[self.direction_index][1] * self.step

    def accepts(self, score: float) -> bool:
        if self.anchor_score is None:
            raise ValueError("Anchor must be scored before testing candidates")
        return float(score) < float(self.anchor_score)

    def accept(self, offset, joints, score: float) -> None:
        self.anchor_offset = np.asarray(offset, dtype=float).copy()
        self.anchor_joints = np.asarray(joints, dtype=float).copy()
        self.anchor_score = float(score)
        self.improved_in_cycle = True
        # Keep the same direction until it stops improving.

    def reject_or_skip(self) -> bool:
        """Advance deterministically; return False only at the final local minimum."""
        self.direction_index += 1
        if self.direction_index < len(HILL_DIRECTIONS):
            return True
        self.direction_index = 0
        if self.improved_in_cycle:
            self.improved_in_cycle = False
            return True
        if self.step_index + 1 < len(HILL_STEPS_M):
            self.step_index += 1
            return True
        return False
