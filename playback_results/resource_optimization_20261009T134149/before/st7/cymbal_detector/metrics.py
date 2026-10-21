from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class EventMetrics:
    true_positives: int = 0
    false_positives: int = 0
    false_negatives: int = 0
    duplicates: int = 0
    negative_seconds: float = 0.0
    onset_errors: list[float] | None = None

    def __post_init__(self) -> None:
        if self.onset_errors is None:
            self.onset_errors = []

    def add(self, other: "EventMetrics") -> None:
        self.true_positives += other.true_positives
        self.false_positives += other.false_positives
        self.false_negatives += other.false_negatives
        self.duplicates += other.duplicates
        self.negative_seconds += other.negative_seconds
        self.onset_errors.extend(other.onset_errors or [])

    @property
    def precision(self) -> float:
        denominator = self.true_positives + self.false_positives
        return self.true_positives / denominator if denominator else 1.0

    @property
    def recall(self) -> float:
        denominator = self.true_positives + self.false_negatives
        return self.true_positives / denominator if denominator else 1.0

    @property
    def f1(self) -> float:
        denominator = self.precision + self.recall
        return 2 * self.precision * self.recall / denominator if denominator else 0.0

    def as_dict(self) -> dict:
        errors = np.asarray(self.onset_errors or [], dtype=np.float64)
        return {
            "true_positives": self.true_positives,
            "false_positives": self.false_positives,
            "false_negatives": self.false_negatives,
            "duplicates": self.duplicates,
            "precision": self.precision,
            "recall": self.recall,
            "f1": self.f1,
            "onset_mae_seconds": float(np.mean(np.abs(errors))) if len(errors) else None,
            "onset_median_abs_seconds": float(np.median(np.abs(errors))) if len(errors) else None,
            "negative_seconds": self.negative_seconds,
            "false_positives_per_negative_minute": (
                self.false_positives / (self.negative_seconds / 60.0) if self.negative_seconds else None
            ),
        }


def match_events(
    truth: tuple[float, ...] | list[float],
    predicted: list[float],
    tolerance_seconds: float,
    duration: float,
) -> EventMetrics:
    candidates: list[tuple[float, int, int]] = []
    for truth_index, expected in enumerate(truth):
        for prediction_index, actual in enumerate(predicted):
            difference = abs(actual - expected)
            if difference <= tolerance_seconds:
                candidates.append((difference, truth_index, prediction_index))
    matched_truth: set[int] = set()
    matched_predictions: set[int] = set()
    errors: list[float] = []
    for _, truth_index, prediction_index in sorted(candidates):
        if truth_index in matched_truth or prediction_index in matched_predictions:
            continue
        matched_truth.add(truth_index)
        matched_predictions.add(prediction_index)
        errors.append(predicted[prediction_index] - truth[truth_index])

    unmatched_predictions = [
        value for index, value in enumerate(predicted) if index not in matched_predictions
    ]
    duplicates = 0
    for value in unmatched_predictions:
        if any(0.0 < value - expected <= 1.5 for expected in truth):
            duplicates += 1
    return EventMetrics(
        true_positives=len(matched_truth),
        false_positives=len(predicted) - len(matched_predictions),
        false_negatives=len(truth) - len(matched_truth),
        duplicates=duplicates,
        negative_seconds=duration if not truth else 0.0,
        onset_errors=errors,
    )

