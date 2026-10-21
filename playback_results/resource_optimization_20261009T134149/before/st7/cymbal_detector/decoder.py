from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DecoderConfig:
    high_threshold: float
    low_threshold: float
    min_gap_seconds: float
    smooth_frames: int = 1
    release_frames: int = 2
    max_peak_frames: int = 3

    def as_dict(self) -> dict:
        return {
            "high_threshold": self.high_threshold,
            "low_threshold": self.low_threshold,
            "min_gap_seconds": self.min_gap_seconds,
            "smooth_frames": self.smooth_frames,
            "release_frames": self.release_frames,
            "max_peak_frames": self.max_peak_frames,
        }

    @classmethod
    def from_dict(cls, value: dict) -> "DecoderConfig":
        return cls(**value)


def causal_smooth(values: np.ndarray, frames: int) -> np.ndarray:
    if frames <= 1:
        return values.astype(np.float64, copy=True)
    cumulative = np.concatenate(([0.0], np.cumsum(values, dtype=np.float64)))
    output = np.empty(len(values), dtype=np.float64)
    for index in range(len(values)):
        start = max(0, index - frames + 1)
        output[index] = (cumulative[index + 1] - cumulative[start]) / (index - start + 1)
    return output


class EventDecoder:
    """Hysteretic peak-and-latch decoder that emits at most one event per peak."""

    def __init__(self, config: DecoderConfig, frame_hop_seconds: float = 0.01) -> None:
        self.config = config
        self.frame_hop_seconds = frame_hop_seconds
        self.reset()

    def reset(self) -> None:
        self.state = "armed"
        self.peak_index = -1
        self.peak_score = 0.0
        self.peak_age = 0
        self.release_count = 0
        self.last_event_time = -float("inf")

    def feed(self, score: float, frame_index: int) -> tuple[float, float] | None:
        cfg = self.config
        time_seconds = frame_index * self.frame_hop_seconds
        if self.state == "armed":
            if score >= cfg.high_threshold and time_seconds - self.last_event_time >= cfg.min_gap_seconds:
                self.state = "peak"
                self.peak_index = frame_index
                self.peak_score = score
                self.peak_age = 0
            return None

        if self.state == "peak":
            self.peak_age += 1
            if score > self.peak_score:
                self.peak_index = frame_index
                self.peak_score = score
            if score < self.peak_score or self.peak_age >= cfg.max_peak_frames:
                event_time = self.peak_index * self.frame_hop_seconds
                result = (event_time, self.peak_score)
                self.last_event_time = event_time
                self.state = "latched"
                self.release_count = 1 if score <= cfg.low_threshold else 0
                return result
            return None

        if score <= cfg.low_threshold:
            self.release_count += 1
        else:
            self.release_count = 0
        if (
            self.release_count >= cfg.release_frames
            and time_seconds - self.last_event_time >= cfg.min_gap_seconds
        ):
            self.state = "armed"
        return None


def decode_probabilities(
    probabilities: np.ndarray,
    config: DecoderConfig,
    frame_hop_seconds: float = 0.01,
) -> list[tuple[float, float]]:
    probabilities = causal_smooth(np.asarray(probabilities), config.smooth_frames)
    decoder = EventDecoder(config, frame_hop_seconds)
    events: list[tuple[float, float]] = []
    for index, score in enumerate(probabilities):
        event = decoder.feed(float(score), index)
        if event is not None:
            events.append(event)
    # A peak at the very final frame has not had a chance to turn downward.
    if decoder.state == "peak":
        events.append((decoder.peak_index * frame_hop_seconds, decoder.peak_score))
    return events
