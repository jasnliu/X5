from __future__ import annotations

import itertools
from typing import Iterable

import numpy as np
import torch

from .cache import load_cached
from .data import Recording
from .decoder import DecoderConfig, decode_probabilities
from .metrics import EventMetrics, match_events


@torch.inference_mode()
def collect_probabilities(model: torch.nn.Module, config: dict, records: Iterable[Recording]) -> dict[str, np.ndarray]:
    model.eval()
    output: dict[str, np.ndarray] = {}
    for record in records:
        features = load_cached(config, record)["features"].unsqueeze(0)
        logits = model(features)
        output[record.id] = torch.softmax(logits, dim=-1)[0, :, 1].cpu().numpy()
    return output


def evaluate_probabilities(
    probabilities: dict[str, np.ndarray],
    records: Iterable[Recording],
    decoder: DecoderConfig,
    tolerance_seconds: float,
    frame_hop_seconds: float = 0.01,
) -> tuple[EventMetrics, dict[str, dict]]:
    total = EventMetrics()
    details: dict[str, dict] = {}
    for record in records:
        events = decode_probabilities(probabilities[record.id], decoder, frame_hop_seconds)
        event_times = [event[0] for event in events]
        metrics = match_events(record.hits, event_times, tolerance_seconds, record.duration)
        total.add(metrics)
        details[record.id] = {
            "truth": list(record.hits),
            "events": [{"time": time, "score": score} for time, score in events],
            "metrics": metrics.as_dict(),
        }
    return total, details


def tune_decoder(
    probabilities: dict[str, np.ndarray],
    records: list[Recording],
    config: dict,
) -> tuple[DecoderConfig, EventMetrics, dict[str, dict]]:
    search = config["decoder_search"]
    tolerance = float(search["match_tolerance_seconds"])
    best: tuple | None = None
    best_payload = None
    for high, ratio, gap, smooth in itertools.product(
        search["high_thresholds"],
        search["low_ratios"],
        search["min_gap_seconds"],
        search["smooth_frames"],
    ):
        decoder = DecoderConfig(
            high_threshold=float(high),
            low_threshold=float(high) * float(ratio),
            min_gap_seconds=float(gap),
            smooth_frames=int(smooth),
        )
        metrics, details = evaluate_probabilities(probabilities, records, decoder, tolerance)
        summary = metrics.as_dict()
        mae = summary["onset_mae_seconds"] if summary["onset_mae_seconds"] is not None else 99.0
        rank = (
            summary["f1"],
            summary["recall"],
            summary["precision"],
            -metrics.duplicates,
            -mae,
            high,
        )
        if best is None or rank > best:
            best = rank
            best_payload = (decoder, metrics, details)
    assert best_payload is not None
    return best_payload
