from __future__ import annotations

import numpy as np
import torch
import json
from pathlib import Path

from cymbal_detector.config import load_config
from cymbal_detector.data import load_manifest
from cymbal_detector.decoder import DecoderConfig, decode_probabilities
from cymbal_detector.detector import CymbalDetector
from cymbal_detector.metrics import match_events
from cymbal_detector.model import CymbalTCN
from cymbal_detector.normality import add_normality_scores, load_configured_normality


def test_decoder_collapses_long_peak() -> None:
    probabilities = np.array([0.01] * 5 + [0.9] * 15 + [0.01] * 10)
    config = DecoderConfig(0.7, 0.2, 0.12)
    events = decode_probabilities(probabilities, config, frame_hop_seconds=0.04)
    assert len(events) == 1


def test_decoder_rearms_for_real_second_attack() -> None:
    probabilities = np.array([0.01] * 4 + [0.9] * 2 + [0.01] * 4 + [0.95] * 2 + [0.01] * 4)
    config = DecoderConfig(0.7, 0.2, 0.12, release_frames=2)
    events = decode_probabilities(probabilities, config, frame_hop_seconds=0.04)
    assert len(events) == 2


def test_decoder_ignores_one_frame_tail_dips() -> None:
    probabilities = np.array([0.01] * 4 + [0.9] * 3 + [0.01, 0.8, 0.01, 0.85] + [0.01] * 5)
    config = DecoderConfig(0.7, 0.2, 0.08, release_frames=2)
    events = decode_probabilities(probabilities, config, frame_hop_seconds=0.04)
    assert len(events) == 1


def test_causal_tcn_past_is_future_invariant() -> None:
    torch.manual_seed(1)
    model = CymbalTCN(input_dim=8, hidden_dim=8, dilations=(1, 2), dropout=0.0).eval()
    first = torch.randn(1, 20, 8)
    second = first.clone()
    second[:, 11:, :] = torch.randn_like(second[:, 11:, :])
    with torch.inference_mode():
        first_output = model(first)
        second_output = model(second)
    torch.testing.assert_close(first_output[:, :11], second_output[:, :11])


def test_event_matching_counts_tail_repeat_as_duplicate() -> None:
    metrics = match_events([1.0], [1.02, 1.4], 0.15, 3.0)
    assert metrics.true_positives == 1
    assert metrics.false_positives == 1
    assert metrics.duplicates == 1


def test_local_dataset_hashes_and_counts() -> None:
    records = load_manifest(load_config(), verify_hashes=True)
    manifest = json.loads(Path("data/ride/manifest.json").read_text())
    assert len(records) == len(manifest["records"]) > 0
    assert sum(len(record.hits) for record in records) == sum(len(row["hits"]) for row in manifest["records"])
    assert all(record.wav.parent.name == "recordings" for record in records)


def test_trained_program_handles_rapid_hits_and_negative() -> None:
    config = load_config()
    detector = CymbalDetector(config)
    positive, _ = detector.detect_file("data/ride/recordings/r35.wav")
    negative, _ = detector.detect_file("data/ride/recordings/r43.wav")
    assert len(positive) == 3
    assert negative == []


def test_normality_is_added_without_changing_hit_detection() -> None:
    config = load_config()
    detector = CymbalDetector(config)
    original, _ = detector.detect_file("data/ride/recordings/r35.wav")
    from cymbal_detector.data import load_audio

    waveform = load_audio("data/ride/recordings/r35.wav", detector.sample_rate)
    decorated = add_normality_scores(waveform, original, load_configured_normality(config))
    assert len(decorated) == len(original) == 3
    for before, after in zip(original, decorated, strict=True):
        assert after["time"] == before["time"]
        assert after["score"] == before["score"]
        assert after["inference_seconds"] == before["inference_seconds"]
        assert 0.0 <= after["normality_score"] <= 100.0


def test_normality_alignment_is_stable_near_a_detected_onset() -> None:
    config = load_config()
    scorer = load_configured_normality(config)
    from cymbal_detector.data import load_audio

    waveform = load_audio("data/ride/recordings/r32.wav", scorer.extractor.sample_rate)
    first = scorer.score_waveform(waveform, 2.521750)
    shifted = scorer.score_waveform(waveform, 2.541750)
    assert abs(first.aligned_time - shifted.aligned_time) <= 0.005
    assert abs(first.score - shifted.score) <= 2.0
