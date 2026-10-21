from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import torch

from .config import rooted
from .data import load_audio
from .decoder import DecoderConfig, decode_probabilities
from .encoder import FrozenAudioEncoder
from .train import load_head, validate_checkpoint_target


class CymbalDetector:
    def __init__(self, config: dict, checkpoint_path: str | Path | None = None) -> None:
        if checkpoint_path is None:
            checkpoint_path = rooted(config, config["runtime"]["checkpoint"])
        self.model, self.checkpoint = load_head(checkpoint_path)
        validate_checkpoint_target(config, self.checkpoint)
        self.event_metadata = {}
        if config["data"].get("target_label") is not None:
            self.event_metadata = {
                "instrument": config["runtime"]["instrument"],
                "label": config["data"]["target_label"],
            }
        encoder_config = config["encoder"]
        self.encoder = FrozenAudioEncoder(
            rooted(config, encoder_config["weights"]),
            encoder_config["weights_sha256"],
        )
        self.encoder.eval()
        self.model.eval()
        self.decoder = DecoderConfig.from_dict(self.checkpoint["decoder"])
        self.sample_rate = int(encoder_config["sample_rate"])
        self.frame_hop_seconds = float(encoder_config["feature_hop_seconds"])

    @torch.inference_mode()
    def probabilities(self, waveform: torch.Tensor) -> np.ndarray:
        if waveform.ndim != 1:
            raise ValueError("waveform must be one-dimensional mono audio")
        features = self.encoder(waveform.unsqueeze(0))
        logits = self.model(features)
        return torch.softmax(logits, dim=-1)[0, :, 1].cpu().numpy()

    def detect_waveform(self, waveform: torch.Tensor) -> list[dict]:
        started = time.perf_counter()
        probabilities = self.probabilities(waveform)
        events = decode_probabilities(probabilities, self.decoder, self.frame_hop_seconds)
        elapsed = time.perf_counter() - started
        return [
            {**self.event_metadata, "time": event_time, "score": score, "inference_seconds": elapsed}
            for event_time, score in events
        ]

    def detect_file(self, path: str | Path) -> tuple[list[dict], float]:
        waveform = load_audio(path, self.sample_rate)
        return self.detect_waveform(waveform), waveform.numel() / self.sample_rate
