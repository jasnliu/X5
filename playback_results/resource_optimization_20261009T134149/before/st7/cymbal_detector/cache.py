from __future__ import annotations

import json
import time
from pathlib import Path

import torch

from .config import rooted
from .data import Recording, dataset_fingerprint, load_audio, load_manifest
from .encoder import FrozenAudioEncoder, sha256_file


def cache_directory(config: dict) -> Path:
    return rooted(config, config.get("cache", {}).get("directory", "cache/fmn06"))


def build_feature_cache(config: dict, force: bool = False) -> list[Recording]:
    records = load_manifest(config, verify_hashes=True)
    directory = cache_directory(config)
    directory.mkdir(parents=True, exist_ok=True)
    weights = rooted(config, config["encoder"]["weights"])
    expected = config["encoder"]["weights_sha256"]
    encoder = FrozenAudioEncoder(weights, expected)
    encoder.eval()
    started = time.perf_counter()
    for index, record in enumerate(records, 1):
        target = directory / f"{record.id}.pt"
        if target.exists() and not force:
            cached = torch.load(target, map_location="cpu", weights_only=True)
            if (
                cached.get("wav_sha256") == record.wav_sha256
                and cached.get("encoder_sha256") == expected
                and cached.get("feature_version") == encoder.feature_version
            ):
                continue
        waveform = load_audio(record.wav)
        features = encoder(waveform.unsqueeze(0)).squeeze(0).cpu()
        torch.save(
            {
                "id": record.id,
                "features": features,
                "hits": list(record.hits),
                "duration": record.duration,
                "split": record.split,
                "wav_sha256": record.wav_sha256,
                "encoder_sha256": expected,
                "feature_hop_seconds": encoder.feature_hop_seconds,
                "feature_version": encoder.feature_version,
            },
            target,
        )
        print(f"features {index:02d}/{len(records)} {record.id}: {tuple(features.shape)}", flush=True)
    metadata = {
        "dataset_fingerprint": dataset_fingerprint(records),
        "encoder_sha256": sha256_file(weights),
        "feature_hop_seconds": encoder.feature_hop_seconds,
        "feature_version": encoder.feature_version,
        "records": len(records),
        "elapsed_seconds": time.perf_counter() - started,
    }
    (directory / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return records


def load_cached(config: dict, record: Recording) -> dict:
    path = cache_directory(config) / f"{record.id}.pt"
    if not path.is_file():
        raise FileNotFoundError(f"Missing feature cache {path}; run the extract command first")
    return torch.load(path, map_location="cpu", weights_only=True)
