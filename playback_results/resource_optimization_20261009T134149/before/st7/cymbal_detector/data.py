from __future__ import annotations

import csv
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import soundfile as sf
import torch
import torchaudio

from .config import rooted


@dataclass(frozen=True)
class Recording:
    id: str
    wav: Path
    csv: Path
    hits: tuple[float, ...]
    duration: float
    split: str
    wav_sha256: str
    csv_sha256: str
    target_label: str | None = None


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_manifest(config: dict, verify_hashes: bool = False) -> list[Recording]:
    manifest_path = rooted(config, config["data"]["manifest"])
    recordings_dir = rooted(config, config["data"]["recordings"])
    timestamps_dir = rooted(config, config["data"]["timestamps"])
    raw = json.loads(manifest_path.read_text())
    target_label = config["data"].get("target_label")
    if target_label is not None and raw.get("target_label") != target_label:
        raise ValueError("Manifest target_label must match the configured detector target")
    allowed_labels = set(config["data"].get("allowed_labels", [target_label]))
    records: list[Recording] = []
    for item in raw["records"]:
        number = item["id"].removeprefix("r")
        wav = recordings_dir / f"r{number}.wav"
        timestamp_csv = timestamps_dir / f"t{number}.csv"
        if not wav.is_file() or not timestamp_csv.is_file():
            raise FileNotFoundError(f"Missing source files for {item['id']}")
        with timestamp_csv.open(newline="") as stream:
            reader = csv.DictReader(stream)
            if target_label is not None and not {"recording_file", "hit_time_seconds", "label"}.issubset(reader.fieldnames or []):
                raise ValueError(f"CSV requires recording_file, hit_time_seconds, label for {item['id']}")
            rows = list(reader)
        if target_label is not None:
            # Filter instrument-specific annotations, without changing legacy
            # ride configurations or their existing dataset fingerprints.
            for row in rows:
                if row.get("label") not in allowed_labels:
                    raise ValueError(f"Unknown annotation label for {item['id']}: {row.get('label')!r}")
                if row.get("recording_file") != wav.name:
                    raise ValueError(f"CSV recording_file mismatch for {item['id']}")
                seconds = float(row["hit_time_seconds"])
                if not math.isfinite(seconds) or not 0 <= seconds < float(item["duration"]):
                    raise ValueError(f"Invalid timestamp for {item['id']}: {seconds}")
        hits = tuple(float(row["hit_time_seconds"]) for row in rows
                     if target_label is None or row["label"] == target_label)
        if hits != tuple(float(value) for value in item["hits"]):
            raise ValueError(f"CSV/manifest timestamps disagree for {item['id']}")
        if verify_hashes:
            if _sha256(wav) != item["wav_sha256"] or _sha256(timestamp_csv) != item["csv_sha256"]:
                raise ValueError(f"Source hash mismatch for {item['id']}")
        records.append(
            Recording(
                id=item["id"],
                wav=wav,
                csv=timestamp_csv,
                hits=hits,
                duration=float(item["duration"]),
                split=item["split"],
                wav_sha256=item["wav_sha256"],
                csv_sha256=item["csv_sha256"],
                target_label=target_label,
            )
        )
    return records


def load_audio(path: str | Path, sample_rate: int = 16_000) -> torch.Tensor:
    samples, source_rate = sf.read(path, dtype="float32", always_2d=True)
    waveform = torch.from_numpy(np.mean(samples, axis=1))
    if source_rate != sample_rate:
        waveform = torchaudio.functional.resample(waveform, source_rate, sample_rate)
    return waveform.contiguous()


def split_records(records: Iterable[Recording], split: str) -> list[Recording]:
    return [record for record in records if record.split == split]


def dataset_fingerprint(records: Iterable[Recording]) -> str:
    digest = hashlib.sha256()
    for record in sorted(records, key=lambda item: item.id):
        digest.update(record.id.encode())
        digest.update(record.wav_sha256.encode())
        digest.update(record.csv_sha256.encode())
        digest.update(record.split.encode())
        if record.target_label is not None:
            digest.update(b"\0target_label=")
            digest.update(record.target_label.encode())
    return digest.hexdigest()
