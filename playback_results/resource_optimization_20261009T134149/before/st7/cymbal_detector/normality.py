from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from .config import rooted
from .data import dataset_fingerprint, load_audio, load_manifest
from .frontend import LogMelFrontend


NORMALITY_VERSION = 1


@dataclass(frozen=True)
class NormalityResult:
    score: float
    distance: float
    aligned_time: float


class NormalityFeatureExtractor:
    """Create a gain-normalized spectral signature around a detected hit.

    This is deliberately separate from the onset model. The small alignment
    refinement is used only for the quality window and never changes the HIT
    timestamp or the detector's event decisions.
    """

    def __init__(
        self,
        sample_rate: int,
        before_seconds: float,
        after_seconds: float,
        alignment_search_seconds: float,
        pooled_mels: int,
        pooled_frames: int,
    ) -> None:
        self.sample_rate = int(sample_rate)
        self.before_seconds = float(before_seconds)
        self.after_seconds = float(after_seconds)
        self.alignment_search_seconds = float(alignment_search_seconds)
        self.pooled_mels = int(pooled_mels)
        self.pooled_frames = int(pooled_frames)
        if self.sample_rate != LogMelFrontend.sample_rate:
            raise ValueError(
                f"Normality frontend requires {LogMelFrontend.sample_rate} Hz, got {self.sample_rate}"
            )
        if min(self.before_seconds, self.after_seconds, self.alignment_search_seconds) < 0:
            raise ValueError("Normality time windows cannot be negative")
        if self.after_seconds <= 0 or self.pooled_mels <= 0 or self.pooled_frames <= 0:
            raise ValueError("Normality window and pooled dimensions must be positive")
        self.frontend = LogMelFrontend().eval()

    @property
    def signature_dim(self) -> int:
        return self.pooled_mels * self.pooled_frames

    def _quality_onset(self, waveform: torch.Tensor, candidate_time: float) -> float:
        """Align a quality window to the strongest nearby high-frequency rise."""

        sample_rate = self.sample_rate
        candidate = int(round(float(candidate_time) * sample_rate))
        context_seconds = self.alignment_search_seconds + 0.04
        context = int(round(context_seconds * sample_rate))
        left = max(0, candidate - context)
        right = min(waveform.numel(), candidate + context)
        samples = waveform[left:right]
        frame_samples = round(0.020 * sample_rate)
        hop_samples = round(0.005 * sample_rate)
        if samples.numel() < frame_samples:
            return max(0.0, min(float(candidate_time), waveform.numel() / sample_rate))

        # Pre-emphasis makes the alignment favor the sharp attack rather than
        # the already-ringing low-frequency tail.
        emphasized = torch.cat((samples[:1], samples[1:] - 0.97 * samples[:-1]))
        frames = emphasized.unfold(0, frame_samples, hop_samples)
        log_energy = torch.log(frames.square().mean(dim=1) + 1e-10)
        rise = torch.full_like(log_energy, -float("inf"))
        history_frames = 4
        for index in range(history_frames, log_energy.numel()):
            rise[index] = log_energy[index] - log_energy[index - history_frames : index].mean()

        centers = (
            torch.arange(log_energy.numel(), dtype=torch.float32) * hop_samples
            + frame_samples / 2
            + left
        ) / sample_rate
        candidate_tensor = torch.tensor(float(candidate_time))
        valid = torch.abs(centers - candidate_tensor) <= self.alignment_search_seconds
        if not bool(valid.any()):
            return max(0.0, min(float(candidate_time), waveform.numel() / sample_rate))
        best = int(torch.argmax(torch.where(valid, rise, torch.tensor(-float("inf")))).item())
        return float(centers[best])

    def _fixed_crop(self, waveform: torch.Tensor, center_seconds: float) -> torch.Tensor:
        before = int(round(self.before_seconds * self.sample_rate))
        after = int(round(self.after_seconds * self.sample_rate))
        center = int(round(center_seconds * self.sample_rate))
        start = center - before
        end = center + after
        source_start = max(0, start)
        source_end = min(waveform.numel(), end)
        crop = waveform[source_start:source_end]
        crop = F.pad(crop, (max(0, -start), max(0, end - waveform.numel())))
        expected = before + after
        if crop.numel() != expected:
            raise RuntimeError(f"Normality crop has {crop.numel()} samples, expected {expected}")
        return crop

    @torch.inference_mode()
    def signature(self, waveform: torch.Tensor, event_time: float) -> tuple[np.ndarray, float]:
        if waveform.ndim != 1:
            raise ValueError("Normality waveform must be one-dimensional mono audio")
        waveform = waveform.detach().to(device="cpu", dtype=torch.float32).contiguous()
        aligned_time = self._quality_onset(waveform, event_time)
        crop = self._fixed_crop(waveform, aligned_time)
        crop = crop - crop.mean()
        # Remove recording gain as a dominant quality cue. This scorer is meant
        # to measure spectral/temporal similarity, not merely loudness.
        crop = (crop / crop.square().mean().sqrt().clamp_min(1e-4)).clamp(-20.0, 20.0)
        logmel = self.frontend(crop).squeeze(0)
        pooled = F.adaptive_avg_pool2d(logmel, (self.pooled_mels, self.pooled_frames))
        signature = pooled.reshape(-1).cpu().numpy().astype(np.float64, copy=False)
        if not np.isfinite(signature).all():
            raise RuntimeError("Normality signature contains non-finite values")
        return signature, aligned_time


class NormalityScorer:
    """Distance-to-normal scorer loaded from a separate fitted artifact."""

    def __init__(
        self,
        metadata: dict,
        feature_mean: np.ndarray,
        feature_scale: np.ndarray,
        projection: np.ndarray,
        component_scale: np.ndarray,
        reference_embeddings: np.ndarray,
    ) -> None:
        self.metadata = metadata
        self.feature_mean = np.asarray(feature_mean, dtype=np.float64)
        self.feature_scale = np.asarray(feature_scale, dtype=np.float64)
        self.projection = np.asarray(projection, dtype=np.float64)
        self.component_scale = np.asarray(component_scale, dtype=np.float64)
        self.reference_embeddings = np.asarray(reference_embeddings, dtype=np.float64)
        self.neighbors = int(metadata["neighbors"])
        self.distance_scale = float(metadata["distance_scale"])
        self.score_at_distance_scale = float(metadata["score_at_distance_scale"])
        self.extractor = NormalityFeatureExtractor(
            sample_rate=int(metadata["sample_rate"]),
            before_seconds=float(metadata["before_seconds"]),
            after_seconds=float(metadata["after_seconds"]),
            alignment_search_seconds=float(metadata["alignment_search_seconds"]),
            pooled_mels=int(metadata["pooled_mels"]),
            pooled_frames=int(metadata["pooled_frames"]),
        )
        self._validate()

    def _validate(self) -> None:
        if int(self.metadata.get("normality_version", -1)) != NORMALITY_VERSION:
            raise RuntimeError("Unsupported normality artifact version")
        dim = self.extractor.signature_dim
        if self.feature_mean.shape != (dim,) or self.feature_scale.shape != (dim,):
            raise RuntimeError("Normality artifact feature dimensions do not match its frontend")
        if self.projection.ndim != 2 or self.projection.shape[0] != dim:
            raise RuntimeError("Normality artifact has an invalid PCA projection")
        components = self.projection.shape[1]
        if self.component_scale.shape != (components,):
            raise RuntimeError("Normality artifact has invalid component scaling")
        if self.reference_embeddings.ndim != 2 or self.reference_embeddings.shape[1] != components:
            raise RuntimeError("Normality artifact has invalid reference embeddings")
        if not 1 <= self.neighbors <= len(self.reference_embeddings):
            raise RuntimeError("Normality artifact has an invalid neighbor count")
        arrays = (
            self.feature_mean,
            self.feature_scale,
            self.projection,
            self.component_scale,
            self.reference_embeddings,
        )
        if not all(np.isfinite(value).all() for value in arrays):
            raise RuntimeError("Normality artifact contains non-finite values")
        if self.distance_scale <= 0 or not 0 < self.score_at_distance_scale < 100:
            raise RuntimeError("Normality artifact has invalid score calibration")

    @classmethod
    def load(cls, path: str | Path) -> "NormalityScorer":
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(
                f"Missing normality artifact {path}; run `python cymbal.py fit-normality`"
            )
        with np.load(path, allow_pickle=False) as saved:
            metadata = json.loads(str(saved["metadata_json"].item()))
            return cls(
                metadata,
                saved["feature_mean"],
                saved["feature_scale"],
                saved["projection"],
                saved["component_scale"],
                saved["reference_embeddings"],
            )

    def embed_signature(self, signature: np.ndarray) -> np.ndarray:
        signature = np.asarray(signature, dtype=np.float64)
        standardized = (signature - self.feature_mean) / self.feature_scale
        return (standardized @ self.projection) / self.component_scale

    def distance_for_embedding(self, embedding: np.ndarray) -> float:
        distances = np.sqrt(np.mean((self.reference_embeddings - embedding) ** 2, axis=1))
        nearest = np.partition(distances, self.neighbors - 1)[: self.neighbors]
        return float(nearest.mean())

    def score_for_distance(self, distance: float) -> float:
        exponent = math.log(self.score_at_distance_scale / 100.0)
        score = 100.0 * math.exp(exponent * (float(distance) / self.distance_scale) ** 2)
        return max(0.0, min(100.0, score))

    def score_waveform(self, waveform: torch.Tensor, event_time: float) -> NormalityResult:
        signature, aligned_time = self.extractor.signature(waveform, event_time)
        distance = self.distance_for_embedding(self.embed_signature(signature))
        return NormalityResult(
            score=self.score_for_distance(distance),
            distance=distance,
            aligned_time=aligned_time,
        )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def configured_normality_path(config: dict) -> Path:
    return rooted(config, config["normality"]["artifact"])


def load_configured_normality(config: dict, override: str | Path | None = None) -> NormalityScorer:
    return NormalityScorer.load(override or configured_normality_path(config))


def add_normality_scores(
    waveform: torch.Tensor,
    events: list[dict],
    scorer: NormalityScorer,
) -> list[dict]:
    decorated: list[dict] = []
    for event in events:
        result = scorer.score_waveform(waveform, float(event["time"]))
        decorated.append({**event, "normality_score": round(result.score, 1)})
    return decorated


def _neighbor_distance(
    embedding: np.ndarray,
    references: np.ndarray,
    neighbors: int,
    excluded_index: int | None = None,
) -> float:
    distances = np.sqrt(np.mean((references - embedding) ** 2, axis=1))
    if excluded_index is not None:
        distances[excluded_index] = float("inf")
    nearest = np.partition(distances, neighbors - 1)[:neighbors]
    return float(nearest.mean())


def _summary(values: list[float]) -> dict:
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "minimum": float(array.min()),
        "median": float(np.median(array)),
        "maximum": float(array.max()),
    }


def _atomic_save_npz(path: Path, **arrays: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=f".{path.stem}-", suffix=".npz", dir=path.parent)
    os.close(handle)
    try:
        np.savez_compressed(temporary, **arrays)
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def _atomic_write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=f".{path.name}-", dir=path.parent, text=True)
    try:
        with os.fdopen(handle, "w") as stream:
            stream.write(value)
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def fit_normality(config: dict, output_path: str | Path | None = None) -> dict:
    settings = config["normality"]
    output = Path(output_path) if output_path is not None else configured_normality_path(config)
    if not output.is_absolute():
        output = rooted(config, output)
    report_path = rooted(config, settings["report"])
    detector_checkpoint = rooted(config, config["runtime"]["checkpoint"])
    records = load_manifest(config, verify_hashes=True)
    reference_split = str(settings["reference_split"])
    calibration_split = str(settings["calibration_split"])
    extractor = NormalityFeatureExtractor(
        sample_rate=int(config["encoder"]["sample_rate"]),
        before_seconds=float(settings["before_seconds"]),
        after_seconds=float(settings["after_seconds"]),
        alignment_search_seconds=float(settings["alignment_search_seconds"]),
        pooled_mels=int(settings["pooled_mels"]),
        pooled_frames=int(settings["pooled_frames"]),
    )

    examples: list[dict] = []
    for record in records:
        if not record.hits:
            continue
        waveform = load_audio(record.wav, extractor.sample_rate)
        for hit_index, hit_time in enumerate(record.hits):
            signature, aligned_time = extractor.signature(waveform, hit_time)
            examples.append(
                {
                    "record": record,
                    "hit_index": hit_index,
                    "hit_time": hit_time,
                    "aligned_time": aligned_time,
                    "signature": signature,
                }
            )

    reference_examples = [item for item in examples if item["record"].split == reference_split]
    calibration_examples = [item for item in examples if item["record"].split == calibration_split]
    neighbors = int(settings["neighbors"])
    if len(reference_examples) <= neighbors:
        raise RuntimeError("Not enough reference hits for the configured normality neighbor count")
    if not calibration_examples:
        raise RuntimeError("Normality calibration split contains no hits")

    reference_signatures = np.stack([item["signature"] for item in reference_examples])
    feature_mean = reference_signatures.mean(axis=0)
    feature_scale = np.maximum(
        reference_signatures.std(axis=0, ddof=1), float(settings["feature_std_floor"])
    )
    standardized = (reference_signatures - feature_mean) / feature_scale
    _u, singular_values, right_vectors = np.linalg.svd(standardized, full_matrices=False)
    component_count = min(int(settings["pca_components"]), len(reference_examples) - 1)
    projection = right_vectors[:component_count].T
    projected = standardized @ projection
    component_scale = np.maximum(
        projected.std(axis=0, ddof=1), float(settings["component_std_floor"])
    )
    reference_embeddings = projected / component_scale

    def embed(signature: np.ndarray) -> np.ndarray:
        return ((signature - feature_mean) / feature_scale @ projection) / component_scale

    reference_distances = [
        _neighbor_distance(value, reference_embeddings, neighbors, excluded_index=index)
        for index, value in enumerate(reference_embeddings)
    ]
    calibration_distances = [
        _neighbor_distance(embed(item["signature"]), reference_embeddings, neighbors)
        for item in calibration_examples
    ]
    distance_scale = float(
        np.quantile(
            np.asarray(reference_distances + calibration_distances),
            float(settings["calibration_quantile"]),
        )
    )
    score_at_scale = float(settings["score_at_distance_scale"])
    explained = np.square(singular_values)
    explained_fraction = float(explained[:component_count].sum() / explained.sum())

    metadata = {
        "normality_version": NORMALITY_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "dataset_fingerprint": dataset_fingerprint(records),
        "detector_checkpoint": str(detector_checkpoint),
        "detector_checkpoint_sha256": _sha256(detector_checkpoint),
        "reference_split": reference_split,
        "calibration_split": calibration_split,
        "reference_hits": len(reference_examples),
        "calibration_hits": len(calibration_examples),
        "sample_rate": extractor.sample_rate,
        "before_seconds": extractor.before_seconds,
        "after_seconds": extractor.after_seconds,
        "alignment_search_seconds": extractor.alignment_search_seconds,
        "pooled_mels": extractor.pooled_mels,
        "pooled_frames": extractor.pooled_frames,
        "signature_dim": extractor.signature_dim,
        "pca_components": component_count,
        "pca_explained_variance_fraction": explained_fraction,
        "neighbors": neighbors,
        "distance_metric": "mean RMS Euclidean distance to nearest reference embeddings",
        "distance_scale": distance_scale,
        "calibration_quantile": float(settings["calibration_quantile"]),
        "score_at_distance_scale": score_at_scale,
        "score_semantics": "0-100 acoustic similarity to reference normal hits; not a probability",
        "gain_normalized": True,
        "reference_sources": [
            {
                "recording": item["record"].id,
                "hit_index": item["hit_index"],
                "hit_time": item["hit_time"],
            }
            for item in reference_examples
        ],
    }
    _atomic_save_npz(
        output,
        metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
        feature_mean=feature_mean,
        feature_scale=feature_scale,
        projection=projection,
        component_scale=component_scale,
        reference_embeddings=reference_embeddings,
    )

    scorer = NormalityScorer.load(output)
    rows_by_split: dict[str, list[dict]] = {"train": [], "val": [], "test": []}
    reference_indices = {id(item): index for index, item in enumerate(reference_examples)}
    for item in examples:
        embedding = scorer.embed_signature(item["signature"])
        reference_index = reference_indices.get(id(item))
        distance = _neighbor_distance(
            embedding,
            scorer.reference_embeddings,
            scorer.neighbors,
            excluded_index=reference_index,
        )
        rows_by_split[item["record"].split].append(
            {
                "recording": item["record"].id,
                "hit_index": item["hit_index"],
                "hit_time": item["hit_time"],
                "quality_aligned_time": item["aligned_time"],
                "distance": distance,
                "normality_score": scorer.score_for_distance(distance),
            }
        )

    report = {
        "artifact": str(output),
        "artifact_sha256": _sha256(output),
        "model_checkpoint_modified": False,
        "metadata": metadata,
        "splits": {
            split: {
                "distance": _summary([row["distance"] for row in rows]),
                "normality_score": _summary([row["normality_score"] for row in rows]),
                "hits": rows,
            }
            for split, rows in rows_by_split.items()
        },
        "limitations": [
            "Only normal hits are available, so this is a relative typicality score, not wrong-hit accuracy.",
            "Reference and calibration recordings may cover limited sessions; microphone or room changes may lower the score.",
            "The 0-100 value is not a probability and has no musical-quality interpretation.",
        ],
    }
    _atomic_write_text(report_path, json.dumps(report, indent=2) + "\n")
    return report
