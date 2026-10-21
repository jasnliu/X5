from __future__ import annotations

import json
import math
import os
import random
import time
from pathlib import Path

import numpy as np
import torch
from torch import nn

from .cache import build_feature_cache, load_cached
from .config import rooted
from .data import Recording, dataset_fingerprint, load_manifest, split_records
from .evaluate import collect_probabilities, evaluate_probabilities, tune_decoder
from .model import CymbalTCN, make_model


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.use_deterministic_algorithms(True)


def frame_targets(record: Recording, length: int, hop: float, radius: int) -> torch.Tensor:
    target = torch.zeros(length, dtype=torch.long)
    for hit in record.hits:
        center = min(length - 1, max(0, round(hit / hop)))
        target[max(0, center - radius) : min(length, center + radius + 1)] = 1
    return target


def ctc_loss_for_record(logits: torch.Tensor, hit_count: int, loss_fn: nn.CTCLoss) -> torch.Tensor:
    length = logits.shape[1]
    targets = torch.ones(hit_count, dtype=torch.long)
    input_lengths = torch.tensor([length], dtype=torch.long)
    target_lengths = torch.tensor([hit_count], dtype=torch.long)
    log_probs = torch.log_softmax(logits, dim=-1).transpose(0, 1)
    return loss_fn(log_probs, targets, input_lengths, target_lengths) / max(1, hit_count)


def train_seed(config: dict, records: list[Recording], seed: int, output: Path) -> dict:
    set_seed(seed)
    output.mkdir(parents=True, exist_ok=True)
    training = config["training"]
    hop = float(config["encoder"]["feature_hop_seconds"])
    train_records = split_records(records, "train")
    validation_records = split_records(records, "val")
    cached = {record.id: load_cached(config, record)["features"] for record in train_records}
    model = make_model(config)
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=float(training["learning_rate"]),
        weight_decay=float(training["weight_decay"]),
    )
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode="max", factor=0.5, patience=15)
    ce_loss = nn.CrossEntropyLoss(weight=torch.tensor([1.0, float(training["positive_weight"])]))
    ctc_loss = nn.CTCLoss(blank=0, reduction="sum", zero_infinity=True)
    ctc_weight = float(training["ctc_weight"])
    radius = int(training["onset_radius_frames"])
    patience = int(training["patience"])
    best_rank: tuple | None = None
    stale = 0
    history: list[dict] = []
    started = time.perf_counter()

    for epoch in range(1, int(training["epochs"]) + 1):
        model.train()
        order = list(train_records)
        random.shuffle(order)
        losses: list[float] = []
        ce_values: list[float] = []
        ctc_values: list[float] = []
        for record in order:
            features = cached[record.id].unsqueeze(0)
            # Small embedding-space perturbation regularizes the tiny single-session set.
            scale = 1.0 + (torch.rand(1).item() - 0.5) * 0.04
            noise = torch.randn_like(features) * 0.005
            logits = model(features * scale + noise)
            target = frame_targets(record, logits.shape[1], hop, radius)
            frame_loss = ce_loss(logits.squeeze(0), target)
            sequence_loss = ctc_loss_for_record(logits, len(record.hits), ctc_loss)
            loss = frame_loss + ctc_weight * sequence_loss
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), float(training["gradient_clip"]))
            optimizer.step()
            losses.append(float(loss.detach()))
            ce_values.append(float(frame_loss.detach()))
            ctc_values.append(float(sequence_loss.detach()))

        val_probabilities = collect_probabilities(model, config, validation_records)
        decoder, val_metrics, _ = tune_decoder(val_probabilities, validation_records, config)
        summary = val_metrics.as_dict()
        mae = summary["onset_mae_seconds"] if summary["onset_mae_seconds"] is not None else 99.0
        rank = (
            summary["f1"],
            summary["recall"],
            summary["precision"],
            -summary["duplicates"],
            -mae,
        )
        scheduler.step(summary["f1"])
        row = {
            "epoch": epoch,
            "loss": float(np.mean(losses)),
            "frame_loss": float(np.mean(ce_values)),
            "ctc_loss": float(np.mean(ctc_values)),
            "learning_rate": optimizer.param_groups[0]["lr"],
            "validation": summary,
            "decoder": decoder.as_dict(),
        }
        history.append(row)
        if best_rank is None or rank > best_rank:
            best_rank = rank
            stale = 0
            torch.save(
                {
                    "state_dict": model.state_dict(),
                    "model_args": model.model_args,
                    "decoder": decoder.as_dict(),
                    "seed": seed,
                    "epoch": epoch,
                    "validation": summary,
                    "dataset_fingerprint": dataset_fingerprint(records),
                    "encoder": config["encoder"],
                    "config": config,
                    "target_label": config["data"].get("target_label"),
                },
                output / "best.pt",
            )
        else:
            stale += 1
        if epoch == 1 or epoch % 10 == 0 or stale == 0:
            print(
                f"seed={seed} epoch={epoch:03d} loss={row['loss']:.4f} "
                f"val_f1={summary['f1']:.3f} P={summary['precision']:.3f} "
                f"R={summary['recall']:.3f} fp={summary['false_positives']} "
                f"dup={summary['duplicates']} stale={stale}",
                flush=True,
            )
        if stale >= patience:
            print(f"seed={seed} early stop at epoch {epoch}", flush=True)
            break

    (output / "history.json").write_text(json.dumps(history, indent=2) + "\n")
    checkpoint = torch.load(output / "best.pt", map_location="cpu", weights_only=False)
    return {
        "seed": seed,
        "checkpoint": str(output / "best.pt"),
        "epoch": checkpoint["epoch"],
        "validation": checkpoint["validation"],
        "elapsed_seconds": time.perf_counter() - started,
    }


def load_head(checkpoint_path: str | Path) -> tuple[CymbalTCN, dict]:
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    args = checkpoint["model_args"]
    args["dilations"] = tuple(args["dilations"])
    model = CymbalTCN(**args)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()
    return model, checkpoint


def validate_checkpoint_target(config: dict, checkpoint: dict) -> None:
    requested = config["data"].get("target_label")
    saved = checkpoint.get("target_label", checkpoint.get("config", {}).get("data", {}).get("target_label"))
    if requested != saved:
        raise ValueError(
            f"Checkpoint target {saved!r} does not match config target {requested!r}. "
            "Use the matching model version's config.yaml."
        )


def train_all(config: dict, force_features: bool = False) -> dict:
    torch.set_num_threads(min(8, os.cpu_count() or 1))
    run_directory = rooted(config, config["training"].get("output_dir", "runs/cymbal-fmn-tcn"))
    if (run_directory / "best.pt").exists() or any(run_directory.glob("seed-*/*.pt")):
        raise FileExistsError(
            f"Refusing to overwrite checkpoints in {run_directory}. "
            "Set training.output_dir to a new version folder."
        )
    records = build_feature_cache(config, force=force_features)
    run_directory.mkdir(parents=True, exist_ok=True)
    print(f"Training output: {run_directory}", flush=True)
    for split in ("train", "val", "test"):
        chosen = split_records(records, split)
        print(f"{split}: {len(chosen)} recordings, {sum(len(r.hits) for r in chosen)} hits", flush=True)
    summaries = []
    for seed in config["training"]["seeds"]:
        summaries.append(train_seed(config, records, int(seed), run_directory / f"seed-{seed}"))

    def rank(item: dict) -> tuple:
        value = item["validation"]
        mae = value["onset_mae_seconds"] if value["onset_mae_seconds"] is not None else 99.0
        return (value["f1"], value["recall"], value["precision"], -value["duplicates"], -mae)

    winner = max(summaries, key=rank)
    selected = torch.load(winner["checkpoint"], map_location="cpu", weights_only=False)
    selected["selection"] = summaries
    best_path = run_directory / "best.pt"
    torch.save(selected, best_path)

    model, checkpoint = load_head(best_path)
    tolerance = float(config["decoder_search"]["match_tolerance_seconds"])
    from .decoder import DecoderConfig

    decoder = DecoderConfig.from_dict(checkpoint["decoder"])
    report = {"selected_seed": checkpoint["seed"], "selected_epoch": checkpoint["epoch"], "splits": {}}
    for split in ("train", "val", "test"):
        selected_records = split_records(records, split)
        probabilities = collect_probabilities(model, config, selected_records)
        metrics, details = evaluate_probabilities(probabilities, selected_records, decoder, tolerance)
        report["splits"][split] = {"summary": metrics.as_dict(), "records": details}
    report["decoder"] = checkpoint["decoder"]
    report["selection"] = summaries
    (run_directory / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["splits"], indent=2), flush=True)
    return report
