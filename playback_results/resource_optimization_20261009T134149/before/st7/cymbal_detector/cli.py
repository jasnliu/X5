from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import torch

from .cache import build_feature_cache
from .config import load_config, rooted
from .data import load_audio, load_manifest, split_records
from .decoder import DecoderConfig
from .detector import CymbalDetector
from .evaluate import collect_probabilities, evaluate_probabilities
from .listener import list_devices, listen
from .normality import add_normality_scores, fit_normality, load_configured_normality
from .train import load_head, train_all, validate_checkpoint_target


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="Detect timestamped ride hits, hi-hat closures, or snare hits using the selected model config")
    result.add_argument("--config", default="configs/baseline.yaml")
    subparsers = result.add_subparsers(dest="command", required=True)
    extract = subparsers.add_parser("extract", help="validate data and cache frozen MobileNet features")
    extract.add_argument("--force", action="store_true")
    train = subparsers.add_parser("train", help="train all configured seeds and select on validation")
    train.add_argument("--force-features", action="store_true")
    normality = subparsers.add_parser(
        "fit-normality", help="fit the separate normal-hit similarity reference"
    )
    normality.add_argument("--output", help="override the configured normality artifact path")
    evaluate = subparsers.add_parser("evaluate", help="evaluate the selected checkpoint")
    evaluate.add_argument("--split", choices=("train", "val", "test", "all"), default="all")
    evaluate.add_argument("--checkpoint")
    detect = subparsers.add_parser("file", help="detect hits in an audio file")
    detect.add_argument("path")
    detect.add_argument("--checkpoint")
    detect.add_argument("--normality-artifact")
    subparsers.add_parser("devices", help="list microphone devices")
    live = subparsers.add_parser("listen", help="listen to a microphone and print HIT events as JSON")
    live.add_argument("--device", help="sounddevice index or name; default uses the system default")
    live.add_argument("--checkpoint")
    live.add_argument("--normality-artifact")
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    config = load_config(args.config)
    if args.command == "extract":
        records = build_feature_cache(config, force=args.force)
        print(json.dumps({"status": "ok", "records": len(records)}))
        return 0
    if args.command == "train":
        train_all(config, force_features=args.force_features)
        return 0
    if args.command == "fit-normality":
        report = fit_normality(config, args.output)
        print(json.dumps(report, indent=2))
        return 0
    if args.command == "devices":
        print(list_devices())
        return 0
    checkpoint = getattr(args, "checkpoint", None)
    checkpoint_path = checkpoint or rooted(config, config["runtime"]["checkpoint"])
    if args.command == "file":
        detector = CymbalDetector(config, checkpoint_path)
        events, duration = detector.detect_file(args.path)
        scorer = load_configured_normality(config, args.normality_artifact)
        waveform = load_audio(args.path, detector.sample_rate)
        events = add_normality_scores(waveform, events, scorer)
        print(json.dumps({"file": str(Path(args.path)), "duration": duration, "events": events}, indent=2))
        return 0
    if args.command == "listen":
        detector = CymbalDetector(config, checkpoint_path)
        scorer = load_configured_normality(config, args.normality_artifact)
        device = args.device if args.device is not None else config["runtime"].get("device")
        listen(config, detector, scorer, device)
        return 0
    if args.command == "evaluate":
        model, saved = load_head(checkpoint_path)
        validate_checkpoint_target(config, saved)
        records = load_manifest(config, verify_hashes=True)
        splits = ("train", "val", "test") if args.split == "all" else (args.split,)
        result = {}
        decoder = DecoderConfig.from_dict(saved["decoder"])
        tolerance = float(config["decoder_search"]["match_tolerance_seconds"])
        for split in splits:
            chosen = split_records(records, split)
            probabilities = collect_probabilities(model, config, chosen)
            metrics, details = evaluate_probabilities(probabilities, chosen, decoder, tolerance)
            result[split] = {"summary": metrics.as_dict(), "records": details}
        print(json.dumps(result, indent=2))
        return 0
    raise AssertionError(args.command)


if __name__ == "__main__":
    sys.exit(main())
