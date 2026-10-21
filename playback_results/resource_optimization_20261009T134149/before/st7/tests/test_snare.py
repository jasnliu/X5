"""Snare artifacts remain separate and keep the original X5 test capture held out."""
import json
import hashlib
import importlib.util
from pathlib import Path

import pytest
import torch

from cymbal_detector.cache import cache_directory
from cymbal_detector.config import load_config, rooted
from cymbal_detector.data import dataset_fingerprint, load_manifest
from cymbal_detector.detector import CymbalDetector
from cymbal_detector.normality import add_normality_scores, load_configured_normality
from cymbal_detector.data import load_audio
from cymbal_detector.train import validate_checkpoint_target


FOLDER = Path(__file__).resolve().parents[1] / "models/snare_v1"


def test_snare_dataset_and_manifest_paths_use_local_copy():
    config = load_config(FOLDER / "config.yaml")
    local = FOLDER.parents[1] / "data/snare"
    assert config["data"]["recordings"] == "data/snare/recordings"
    assert config["data"]["timestamps"] == "data/snare/timestamps"
    records = load_manifest(config, verify_hashes=True)
    assert all(r.wav.parent == local / "recordings" for r in records)
    assert all(r.csv.parent == local / "timestamps" for r in records)
    assert all(not r.wav.is_symlink() and not r.csv.is_symlink() for r in records)
    manifest = json.loads((FOLDER / "manifest.json").read_text())
    source_manifest = rooted(config, manifest["source_manifest"])
    assert source_manifest == local / "manifest.json"
    assert hashlib.sha256(source_manifest.read_bytes()).hexdigest() == manifest["source_manifest_sha256"]
    assert source_manifest.read_bytes() == (FOLDER / "source_manifest.json").read_bytes()
    for row, record in zip(manifest["records"], records):
        assert rooted(config, row["wav"]) == record.wav
        assert rooted(config, row["csv"]) == record.csv


def test_local_snare_preparation_reproduces_existing_split_without_overwrite(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("snare_preparation", FOLDER / "prepare_data.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.SOURCE == FOLDER.parents[1] / "data/snare"
    with pytest.raises(FileExistsError, match="Refusing to overwrite"):
        module.prepare()
    monkeypatch.setattr(module, "OUTPUT", tmp_path)
    module.prepare()
    assert json.loads((tmp_path / "manifest.json").read_text()) == json.loads((FOLDER / "manifest.json").read_text())
    assert (tmp_path / "config.yaml").read_bytes() == (FOLDER / "config.yaml").read_bytes()
    assert (tmp_path / "source_manifest.json").read_bytes() == (FOLDER / "source_manifest.json").read_bytes()
    assert json.loads((tmp_path / "input_audit.json").read_text())["source_directory"] == str(module.SOURCE)


def test_snare_pairing_hashes_labels_and_original_test_capture():
    config = load_config(FOLDER / "config.yaml")
    records = load_manifest(config, verify_hashes=True)
    assert len(records) == 60
    assert sum(len(r.hits) for r in records) == 45
    assert sum(not r.hits for r in records) == 20
    assert all(r.target_label == "snare_hit" for r in records)
    assert {r.id for r in records if r.split == "test"} == {f"r{i}" for i in range(49, 61)}
    for record in records:
        assert record.wav.name == f"{record.id}.wav"
        assert record.csv.name == f"t{record.id[1:]}.csv"
    hashes = {s: {r.wav_sha256 for r in records if r.split == s} for s in ("train", "val", "test")}
    assert not hashes["train"] & (hashes["val"] | hashes["test"])
    assert not hashes["val"] & hashes["test"]
    manifest = json.loads((FOLDER / "manifest.json").read_text())
    development_groups = {r["group"] for r in manifest["records"] if r["split"] != "test"}
    test_groups = {r["group"] for r in manifest["records"] if r["split"] == "test"}
    assert not development_groups & test_groups
    for split, counts in {"train": (38, 29), "val": (10, 7), "test": (12, 9)}.items():
        chosen = [r for r in records if r.split == split]
        assert (len(chosen), sum(len(r.hits) for r in chosen)) == counts


def test_snare_artifact_paths_and_checkpoint_are_isolated():
    config = load_config(FOLDER / "config.yaml")
    for section, name in (("data", "manifest"), ("training", "output_dir"),
                          ("runtime", "checkpoint"), ("normality", "artifact"),
                          ("normality", "report")):
        assert rooted(config, config[section][name]).is_relative_to(FOLDER)
    assert cache_directory(config).is_relative_to(FOLDER)
    checkpoint = torch.load(FOLDER / "best.pt", map_location="cpu", weights_only=False)
    validate_checkpoint_target(config, checkpoint)
    assert checkpoint["dataset_fingerprint"] == dataset_fingerprint(load_manifest(config))
    for version in ("v1", "v2", "hihat_v1"):
        old = load_config(FOLDER.parent / version / "config.yaml")
        with pytest.raises(ValueError, match="does not match"):
            validate_checkpoint_target(old, checkpoint)
        with pytest.raises(ValueError, match="does not match"):
            CymbalDetector(config, FOLDER.parent / version / "best.pt")


def test_snare_normality_fit_excludes_test_capture():
    config = load_config(FOLDER / "config.yaml")
    records = load_manifest(config)
    report = json.loads((FOLDER / "normality/report.json").read_text())
    meta = report["metadata"]
    assert meta["reference_split"] == "train"
    assert meta["calibration_split"] == "val"
    assert meta["reference_hits"] == 29
    assert meta["calibration_hits"] == 7
    train_ids = {r.id for r in records if r.split == "train"}
    assert all(r["recording"] in train_ids for r in meta["reference_sources"])


def test_snare_raw_audio_and_normality_preserve_events():
    config = load_config(FOLDER / "config.yaml")
    records = load_manifest(config)
    detector = CymbalDetector(config)
    scorer = load_configured_normality(config)
    for identifier in ("r49", "r50", "r53", "r54", "r55", "r56", "r58", "r60"):
        record = next(r for r in records if r.id == identifier)
        waveform = load_audio(record.wav)
        events = detector.detect_waveform(waveform)
        assert len(events) == len(record.hits)
        assert all(e["instrument"] == "snare" and e["label"] == "snare_hit" for e in events)
        assert all(abs(e["time"] - t) <= .05 for e, t in zip(events, record.hits))
        decorated = add_normality_scores(waveform, events, scorer)
        assert [{k: v for k, v in e.items() if k != "normality_score"} for e in decorated] == events
        assert all(0 <= e["normality_score"] <= 100 for e in decorated)
