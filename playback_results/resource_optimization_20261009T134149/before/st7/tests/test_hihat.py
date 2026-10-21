"""Instrument-specific labels must not alter the legacy ride detector."""
import csv
import hashlib
import json
from pathlib import Path

import pytest
import torch

from cymbal_detector.config import load_config
from cymbal_detector.data import load_manifest, dataset_fingerprint
from cymbal_detector.train import validate_checkpoint_target


def test_hihat_manifest_labels_and_disjoint_splits():
    config = load_config('models/hihat_v1/config.yaml')
    records = load_manifest(config, verify_hashes=True)
    assert len(records) == 60
    assert sum(len(r.hits) for r in records) == 34
    assert all(r.target_label == 'hihat_close' for r in records)
    assert all(not r.hits for r in records if int(r.id[1:]) > 30)
    hashes = {s: {r.wav_sha256 for r in records if r.split == s} for s in ('train', 'val', 'test')}
    assert not hashes['train'] & (hashes['val'] | hashes['test'])
    assert not hashes['val'] & hashes['test']


def test_legacy_ride_fingerprints_unchanged():
    for version in ('v1', 'v2'):
        config = load_config(f'models/{version}/config.yaml')
        records = load_manifest(config, verify_hashes=True)
        saved = torch.load(f'models/{version}/best.pt', map_location='cpu', weights_only=False)
        assert dataset_fingerprint(records) == saved['dataset_fingerprint']
        assert all(r.target_label is None for r in records)
        validate_checkpoint_target(config, saved)


def test_wrong_instrument_checkpoint_is_rejected():
    hihat = {'data': {'target_label': 'hihat_close'}}
    ride = {'data': {}}
    validate_checkpoint_target(hihat, {'target_label': 'hihat_close'})
    validate_checkpoint_target(ride, {'config': ride})
    with pytest.raises(ValueError, match='does not match'):
        validate_checkpoint_target(hihat, {'config': ride})
    with pytest.raises(ValueError, match='does not match'):
        validate_checkpoint_target(ride, {'target_label': 'hihat_close'})


def test_label_filtering_and_unknown_labels(tmp_path):
    (tmp_path / 'r1.wav').write_bytes(b'placeholder: loader only reads hash')
    source = tmp_path / 't1.csv'
    source.write_text('recording_file,hit_time_seconds,label\nr1.wav,0.1,cymbal_hit\nr1.wav,0.3,hihat_close\n')
    row = {'id': 'r1', 'hits': [0.3], 'duration': 1., 'split': 'train',
           'wav_sha256': hashlib.sha256((tmp_path / 'r1.wav').read_bytes()).hexdigest(),
           'csv_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    manifest = tmp_path / 'manifest.json'
    manifest.write_text(json.dumps({'target_label': 'hihat_close', 'records': [row]}))
    config = {'_root': str(tmp_path), 'data': {'manifest': 'manifest.json', 'recordings': '.',
              'timestamps': '.', 'target_label': 'hihat_close', 'allowed_labels': ['hihat_close', 'cymbal_hit']}}
    assert load_manifest(config, verify_hashes=True)[0].hits == (0.3,)
    source.write_text(source.read_text().replace('hihat_close', 'typo'))
    with pytest.raises(ValueError, match='Unknown annotation label'):
        load_manifest(config)
    source.write_text('wrong,header\n')
    with pytest.raises(ValueError, match='CSV requires'):
        load_manifest(config)


def test_trained_hihat_raw_wav_events_and_ride_rejection():
    from cymbal_detector.detector import CymbalDetector
    config = load_config('models/hihat_v1/config.yaml')
    detector = CymbalDetector(config)
    events, _ = detector.detect_file('data/hihat/recording/r1.wav')
    assert len(events) == 2
    assert all(e['instrument'] == 'hihat' and e['label'] == 'hihat_close' for e in events)
    assert abs(events[0]['time'] - 1.5) <= .05
    assert abs(events[1]['time'] - 2.7055) <= .05
    negative, _ = detector.detect_file('data/hihat/recording/r41.wav')
    assert negative == []


def test_detector_rejects_ride_weights_under_hihat_config():
    from cymbal_detector.detector import CymbalDetector
    with pytest.raises(ValueError, match='does not match'):
        CymbalDetector(load_config('models/hihat_v1/config.yaml'), 'models/v1/best.pt')
