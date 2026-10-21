"""Verify instrument folders without creating or modifying dataset files."""
import json
from pathlib import Path

import pytest

from cymbal_detector.config import load_config, rooted
from cymbal_detector.data import load_manifest
from data_checker import DEFAULT_DATA_DIR, pair_folders
from record_camera_mic import DATA_DIR, RECORDINGS_DIR, TIMESTAMPS_DIR, next_dataset_index


@pytest.mark.parametrize('config_path', [
    'configs/baseline.yaml', 'models/v1/config.yaml', 'models/v2/config.yaml',
])
def test_active_models_use_ride_data(config_path):
    config = load_config(config_path)
    records = load_manifest(config, verify_hashes=True)
    assert records
    assert all(r.wav.parent == RECORDINGS_DIR and r.csv.parent == TIMESTAMPS_DIR for r in records)
    manifest = json.loads(rooted(config, config['data']['manifest']).read_text())
    for row in manifest['records']:
        assert rooted(config, row['wav']).is_file()
        assert rooted(config, row['csv']).is_file()


def test_recorder_and_checker_defaults_preserve_numbering():
    root = Path(__file__).resolve().parents[1]
    assert DATA_DIR == DEFAULT_DATA_DIR == root / 'data/ride'
    pairs = pair_folders(RECORDINGS_DIR, TIMESTAMPS_DIR)
    assert next_dataset_index() == max(p.number for p in pairs) + 1
    assert (root / 'data/hihat').is_dir()
    assert not (root / 'data/recordings').exists()
    assert not (root / 'data/timestamps').exists()
