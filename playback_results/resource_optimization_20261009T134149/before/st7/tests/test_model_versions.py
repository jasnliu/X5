from pathlib import Path
from unittest.mock import patch
import pytest

from cymbal_detector.cache import cache_directory
from cymbal_detector.config import load_config
from cymbal_detector.data import dataset_fingerprint, load_manifest
from cymbal_detector.train import train_all


def test_cache_paths_are_isolated():
    original = load_config()
    version = load_config('models/v2/config.yaml')
    assert cache_directory(original) == Path(original['_root']) / 'cache/fmn06'
    assert cache_directory(version) == Path(original['_root']) / 'models/v2/cache/fmn06'


@pytest.mark.parametrize('checkpoint', ['best.pt', 'seed-0/best.pt'])
def test_training_refuses_checkpoint_overwrite_before_cache(tmp_path, checkpoint):
    target = tmp_path / checkpoint
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(b'preserve me')
    config = {'_root': str(tmp_path), 'training': {'output_dir': str(tmp_path)}}
    with patch('cymbal_detector.train.build_feature_cache') as extract:
        with pytest.raises(FileExistsError, match='Refusing to overwrite'):
            train_all(config)
        extract.assert_not_called()
    assert target.read_bytes() == b'preserve me'


def test_v1_v2_splits_preserve_old_and_group_new_session():
    old = load_manifest(load_config('models/v1/config.yaml'), verify_hashes=True)
    new = load_manifest(load_config('models/v2/config.yaml'), verify_hashes=True)
    by_id = {r.id: r for r in new}
    assert len(old) == 55 and len(new) == 85
    assert all(r.split == by_id[r.id].split for r in old)
    assert all(r.split == 'train' for r in new if int(r.id[1:]) >= 56)
    assert dataset_fingerprint(old) == '48600b6f2f0d0df3527df94d92eca99447e91766188d0babbe6211419b6b17bc'
