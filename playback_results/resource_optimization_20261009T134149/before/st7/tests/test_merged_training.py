"""Merged-data version preparation must not turn unknown labels into negatives."""
import json
from pathlib import Path

import pytest
import torch

from cymbal_detector.config import load_config, rooted
from cymbal_detector.data import load_manifest
from cymbal_detector.train import validate_checkpoint_target
from merge_datasets import ROOT, TARGETS, sha256
from prepare_merged_training import VERSIONS, eligibility, prepare
from verify_merged_models import score_replay


def merged_records():
    return json.loads((ROOT/'data/merged/manifest.json').read_text())['records']


@pytest.mark.parametrize('instrument,version,count,hits', [
    ('ride','ride_v3',123,95), ('hihat','hihat_v2',107,34), ('snare','snare_v2',89,45)])
def test_new_version_paths_counts_coverage_and_no_duplicate_split_leakage(instrument, version, count, hits):
    folder=ROOT/'models'/version
    config=load_config(folder/'config.yaml')
    records=load_manifest(config, verify_hashes=True)
    assert len(records)==count and sum(len(r.hits) for r in records)==hits
    assert config['data']['target_label']==TARGETS[instrument]
    assert set(config['data']['allowed_labels'])==set(TARGETS.values())
    assert config['runtime']['instrument']==instrument
    assert all(r.wav.parent==ROOT/'data/merged/recordings' for r in records)
    for section,key in (('training','output_dir'),('cache','directory'),('runtime','checkpoint'),('normality','artifact')):
        assert rooted(config,config[section][key]).is_relative_to(folder)
    manifest=json.loads((folder/'manifest.json').read_text())
    eligible={r['id'] for r in merged_records() if eligibility(r,instrument)}
    assert {r.id for r in records}==eligible
    assert {r['id'] for r in manifest['excluded']}=={r['id'] for r in merged_records()}-eligible
    for split in ('train','val','test'):
        selected=[r for r in records if r.split==split]
        assert selected and sum(len(r.hits) for r in selected)>0
    hashes={s:{r.wav_sha256 for r in records if r.split==s} for s in ('train','val','test')}
    assert not (hashes['train']&hashes['test'] or hashes['train']&hashes['val'] or hashes['val']&hashes['test'])
    assert manifest['source_merged_manifest_sha256']==sha256(ROOT/'data/merged/manifest.json')


def test_known_mixed_target_onsets_missing_are_excluded_not_labeled_negative():
    rows=merged_records()
    mixed=next(r for r in rows if any(s['instrument']=='snare' and s['metadata']['kind']=='snare_both' for s in r['sources']))
    assert eligibility(mixed,'snare')=='original_target_annotations'
    assert eligibility(mixed,'ride') is None
    assert eligibility(mixed,'hihat') is None
    solo=next(r for r in rows if any(s['instrument']=='snare' and s['metadata']['kind']=='snare_only' for s in r['sources']))
    assert eligibility(solo,'ride').startswith('explicit_snare_collection_target_absent')
    assert eligibility(solo,'hihat').startswith('explicit_snare_collection_target_absent')
    unknown=next(r for r in rows if r['id']=='r1')
    assert eligibility(unknown,'ride')=='original_target_annotations'
    assert eligibility(unknown,'hihat') is None and eligibility(unknown,'snare') is None


def test_shared_split_keeps_all_original_positive_holdouts_and_snare_test_capture():
    assignments={}
    for instrument,(old,version) in VERSIONS.items():
        manifest=json.loads((ROOT/'models'/version/'manifest.json').read_text())
        for record in manifest['records']:
            assignments.setdefault(record['id'],record['split'])
            assert assignments[record['id']]==record['split']
        old_rows=json.loads((ROOT/'models'/old/'manifest.json').read_text())['records']
        model_rows={r['id']:r for r in manifest['records']}
        for old_row in old_rows:
            if not old_row['hits']:continue
            merged=next(r for r in merged_records() if r['wav_sha256']==old_row['wav_sha256'])
            assert model_rows[merged['id']]['split']==old_row['split']
    for row in merged_records():
        if any(s['instrument']=='snare' and s['metadata']['split']=='test' for s in row['sources']):
            assert assignments[row['id']]=='test'


def test_preparation_refuses_existing_model_directories(tmp_path):
    with pytest.raises(FileExistsError,match='Refusing to overwrite'):
        prepare(tmp_path/'audit')
    assert not (tmp_path/'audit').exists()


def test_listener_scoring_keeps_padding_events_as_explicit_diagnostics():
    events = [dict(time=t,score=0.9) for t in (-0.2,0.0,0.5,0.9,1.0,1.2)]
    original = [dict(event) for event in events]
    scored,padding,metrics,inclusive = score_replay([0.0,0.5,0.7],events,0.05,1.0)
    assert [e['time'] for e in scored] == [0.0,0.5,0.9]
    assert [e['time'] for e in padding] == [-0.2,1.0,1.2]
    assert metrics.as_dict()['true_positives'] == 2
    assert metrics.as_dict()['false_positives'] == 1
    assert metrics.as_dict()['false_negatives'] == 1
    assert inclusive.as_dict()['true_positives'] == 2
    assert inclusive.as_dict()['false_positives'] == 4
    assert inclusive.as_dict()['false_negatives'] == 1
    assert events == original


@pytest.mark.parametrize('instrument', list(VERSIONS))
def test_completed_checkpoints_and_normality_are_isolated_and_validation_selected(instrument):
    _,version=VERSIONS[instrument]
    folder=ROOT/'models'/version
    assert (folder/'training.exitcode').read_text().strip()=='0'
    assert (folder/'normality-fit.exitcode').read_text().strip()=='0'
    config=load_config(folder/'config.yaml')
    checkpoint=torch.load(folder/'best.pt',map_location='cpu',weights_only=False)
    validate_checkpoint_target(config,checkpoint)
    assert {s['seed'] for s in checkpoint['selection']}=={0,1,2}
    for seed in (0,1,2):
        history=json.loads((folder/f'seed-{seed}/history.json').read_text())
        assert history and (folder/f'seed-{seed}/best.pt').is_file()
    for other,(_,other_version) in VERSIONS.items():
        if other!=instrument:
            with pytest.raises(ValueError,match='does not match'):
                validate_checkpoint_target(load_config(ROOT/'models'/other_version/'config.yaml'),checkpoint)
    normality=json.loads((folder/'normality/report.json').read_text())['metadata']
    assert normality['detector_checkpoint_sha256']==sha256(folder/'best.pt')
    assert normality['dataset_fingerprint']==checkpoint['dataset_fingerprint']
    records=load_manifest(config)
    training_ids={r.id for r in records if r.split=='train'}
    assert all(r['recording'] in training_ids for r in normality['reference_sources'])
    assert normality['reference_split']=='train' and normality['calibration_split']=='val'
