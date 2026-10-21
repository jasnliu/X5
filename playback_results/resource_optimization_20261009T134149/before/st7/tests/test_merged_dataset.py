"""Read-only-source dataset merging, label union, and explicit unknown coverage."""
import copy
import csv
from decimal import Decimal
import json
from pathlib import Path
import wave

import pytest

from merge_datasets import (FIELDS, ROOT, SOURCES, TARGETS, group_records, merge,
                            plan_merge, read_sources, session_key, sha256, write_csv, write_json)


def fixtures(root):
    for instrument, wav_dir, csv_dir, manifest in SOURCES:
        wav = root/wav_dir/'r1.wav'; csv_path = root/csv_dir/'t1.csv'
        wav.parent.mkdir(parents=True); csv_path.parent.mkdir(parents=True)
        # Identical test-only PCM for all instruments; no microphone or inference.
        with wave.open(str(wav), 'wb') as output:
            output.setparams((1, 2, 16000, 16000, 'NONE', 'not compressed'))
            output.writeframes(b'\x00\x00' * 16000)
        write_csv(csv_path, FIELDS, [dict(recording_file='r1.wav', hit_time_seconds='0.250000',
                                       label=TARGETS[instrument])])
        path = root/manifest; path.parent.mkdir(parents=True, exist_ok=True)
        write_json(path, dict(records=[dict(id='r1', hits=[.25], duration=1.,
            wav_sha256=sha256(wav), csv_sha256=sha256(csv_path), group='r1', split='test')]))


def test_merge_deduplicates_audio_but_keeps_simultaneous_instruments(tmp_path):
    fixtures(tmp_path)
    before = {p: p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}
    summary = merge(tmp_path)
    assert summary['source_recordings'] == 3
    assert summary['unique_recordings'] == 1
    assert summary['duplicate_copies_omitted'] == 2
    assert summary['annotation_rows'] == 3
    destination = tmp_path/'data/merged'
    with (destination/'timestamps/t1.csv').open() as f:
        rows = list(csv.DictReader(f))
    assert {r['label'] for r in rows} == set(TARGETS.values())
    assert {Decimal(r['hit_time_seconds']) for r in rows} == {Decimal('.25')}
    assert all(p.read_bytes() == value for p, value in before.items())
    assert (destination/'recordings/r1.wav').stat().st_ino != (tmp_path/SOURCES[0][1]/'r1.wav').stat().st_ino
    manifest = json.loads((destination/'manifest.json').read_text())
    record = manifest['records'][0]
    assert len(record['sources']) == 3
    assert record['unreviewed_labels'] == []
    assert record['previously_held_out'] and record['split'] == 'unassigned'
    assert summary['training_ready'] is False
    with pytest.raises(FileExistsError, match='Refusing to overwrite'):
        merge(tmp_path)


def test_negative_for_one_target_does_not_erase_another_targets_positive(tmp_path):
    fixtures(tmp_path)
    entries, _ = read_sources(tmp_path)
    for e in entries[1:]:
        e['events'] = []
        e['metadata']['hits'] = []
    records, _, _ = plan_merge(entries)
    assert records[0]['events'] == [dict(hit_time_seconds='0.250000', label='cymbal_hit')]
    assert set(records[0]['annotated_labels']) == set(TARGETS.values())


def test_mixed_snare_annotations_are_unknown_not_other_instrument_negatives(tmp_path):
    fixtures(tmp_path)
    entries, _ = read_sources(tmp_path)
    snare = entries[2]
    snare['metadata']['kind'] = 'snare_both'
    records, _, reviews = plan_merge([snare])
    assert records[0]['unreviewed_labels'] == ['cymbal_hit', 'hihat_close']
    assert records[0]['expected_but_unlabeled'] == ['cymbal_hit', 'hihat_close']
    assert records[0]['annotated_labels'] == ['snare_hit']
    assert reviews[0]['expected_but_unlabeled'] == 'cymbal_hit;hihat_close'
    assert len(records[0]['events']) == 1  # No fabricated onsets.


def test_disagreeing_duplicate_annotations_are_retained_and_flagged(tmp_path):
    fixtures(tmp_path)
    entries, _ = read_sources(tmp_path)
    extra = copy.deepcopy(entries[0]); extra['events'] = [(Decimal('.3'), 'cymbal_hit')]
    records, _, reviews = plan_merge([entries[0], extra])
    assert len(records[0]['events']) == 2
    assert records[0]['conflicting_labels'] == ['cymbal_hit']
    assert reviews[0]['conflicting_labels'] == 'cymbal_hit'


def test_session_groups_are_transitive_and_original_audio_session_wins():
    records = [dict(session_keys=k) for k in (['A'], ['B'], ['A', 'B'], [])]
    group_records(records)
    assert len({r['group'] for r in records[:3]}) == 1
    assert records[3]['group'] != records[0]['group']
    assert session_key(dict(metadata=dict(group='original-session', collection_session='copied-later'))) == 'original-session'
    assert session_key(dict(metadata=dict(group='r1', session='unknown'))) is None


def test_tampered_source_rejected_without_output(tmp_path):
    fixtures(tmp_path)
    (tmp_path/SOURCES[0][2]/'t1.csv').write_text('corrupted')
    with pytest.raises(ValueError, match='hash mismatch'):
        merge(tmp_path)
    assert not (tmp_path/'data/merged').exists()


@pytest.mark.parametrize('label,at', [('wrong_label', '.25'), ('cymbal_hit', 'NaN'), ('cymbal_hit', '2')])
def test_invalid_labels_or_times_rejected(tmp_path, label, at):
    fixtures(tmp_path)
    csv_path = tmp_path/SOURCES[0][2]/'t1.csv'
    write_csv(csv_path, FIELDS, [dict(recording_file='r1.wav', hit_time_seconds=at, label=label)])
    manifest = tmp_path/SOURCES[0][3]
    data = json.loads(manifest.read_text()); data['records'][0]['csv_sha256'] = sha256(csv_path)
    write_json(manifest, data)
    with pytest.raises(ValueError):
        merge(tmp_path)
    assert not (tmp_path/'data/merged').exists()


def test_current_merged_dataset_preserves_every_source_and_existing_onset():
    entries, _ = read_sources(ROOT)
    destination = ROOT/'data/merged'
    manifest = json.loads((destination/'manifest.json').read_text())
    expected, mapping, reviews = plan_merge(entries)
    assert manifest['summary']['source_recordings'] == 205
    assert manifest['summary']['unique_recordings'] == 184
    assert manifest['summary']['duplicate_copies_omitted'] == 21
    assert manifest['summary']['annotation_rows'] == 174
    assert manifest['summary']['recordings_with_expected_missing_labels'] == 35
    assert len(list((destination/'recordings').glob('*.wav'))) == 184
    assert len(list((destination/'timestamps').glob('*.csv'))) == 184
    assert sum(len(r['sources']) for r in manifest['records']) == len(mapping) == 205
    assert len({r['wav_sha256'] for r in manifest['records']}) == 184
    assert manifest['records'][55]['id'] == 'r56'
    assert {s['instrument'] for s in manifest['records'][55]['sources']} == set(TARGETS)
    for expected_record, record in zip(expected, manifest['records']):
        assert {k: v for k, v in record.items() if k != 'csv_sha256'} == expected_record
        wav, csv_path = destination/record['wav'], destination/record['csv']
        assert not wav.is_symlink()
        assert sha256(wav) == record['wav_sha256']
        assert sha256(csv_path) == record['csv_sha256']
        with csv_path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert rows == [dict(recording_file=wav.name, **event) for event in record['events']]
    with (destination/'source_map.csv').open() as stream:
        assert list(csv.DictReader(stream)) == mapping
    with (destination/'annotation_review.csv').open() as stream:
        assert list(csv.DictReader(stream)) == reviews
