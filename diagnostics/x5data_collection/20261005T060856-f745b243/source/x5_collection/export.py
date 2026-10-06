"""Offline WAV/CSV export from explicitly reviewed audio onsets. Never opens CAN.

Encoder release times alone are NOT labels. review.json must specify actual
audio onsets in raw.wav seconds, with acceptance and review provenance.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import wave

import numpy as np

from .plan import CSV_FIELDS

ROOT = Path(__file__).resolve().parents[1]


def export(session, review, output):
    session, output = Path(session).resolve(), Path(output).resolve()
    result = json.loads((session/'result.json').read_text())
    final = json.loads((session/'query_final_disabled.json').read_text())
    audio = json.loads((session/'audio_result.json').read_text())
    if not result['relaxed_verified'] or not final['all_16_disabled_fault_free']:
        raise ValueError('Label/export only after verified center and disabled feedback')
    if audio['errors']:
        raise ValueError('Do not export discontinuous or failed audio capture')
    if not review.get('method'):
        raise ValueError('Review provenance is required; attempts are not labels')
    with wave.open(str(session/'raw.wav')) as source:
        if (source.getframerate(), source.getnchannels(), source.getsampwidth()) != (16000, 1, 2):
            raise ValueError('Expected ST7-style PCM16 mono 16 kHz raw capture')
        raw = np.frombuffer(source.readframes(source.getnframes()), dtype='<i2')
    blocks = {b['block']: b for b in json.loads((session/'blocks.json').read_text())}
    events = [json.loads(line) for line in (session/'strikes.jsonl').read_text().splitlines()]
    planned = []
    for item in review['examples']:
        if not item.get('accepted', False):
            continue
        block = blocks[item['block']]
        if not 10 <= block['depth_deg'] <= 12:
            raise ValueError('Sub-10 or over-12 attempts cannot enter this dataset')
        onsets = sorted(float(t) for t in item['onsets_raw_seconds'])
        if not onsets or len(onsets) > block['strikes'] or any(not np.isfinite(t) for t in onsets):
            raise ValueError('Invalid audio onset annotation')
        if len(onsets) != block['strikes']:
            raise ValueError('Ambiguous/missing impacts: retain the block in the archive, not as a mislabeled positive')
        if any(b-a < .08 for a, b in zip(onsets, onsets[1:])):
            raise ValueError('Duplicate/too-close onset annotations require review')
        lead = float(item.get('lead_in_seconds', 2.))
        tail = float(item.get('tail_seconds', 4.5))
        if not 1. <= lead <= 3. or not 2. <= tail <= 6.:
            raise ValueError('Invalid reviewed clip margins')
        start = max(0, round((onsets[0]-lead)*16000))
        end = min(len(raw), round((onsets[-1]+tail)*16000))
        if not start < end or any(t*16000 < start or t*16000 >= end for t in onsets):
            raise ValueError('Annotation outside captured audio')
        identifier = session.name+f'_b{block["block"]:02d}'
        wav = output/'recordings'/f'r_{identifier}.wav'
        labels = output/'timestamps'/f't_{identifier}.csv'
        clip = raw[start:end]
        if np.mean(np.abs(clip.astype(np.int32)) >= 32767) > .001:
            raise ValueError('Excessively clipped recording requires review')
        # ST7 compares CSV and manifest floats exactly. Use the same six
        # decimal representation for both rather than unrounded subtraction.
        hits = [round(t-start/16000, 6) for t in onsets]
        peaks = [e['peak_deg'] for e in events if e['kind'] == 'closed' and e['block'] == block['block']]
        # ST7 derives r*/t* filenames from id.removeprefix('r').
        record = dict(id=wav.stem, wav=str(wav.relative_to(output)), csv=str(labels.relative_to(output)),
            session=session.name, group=session.name, split='unassigned', kind=block['kind'],
            commanded_depth_deg=block['depth_deg'], measured_peaks_deg=peaks, hits=hits,
            raw_offset_seconds=start/16000, duration=len(clip)/16000,
            review_method=review['method'], human_verified=bool(review.get('human_verified', False)),
            label_status='draft' if not review.get('human_verified', False) else 'human-reviewed',
            notes=item.get('notes', ''))
        planned.append((wav, labels, clip, record))
    targets = [p for wav, labels, _, _ in planned for p in (wav, labels)]
    if len(set(targets)) != len(targets) or any(p.exists() for p in targets):
        raise ValueError('Duplicate export or existing file; refusing overwrite')
    manifest_path = output/'manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else dict(
        version=1, grouping='physical-session', training_performed=False, records=[])
    previous_ids = {r['id'] for r in manifest['records']}
    if previous_ids & {record['id'] for _, _, _, record in planned}:
        raise ValueError('Recording already exists in manifest; refusing overwrite')
    for kind in ('recordings', 'timestamps'):
        (output/kind).mkdir(parents=True, exist_ok=True)
    records = []
    for wav, labels, clip, record in planned:
        with wave.open(str(wav), 'wb') as dest:
            dest.setparams((1, 2, 16000, 0, 'NONE', 'not compressed'))
            dest.writeframes(clip.astype('<i2').tobytes())
        with labels.open('w', newline='') as dest:
            writer = csv.writer(dest)
            writer.writerow(CSV_FIELDS)
            for hit in record['hits']:
                writer.writerow((wav.name, f'{hit:.6f}', 'cymbal_hit'))
        record['wav_sha256'] = hashlib.sha256(wav.read_bytes()).hexdigest()
        record['csv_sha256'] = hashlib.sha256(labels.read_bytes()).hexdigest()
        records.append(record)
    manifest['records'].extend(records)
    temporary = manifest_path.with_suffix('.tmp')
    temporary.write_text(json.dumps(manifest, indent=2)+'\n')
    temporary.replace(manifest_path)
    return records


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('session', type=Path)
    p.add_argument('review', type=Path)
    p.add_argument('--output', type=Path, default=ROOT/'X5data')
    args = p.parse_args()
    records = export(args.session, json.loads(args.review.read_text()), args.output)
    print(json.dumps(dict(recordings=len(records), labeled_onsets=sum(len(r['hits']) for r in records)), indent=2))


if __name__ == '__main__':
    main()
