"""Verify exported count, pairing, provenance, PCM, labels and held-out separation."""
from pathlib import Path
import csv,hashlib,json,wave
from collections import Counter
import numpy as np
from hihat_collection.plan import COUNTS
from hihat_collection.export import wav_read
root=Path(__file__).resolve().parents[2];d=root/'X5data/snare'
records=json.loads((d/'manifest.json').read_text())['records']
assert len(records)==60
assert sorted(p.name for p in (d/'recordings').glob('*.wav'))==sorted(f'r{i}.wav' for i in range(1,61))
assert sorted(p.name for p in (d/'timestamps').glob('*.csv'))==sorted(f't{i}.csv' for i in range(1,61))
assert Counter(r['kind'] for r in records)==COUNTS
assert sum(len(r['hits']) for r in records)==45
assert sum(bool(r['hits']) for r in records)==40
assert Counter(r['split'] for r in records)==dict(development=48,test=12)
seen={};labels=0;copied=0;duration=0;raw_cache={}
for n,r in enumerate(records,1):
 assert r['id']==f'r{n}' and r['csv']==f'timestamps/t{n}.csv'
 wav=d/r['wav'];ts=d/r['csv'];x=wav_read(wav);duration+=len(x)/16000
 assert hashlib.sha256(wav.read_bytes()).hexdigest()==r['wav_sha256']
 assert hashlib.sha256(ts.read_bytes()).hexdigest()==r['csv_sha256']
 with ts.open() as f:
  reader=csv.DictReader(f);assert reader.fieldnames==['recording_file','hit_time_seconds','label'];rows=list(reader)
 assert len(rows)==len(r['hits'])
 for row,hit in zip(rows,r['hits']):
  assert row['recording_file']==wav.name and row['label']=='snare_hit'
  assert float(row['hit_time_seconds'])==hit and 0<=hit<len(x)/16000
  labels+=1
 assert np.max(abs(x.astype(np.int32)))<32767
 if r['hits']:assert r['commanded_snare_degrees']==11
 group=r['group']
 assert group not in seen or seen[group]==r['split']
 seen[group]=r['split']
 if r.get('source_ride_id'):
  assert wav.read_bytes()==Path(r['source_capture']).read_bytes();copied+=1
 else:
  src=Path(r['source_capture'])/'raw.wav'
  if src not in raw_cache:raw_cache[src]=wav_read(src)
  a=round(r['raw_offset_seconds']*16000)
  assert np.array_equal(x,raw_cache[src][a:a+len(x)])
train={r['wav_sha256'] for r in records if r['split']=='development'}
test={r['wav_sha256'] for r in records if r['split']=='test'}
assert not train&test
result=dict(passed=True,clips=len(records),positive_clips=40,negative_clips=20,snare_labels=labels,
 category_counts=dict(Counter(r['kind'] for r in records)),split_counts=dict(Counter(r['split'] for r in records)),
 independent_groups=len(seen),cross_split_duplicate_audio=0,pcm_preserved=True,
 copied_existing_ride_negatives=copied,newly_recorded_exported_clips=60-copied,duration_s=duration,
 files_numbered='r1.wav-r60.wav / t1.csv-t60.csv',label_status='draft_visual_signal_review')
(root/'diagnostics/snare_data_collection/dataset_verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
