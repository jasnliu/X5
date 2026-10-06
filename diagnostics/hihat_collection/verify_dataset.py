"""Read-only final file/label/integrity checks; no hardware or training."""
import csv,hashlib,json,wave
from collections import Counter
from pathlib import Path
import numpy as np
root=Path.cwd();p=root/'diagnostics/hihat_collection/20261005T175549-54ef005a';data=root/'X5data/hihat'
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
protected=json.loads((root/'diagnostics/hihat_collection/setup_20261005/protected_before.json').read_text())
changed=[f for f,h in protected.items() if not Path(f).exists() or sha(Path(f))!=h]
assert not changed,changed
ride=json.loads((root/'diagnostics/hihat_collection/setup_20261005/ride_before.json').read_text())
ride_changed=[f for f,h in ride.items() if f!='README.md' and sha(root/'X5data/ride'/f)!=h]
assert not ride_changed,ride_changed
records=json.loads((data/'manifest.json').read_text())['records']
assert len(records)==60
assert set(f.name for f in (data/'recording').iterdir())=={f'r{i}.wav' for i in range(1,61)}
assert set(f.name for f in (data/'timestamp').iterdir())=={f't{i}.csv' for i in range(1,61)}
with wave.open(str(p/'raw.wav')) as w:raw=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2')
clipped=0;total=0.;positive=0;onsets=0;new_pcm_verified=0;copies=0
for r in records:
 wpath=data/r['wav'];cpath=data/r['csv']
 assert sha(wpath)==r['wav_sha256'] and sha(cpath)==r['csv_sha256']
 with wave.open(str(wpath)) as w:
  assert (w.getframerate(),w.getnchannels(),w.getsampwidth(),w.getcomptype())==(16000,1,2,'NONE')
  samples=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2')
 assert len(samples)/16000==r['duration']
 with cpath.open(newline='') as f:
  reader=csv.DictReader(f);assert reader.fieldnames==['recording_file','hit_time_seconds','label'];rows=list(reader)
 assert [float(v['hit_time_seconds']) for v in rows]==r['hits']
 assert all(v['label']=='hihat_close' and v['recording_file']==wpath.name for v in rows)
 assert all(0<=v<r['duration'] for v in r['hits'])
 assert all(b>a for a,b in zip(r['hits'],r['hits'][1:]))
 positive+=bool(rows);onsets+=len(rows);total+=r['duration']
 clipped+=int(np.count_nonzero(np.abs(samples.astype(np.int32))>=32767))
 if r['kind']=='ride_negative':
  assert wpath.read_bytes()==(root/'X5data/ride/recordings'/f"{r['source_ride_id']}.wav").read_bytes();copies+=1
 else:
  start=round(r['raw_offset_seconds']*16000)
  np.testing.assert_array_equal(samples,raw[start:start+len(samples)]);new_pcm_verified+=1
assert positive==30 and onsets==34 and clipped==0
result=dict(recordings=len(records),csv_files=len(records),positive_clips=positive,negative_clips=len(records)-positive,
  hi_hat_closure_labels=onsets,kind_counts=dict(Counter(r['kind'] for r in records)),duration_seconds=total,
  wav_format='16000 Hz mono signed PCM16',clipped_samples=clipped,raw_pcm_copies_verified=new_pcm_verified,
  ride_wav_byte_copies_verified=copies,protected_files_unchanged=len(protected),ride_data_and_manifest_unchanged=len(ride)-1,
  old_top_level_recording_directories_absent=not (root/'X5data/recordings').exists() and not (root/'X5data/timestamps').exists(),
  all_labels_visual_signal_review_only=True,training_performed=False)
(p/'dataset_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
