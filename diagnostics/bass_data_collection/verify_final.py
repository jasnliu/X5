"""Read-only audit of final data, captured evidence, and protected existing files."""
from pathlib import Path
from collections import Counter
import hashlib,json,wave
import numpy as np
from bass_collection.export import verify
from bass_collection.plan import PULSE
ROOT=Path(__file__).resolve().parents[2];base=ROOT/'diagnostics/bass_data_collection';out=ROOT/'X5data/bass'
read=lambda p:json.loads(p.read_text())
result=verify(out);manifest=read(out/'manifest.json');rows=manifest['records']
protected=read(base/'protected_before.json');changed=[];missing=[]
for name,expected in protected.items():
 p=Path(name)
 if not p.is_file():missing.append(name)
 elif hashlib.sha256(p.read_bytes()).hexdigest()!=expected:changed.append(name)
preservation=dict(protected_files=len(protected),changed=changed,missing=missing)
(base/'preservation_final.json').write_text(json.dumps(preservation,indent=2)+'\n')
assert not changed and not missing,preservation
sessions={};total_frames=0;peaks=[];clips_match=True
for r in rows:
 p=Path(r['source_capture'])
 if p.name not in sessions:
  session=read(p/'result.json');centers=read(p/'both_center_before_relax.json');acc=read(p/'hihat_result.json')
  assert session['success'] and session['relaxed_verified'] and not session['training_performed']
  assert not session.get('hardware_emergency') and not session['error'] and not session['recovery_error']
  assert read(p/'query_final_disabled.json')['all_16_disabled_fault_free']
  assert not read(p/'audio_result.json')['errors'] and acc['bass_pulse']==PULSE
  assert not acc['error'] and not acc['shutdown_error']
  if p.name!='20261010T232155-train-63812811':
   assert session['left_reverse_verified']
   reverse=read(p/'left_reverse_before_center.json');assert reverse['recording_start_verified']
   assert reverse['finished_at']<min(v['t'] for v in centers.values())
  for value in centers.values():
   assert value['max_error_deg']<=.2 and value['span_deg']<=.12 and value['settled_s']>=.6
  sessions[p.name]=dict(clips=session['clips'],bass_hits=session['bass_hits'],left_reverse_verified=session.get('left_reverse_verified',False),all_16_disabled_fault_free=True,center_before_relax=centers)
 with wave.open(str(out/r['wav']),'rb') as w:
  assert (w.getnchannels(),w.getsampwidth(),w.getframerate())==(1,2,16000)
  n=w.getnframes();data=w.readframes(n);total_frames+=n
  peak=max(abs(np.frombuffer(data,dtype='<i2').astype(np.int32)));peaks.append(int(peak))
  assert 0<peak<32767
 with wave.open(str(p/'raw.wav'),'rb') as w:
  w.setpos(round(r['raw_offset_seconds']*16000));assert w.readframes(n)==data
 assert len(r['hits'])==len(r['annotations'])
 assert all(0<=a<b<r['duration'] for a,b in zip(r['hits'],r['hits'][1:]))
 assert r['annotation_scope']=='bass_only'
audits={}
for name in ('physical_pilot2','physical_pilot3','physical_reverse_check','physical_main'):
 a=read(base/name/'can_audit.json');assert not a['motor_fault_frames'] and not a['unproven_center_disable_commands']
 audits[name]={k:a[k] for k in ('feedback_frames','motor_fault_frames','maximum_temperature_c','active_disable_commands','unproven_center_disable_commands')}
result.update(audio=dict(sample_rate=16000,channels=1,sample_width=2,seconds=total_frames/16000,maximum_peak_fraction=max(peaks)/32768,clipped_samples=0,export_bytes_match_raw_captures=True),split_counts=dict(Counter(r['split'] for r in rows)),left_reverse_first_clips=sum(r['left_return_policy']=='reverse_then_center' for r in rows),pre_instruction_pilot_clips=2,session_evidence=sessions,can_audits=audits,preservation=preservation,label_status='assistant_visual_signal_review',auditory_review_performed=False,independent_human_verification=False)
(base/'final_verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('session_evidence','can_audits')},indent=2))
