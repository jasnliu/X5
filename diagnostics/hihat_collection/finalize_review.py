"""Save explicit visual review decisions; no model or hardware access."""
from collections import Counter
import hashlib,json
from pathlib import Path
p=Path('diagnostics/hihat_collection/20261005T175549-54ef005a')
load=lambda name:json.loads((p/name).read_text())
examples=sorted(load('examples.json'),key=lambda v:v['block'])
commands=[json.loads(l) for l in (p/'hihat_commands.jsonl').read_text().splitlines()]
strikes=[json.loads(l) for l in (p/'strikes.jsonl').read_text().splitlines()]
last=max(v['end'] for v in examples)
close=[v for v in commands if v['kind']=='beat' and v['command']=='C']
assert len(examples)==40 and {v['block'] for v in examples}==set(range(1,41))
assert len(close)==34 and load('hihat_result.json')['closures_commanded']==34
assert Counter(v['block'] for v in close)==Counter({v['block']:v['closures'] for v in examples if v['closures']})
assert len([v for v in strikes if v['kind']=='release'])==10
assert all(v['t']>last for v in commands if v['kind']=='error')
assert all(v['t']>last for v in [json.loads(l) for l in (p/'events.jsonl').read_text().splitlines()] if v['phase']=='NOT RELAXED - ATTENTION REQUIRED')
assert all(10<=v['last_peak_deg']<=12 for v in examples if v['kind']=='mixed')
assert not load('audio_result.json')['errors']
assert load('query_final_disabled.json')['all_16_disabled_fault_free']
assert load('recovery_verified.json')['all_16_disabled']
accessory=load('hihat_result.json')
assert accessory['open_command_at']>last and accessory['release_ack'] and not accessory['shutdown_error']
recovery=dict(capture_completed_before_shutdown_issue=True,
    last_capture_monotonic=last,completed_new_examples=40,completed_closure_commands=34,completed_ride_releases=10,
    original_result_sha256=hashlib.sha256((p/'result.json').read_bytes()).hexdigest(),
    issues=[
      'All 40 capture blocks completed before cleanup errors. The original collector result remains success:false.',
      'During the subsequent arm return a small J7 overshoot near center triggered Joint limit violation; the center-before-relax guard prevented disabling without its measured settle proof.',
      'The hi-hat parent-heartbeat timeout occurred after the last captured block, during arm return. Its worker independently sent O, waited 1.5 seconds, sent S and received release acknowledgement. No encoder-position proof is available for the hi-hat.',
      'After ntfy warning, bounded near-center recovery verified center for 0.646 seconds, then relaxed the arm. A separate audited state-only query confirmed all 16 motors disabled and fault-free.',
      'Onset analysis and label export occurred only after this safe shutdown. Automatic cleanup code was then corrected and regression tested offline, without another physical batch.'
    ])
(p/'post_capture_recovery.json').write_text(json.dumps(recovery,indent=2)+'\n')
proposals=load('onset_proposals.json');offset=load('clock_summary.json')['offset'];items=[]
for row in examples:
 block=row['block'];onsets=[r['onset_raw_seconds'] for r in proposals if r['block']==block]
 assert len(onsets)==row['closures']
 if onsets:
  lead=1.5+((block-1)%5)*.125
  a,b=onsets[0]-lead,onsets[-1]+3.5
  assert a>=row['start']-offset and b<=row['end']-offset
  note=('Waveform, spectrogram and enlarged onset proposal visually reviewed. Timestamp marks the main broadband closure attack, not motor command/opening.'
        + (' Ride overlap retained as positive; no label added for the ride attack.' if row['kind']=='mixed' else ''))
 else:
  a=row['start']-offset+.01;b=a+5.98
  assert b<=row['end']-offset
  note='No closure scheduled. Full background waveform and spectrogram visually reviewed; incidental non-target noise retained.'
 items.append(dict(accepted=True,block=block,clip_start_seconds=round(a*16000)/16000,
                   clip_end_seconds=round(b*16000)/16000,onsets_raw_seconds=onsets,notes=note))
for id in load('plan.json')['plan']['ride_negative_ids']:
 items.append(dict(accepted=True,ride_id=id,notes='Previously collected ride-only source, preserved byte-for-byte. Full waveform/spectrogram reviewed. Target-specific negative CSV has no hi-hat labels.'))
review=dict(method='Offline WAV broadband/high-frequency onset proposals followed by assistant visual review of all 30 positive waveforms/spectrograms and all 34 enlarged onset windows; 30 negatives reviewed separately. No trained model or auditory review.',
 label_status='draft_visual_signal_review',human_verified=False,auditory_review_performed=False,
 target='hihat_close',motor_timestamps_are_labels=False,
 timing_note='Timestamps use a consistent main broadband attack convention. Six decimal places encode sample-based placement, not sub-millisecond semantic certainty; early contact/motor transients can differ by several milliseconds.',
 limitations=['Single new hi-hat recording session; no independent held-out session.','Ride-negative copies retain their original session group.','Mixed depth and relative command offset are paired, not a full factorial experiment.','Manual auditory review remains recommended before training.'],
 reviewed_plots=[f'review_page_{i}.png' for i in range(1,7)]+[f'onset_details_{i}.png' for i in range(1,6)]+['background_review.png','ride_negative_review_1.png','ride_negative_review_2.png'],
 examples=items)
(p/'review.json').write_text(json.dumps(review,indent=2)+'\n')
print('Explicit review saved:',len(items),'clips,',sum(len(i.get('onsets_raw_seconds',[])) for i in items),'onsets')
