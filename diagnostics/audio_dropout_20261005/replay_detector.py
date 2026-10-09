"""Offline replay of an EXISTING pre-BPM physical recording, unchanged ST7 models.

Reproduces listener ring, 0.3-second effective updates, lookahead and event gating.
No microphone, speakers, motors, synthetic hits, training or threshold changes.
"""
import json,os,sys,time
from pathlib import Path
import numpy as np
import soundfile as sf
import torch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT.parent/'st7'))
from cymbal_detector.config import load_config
from cymbal_detector.detector import CymbalDetector
from cymbal_detector.decoder import decode_probabilities,EventDecoder
OUT=Path(__file__).resolve().parent
torch.set_num_threads(1)
p=ROOT/'diagnostics/hihat_sync_20261005/run_20261005_134848/raw.wav'
audio,sr=sf.read(p,dtype='float32');assert sr==16000
# Preserve two seconds before the original swing starts at ~42.43 seconds.
audio=audio[40*sr:84*sr]
summaries=[]
for instrument,version in [('ride','v2'),('hihat','hihat_v1')]:
 config=load_config(ROOT.parent/'st7/models'/version/'config.yaml')
 detector=CymbalDetector(config)
 rows=[];hits=[];last_finalized=0.;last_emitted=-float('inf')
 began=time.monotonic()
 for end in range(2*sr,len(audio)+1,4800):
  start=max(0,end-6*sr);wave=torch.from_numpy(audio[start:end].copy())
  probabilities=detector.probabilities(wave)
  events=decode_probabilities(probabilities,detector.decoder,detector.frame_hop_seconds)
  finalized=end/sr-.4
  fresh=[]
  for t,score in events:
   event_time=start/sr+t
   if last_finalized<event_time<=finalized and event_time-last_emitted>=detector.decoder.min_gap_seconds:
    fresh.append(dict(at=40+event_time,score=score));last_emitted=event_time
  times=start/sr+np.arange(len(probabilities))*.01
  selection=(times>last_finalized)&(times<=finalized)
  scores=probabilities[selection]
  d=EventDecoder(detector.decoder,detector.frame_hop_seconds);states=[]
  for i,score in enumerate(probabilities):
   d.feed(float(score),i);states.append(d.state)
  states=np.asarray(states)[selection]
  rows.append(dict(end=40+end/sr,finalized=40+finalized,hits=fresh,
     probability_min=float(scores.min()) if len(scores) else None,
     probability_max=float(scores.max()) if len(scores) else None,
     latched_fraction=float(np.mean(states=='latched')) if len(states) else None))
  hits.extend(fresh);last_finalized=finalized
 intervals=np.diff([h['at'] for h in hits])
 s=dict(instrument=instrument,source=str(p),source_excerpt=[40,84],model=version,
   decoder=detector.decoder.as_dict(),events=len(hits),events_per_5s=[sum(lo<=h['at']<lo+5 for h in hits) for lo in range(40,80,5)],
   maximum_event_gap_s=float(intervals.max()) if len(intervals) else None,
   high_scores_but_no_event_windows=sum(not r['hits'] and r['probability_max'] is not None and r['probability_max']>=detector.decoder.high_threshold for r in rows),
   long_latched_windows=sum(r['latched_fraction'] is not None and r['latched_fraction']>.95 for r in rows),
   inference_elapsed_s=time.monotonic()-began)
 (OUT/(instrument+'_replay_windows.json')).write_text(json.dumps(rows,indent=2)+'\n')
 summaries.append(s);print(json.dumps(s),flush=True)
(OUT/'replay_summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
