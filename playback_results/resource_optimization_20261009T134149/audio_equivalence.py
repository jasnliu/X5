"""Read-only held-out replay vs the pre-change listener, including padding FPs."""
from pathlib import Path
import importlib.util,json,sys,time,statistics
import numpy as np
import torch
root=Path.cwd();audit=Path(sys.argv[1]);sys.path.insert(0,str(root/'tests'))
from test_shared_detector import bundles,replay
from cymbal_detector import listener
from cymbal_detector.data import load_audio
from cymbal_detector.detector import CymbalDetector
from cymbal_detector.normality import load_configured_normality
from cymbal_detector.shared_detector import SharedDetector
spec=importlib.util.spec_from_file_location('cymbal_detector.pre_optimization_listener',audit/'before/st7/cymbal_detector/listener.py')
legacy=importlib.util.module_from_spec(spec);sys.modules[spec.name]=legacy;spec.loader.exec_module(legacy)
torch.set_num_threads(1);torch.set_num_interop_threads(1)
selected=bundles();group=SharedDetector(selected)
separate={c['runtime']['instrument']:(c,CymbalDetector(c,ck),load_configured_normality(c,n)) for c,ck,n in selected}
paths={}
for version in ('ride_v3','hihat_v2','snare_v2'):
 for record in json.loads((root/'models'/version/'manifest.json').read_text())['records']:
  if record['split']=='test':paths[record['id']]=root/record['wav']
counts={name:0 for name in separate};results=[]
for record,path in sorted(paths.items(),key=lambda x:int(x[0][1:])):
 audio=load_audio(path);combined=replay(listener,group.config,group,None,audio,shared=True)
 for name,(config,detector,scorer) in separate.items():
  old=replay(legacy,config,detector,scorer,audio)
  expected=[e for e in old if e.get('event')=='HIT']
  actual=[e for e in combined if e.get('event')=='HIT' and e['instrument']==name]
  assert actual==expected,(record,name,'detections changed',expected,actual)
  old_progress=[e for e in old if e.get('event')=='AUDIO_PROGRESS']
  new_progress=[{k:v for k,v in e.items() if k not in ('instrument','label')} for e in combined if e.get('event')=='AUDIO_PROGRESS' and e['instrument']==name]
  assert new_progress==old_progress,(record,name,'watermarks changed')
  counts[name]+=len(actual)
 results.append(record);print('Exact streaming match:',record,flush=True)
# CPU-work comparison, warmed up and alternating order, identical 6-second input.
audio=load_audio(next(iter(paths.values())))
audio=torch.nn.functional.pad(audio,(0,max(0,96000-len(audio))))[:96000]
old_call=lambda:[d.detect_waveform(audio) for _,d,_ in separate.values()]
new_call=lambda:group.detect_waveform(audio)
old_call();new_call();times={'separate':[],'shared':[]}
for iteration in range(30):
 for name,fn in ([('separate',old_call),('shared',new_call)] if iteration%2 else [('shared',new_call),('separate',old_call)]):
  start=time.process_time();fn();times[name].append(time.process_time()-start)
report=dict(held_out_wavs=len(results),model_wav_comparisons=3*len(results),exact_hits_scores_normality_and_adc_times=True,exact_watermarks=True,events_including_padding=counts,cpu_median_ms={k:1000*statistics.median(v) for k,v in times.items()},cpu_p95_ms={k:1000*float(np.percentile(v,95)) for k,v in times.items()},files=results)
(audit/'audio_equivalence.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
