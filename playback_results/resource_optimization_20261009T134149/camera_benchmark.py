import json,sys,time,resource
from pathlib import Path
import numpy as np
import cv2
import torch
from realtime_detector import load_model,InferenceWorker
from lighting_normalization import make_processor
out=Path(sys.argv[1]);root=Path.cwd()
frames=[]
normalizer=make_processor('local',root/'models/lighting_profile.json')
for video in ['20261009_111512_451599.mp4','20261009_093840_809799.mp4']:
 cap=cv2.VideoCapture(str(root/'recordings'/video))
 for index in [10,60,120,180]:
  cap.set(cv2.CAP_PROP_POS_FRAMES,index);ok,frame=cap.read()
  if ok:frames.append(normalizer.apply(frame))
 cap.release()
assert len(frames)>=6
worker=InferenceWorker(load_model(root/'models/snare.pt'),640,.5,'cpu')
worker.infer(frames[0])
reference=None;report=[]
for threads in [8,4,2,1,8,2]:
 torch.set_num_threads(threads);worker.infer(frames[0]);times=[];cpu=time.process_time();answers=[]
 for _ in range(3):
  for frame in frames:
   start=time.perf_counter();detections,_=worker.infer(frame);times.append(time.perf_counter()-start);answers.append(detections)
 cpu=time.process_time()-cpu
 if reference is None:reference=answers
 # Compare all public outputs, allowing only floating-point roundoff.
 def compare(a,b):
  if isinstance(a,dict):return set(a)==set(b) and all(compare(a[k],b[k]) for k in a)
  if isinstance(a,(list,tuple)):return len(a)==len(b) and all(compare(x,y) for x,y in zip(a,b))
  if isinstance(a,(int,float)):return abs(a-b)<1e-4
  return a==b
 result=dict(threads=threads,frames=len(times),median_ms=float(np.median(times))*1000,p95_ms=float(np.percentile(times,95))*1000,cpu_seconds=cpu,wall_seconds=sum(times),same_detections=compare(reference,answers),maxrss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
 report.append(result);print(json.dumps(result),flush=True)
(out/'camera_benchmark.json').write_text(json.dumps(report,indent=2)+'\n')
