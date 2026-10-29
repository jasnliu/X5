import json,time,sys
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[2]),str(Path(__file__).resolve().parent)]
from drum_planner.timing import core
from drum_planner.common import atomic_json
from batch_extrema import peaks
import numpy as np
base=Path(__file__).resolve().parent;rows=[]
for folder in ['verify1','polynomial']:
 for f in sorted((base/folder).glob('*-law.json')):
  law=json.loads(f.read_text());p=core.decode(law['path']);h=core.decode(law['density'])
  st=time.monotonic();actual,where=peaks(p,h);fast=time.monotonic()-st
  st=time.monotonic();expected,_=core.density_peaks(p,h);slow=time.monotonic()-st
  err=float(np.max(abs(actual-expected)));np.testing.assert_allclose(actual,expected,rtol=1e-8,atol=1e-9)
  print(f,fast,slow,err,flush=True);rows.append(dict(file=str(f),fast_s=fast,slow_s=slow,max_error=err))
atomic_json(base/'batch_report.json',rows)
