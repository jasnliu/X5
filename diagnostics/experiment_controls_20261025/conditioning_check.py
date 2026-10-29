import json,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from drum_planner.experiment.motion_timing import optimize,validate
from drum_planner.experiment.motion_settings import DEFAULT_LIMITS,matrix
from drum_planner.common import atomic_json
from drum_planner.timing import core
h=Path('diagnostics/experiment_controls_20261025');source=json.loads(Path('diagnostics/fast_posture_20261024/inputs/timing_sources.json').read_text())['old_workspace'];rows=[]
for name,limits in [('original',DEFAULT_LIMITS),('distal',matrix(json.loads((h/'inputs/right_distal.json').read_text())))]:
 for quality in ['fast','balanced','thorough']:
  t=time.monotonic();law=optimize(source,print,limits=limits,quality=quality);elapsed=time.monotonic()-t
  validate(law,source,None,collision=False);peaks,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']));assert np.all(peaks<=limits)
  rows.append(dict(name=name,quality=quality,duration_s=law['duration_s'],calculation_s=elapsed,details=law['details']))
  atomic_json(h/'conditioning_results.json',rows)
print('PASS',flush=True)
