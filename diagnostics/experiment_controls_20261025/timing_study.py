"""Serial, device-free fixed-path timing comparison; never changes robot inputs."""
import sys,json,time,os
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from drum_planner.common import atomic_json
from drum_planner.experiment import fast_timing,motion_timing
from drum_planner.experiment.motion_settings import DEFAULT_LIMITS
from drum_planner.timing import core,motion_quality
H=Path('diagnostics/experiment_controls_20261025');rows=[]
tag=sys.argv[1] if len(sys.argv)>1 else os.environ.get('X5_MATH_RUN','')
report=H/('timing_'+tag+'.json' if tag else 'timing_study.json')
laws=H/('math_laws_'+tag if tag else 'math_laws')
sources={'current_R1_R2':json.loads(Path('diagnostics/fast_posture_20261024/inputs/timing_sources.json').read_text())['old_workspace']}
for name in ['checkpoint_1','return_to_start','left']:
 sources['historical_'+name]=json.loads((Path('diagnostics/fast_gpu_20261024/evidence')/(name+'.json')).read_text())['source']
for name,source in sources.items():
 for method in ['old','fast','balanced','thorough']:
  print('\nCASE',name,method,flush=True);t=time.monotonic()
  law=(fast_timing.optimize(source,print) if method=='old' else motion_timing.optimize(source,print,limits=DEFAULT_LIMITS,quality=method))
  elapsed=time.monotonic()-t
  peaks,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']))
  assert np.all(peaks<=DEFAULT_LIMITS*(1+1e-8))
  assert law['path']==core.encode(core.original(source))
  np.testing.assert_allclose(peaks,law['peaks'],rtol=1e-7,atol=1e-8)
  row=dict(case=name,method=method,duration_s=law['duration_s'],calculation_s=elapsed,quality=motion_quality.law_metrics(law),details=law['details'],verified=True)
  rows.append(row);atomic_json(report,rows)
  (laws/name).mkdir(parents=True,exist_ok=True);atomic_json(laws/name/(method+'.json'),law)
print('ALL PASSED',len(rows),flush=True)
