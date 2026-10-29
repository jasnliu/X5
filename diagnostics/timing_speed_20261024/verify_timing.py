import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.experiment.fast_timing import optimize
from drum_planner.timing import core,motion_quality
from drum_planner.timing.benchmark import validate
import numpy as np
no_can();out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False);rows=[]
for f in sorted(Path('diagnostics/fast_gpu_20261024/evidence').glob('*.json')):
 if f.name=='index.json':continue
 data=json.loads(f.read_text());source=data['source'];law=optimize(source,lambda s:print(f.stem,s,flush=True))
 proof=validate(law,source,None,collision=False)
 peaks,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']))
 assert core.scale_needed(peaks)<=1
 np.testing.assert_allclose(peaks,law['peaks'],rtol=1e-8,atol=1e-9)
 assert law['source_path_sha256']==data['law']['source_path_sha256']
 quality=motion_quality.law_metrics(law);assert quality['internal_stops']==quality['deep_slowdowns']==0
 assert law['duration_s']<data['law']['duration_s']
 atomic_json(out/(f.stem+'-law.json'),law)
 rows.append(dict(name=f.stem,old_movement_s=data['law']['duration_s'],movement_s=law['duration_s'],
    compute_s=law['details']['calculation_seconds'],details=law['details'],quality=quality,proof=proof))
 atomic_json(out/'report.json',rows)
