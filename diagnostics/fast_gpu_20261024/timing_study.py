import sys,time,json,argparse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.timing.vulkan.fast import optimize
from drum_planner.timing import benchmark,motion_quality,core
from drum_planner.experiment.config import no_can
from drum_planner.common import atomic_json
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--controls',type=int,default=32);p.add_argument('--blocks',type=int,default=64);p.add_argument('--output',type=Path,required=True);a=p.parse_args();no_can();a.output.mkdir(parents=True,exist_ok=False);rows=[]
root=Path('diagnostics/gpu_timing_20261024');inputs=root/'replay_inputs'
for r in json.loads((root/'benchmark-2/report.json').read_text())['rows']:
 name=r['case'];source=json.loads((inputs/(name+'.json')).read_text());ref=json.loads((root/'benchmark-2'/name/'reference.json').read_text())
 start=time.monotonic();law=optimize(source,print,controls=a.controls,first_blocks=a.blocks,later_blocks=a.blocks//2);elapsed=time.monotonic()-start
 proof=benchmark.validate(law,source,None,collision=False)
 actual,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']))
 assert np.allclose(actual,law['peaks'],rtol=1e-8,atol=1e-9) and core.scale_needed(actual)<=1
 before=motion_quality.law_metrics(ref);after=motion_quality.law_metrics(law)
 atomic_json(a.output/(name+'-law.json'),law)
 rows.append(dict(case=name,compute_s=elapsed,movement_s=law['duration_s'],ratio=law['duration_s']/ref['duration_s'],before=before,after=after,details=law['details']))
 atomic_json(a.output/'report.json',dict(rows=rows,controls=a.controls,blocks=a.blocks));print(name,'ratio',rows[-1]['ratio'],'s',elapsed,flush=True)
