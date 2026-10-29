"""Repeat resolution comparison with the final, faster continuous checker."""
import sys,json
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[2]),str(Path(__file__).resolve().parent)]
from drum_planner.timing import core
from drum_planner.experiment.timing_extrema import density_peaks
from drum_planner.experiment.config import no_can
from drum_planner.common import atomic_json
from density_trial import solve
no_can();original=core.density_peaks
core.density_peaks=lambda p,h,**kw:density_peaks(p,h)
rows=[];out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
for n in [96,192,384]:
 for name in ['checkpoint_1','checkpoint_2','return_to_start','left']:
  data=json.loads((Path('diagnostics/fast_gpu_20261024/evidence')/(name+'.json')).read_text())
  law=solve(data['source'],method='ipopt',n=n,warm=True,refine=1,gridfactor=4,lobatto=True)
  elapsed=law['details']['elapsed_s'];peaks,_=original(core.decode(law['path']),core.decode(law['density']))
  assert core.scale_needed(peaks)<=1
  row=dict(n=n,case=name,compute_s=elapsed,duration_s=law['duration_s'],history=law['details']['history'])
  rows.append(row);atomic_json(out/f'{n}-{name}.json',law);atomic_json(out/'report.json',rows)
  print(row,flush=True)
