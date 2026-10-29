import sys,argparse,json,time
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[2]),str(Path(__file__).resolve().parent)]
from drum_planner.common import atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.timing import core,motion_quality
from drum_planner.timing.benchmark import validate
from density_trial import solve
no_can();p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path,required=True);args=p.parse_args()
args.output.mkdir(parents=True,exist_ok=False);rows=[]
for cfg in json.loads(args.config.read_text()):
 for name in ('checkpoint_1','checkpoint_2','return_to_start','left'):
  data=json.loads((Path('diagnostics/fast_gpu_20261024/evidence')/(name+'.json')).read_text());s=data['source'];prior=data['law']
  print('START',cfg,name,flush=True);law=solve(s,**cfg);elapsed=law['details']['elapsed_s'];check=validate(law,s,None,collision=False)
  quality=motion_quality.law_metrics(law)
  row=dict(config=cfg,case=name,compute_s=elapsed,duration_s=law['duration_s'],ratio=law['duration_s']/prior['duration_s'],quality=quality,details=law['details'])
  rows.append(row);atomic_json(args.output/f'{len(rows):03d}-law.json',law);atomic_json(args.output/'report.json',rows)
  print('RESULT',name,round(elapsed,3),round(law['duration_s'],6),round(row['ratio'],6),law['details']['history'],flush=True)
