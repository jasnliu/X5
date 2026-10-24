"""Cold sparse timing, frozen selected paths, no sensors or motors."""
import sys,json,time,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from drum_planner.common import atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.timing import sparse,core,motion_quality,benchmark,integration

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--cases');a=p.parse_args()
 no_can();a.output.mkdir(parents=True,exist_ok=True);cfg=integration.policy();rows=[]
 base=ROOT/'planner_runs/20261022T215453Z-hardware-right-8993f5/calculate-2f665e35/plan/result'
 cases={}
 for leg in ('checkpoint_1','checkpoint_2','checkpoint_3','return_to_start'):
  folder=base/leg;obj=json.loads((folder/'plan.json').read_text())
  cases['right_'+leg]=(dict(path_waypoints=obj['path_waypoints'],timing=obj['source_timing'],request=json.loads((folder/'backend_request.json').read_text())),obj['timing'])
 for side in ('left',):
  ui=json.loads((ROOT/f'planner_runs/optimization-20261022-ui-{side}/verification.json').read_text())
  for row in ui['rows'][:1]:
   folder=Path(row['plan']).parent;obj=json.loads((folder/'plan.json').read_text())
   cases[side+'_workspace']=(dict(path_waypoints=obj['path_waypoints'],timing=obj['source_timing'],request=json.loads((folder/'backend_request.json').read_text())),obj['timing'])
 for name,(source,old) in cases.items():
  if a.cases and name not in a.cases.split(','):continue
  print('CASE',name,flush=True);folder=a.output/name;folder.mkdir(exist_ok=True);atomic_json(folder/'source.json',source)
  started=time.monotonic();law=sparse.optimize(source,levels=cfg['levels'],ftol=cfg['ftol'],maxiter=cfg['maxiter'],log=lambda s:print(s,flush=True));elapsed=time.monotonic()-started
  atomic_json(folder/'law.json',law);proof=benchmark.validate(law,source,folder/'check');atomic_json(folder/'proof.json',proof)
  before=motion_quality.law_metrics(old);after=motion_quality.law_metrics(law)
  rows.append(dict(case=name,elapsed_s=elapsed,old_duration_s=old['duration_s'],new_duration_s=law['duration_s'],geometry=proof['geometry'],before_quality=before,after_quality=after,collision_zone=proof['collision_zone']))
  atomic_json(a.output/'report.json',dict(rows=rows,motor_commands=0,camera_access=False));print('PASS',rows[-1],flush=True)
if __name__=='__main__':main()
