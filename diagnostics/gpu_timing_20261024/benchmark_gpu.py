import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.timing.vulkan.optimizer import optimize
from drum_planner.timing.vulkan import device
from drum_planner.timing import benchmark,core
from drum_planner.experiment.config import no_can
from drum_planner.common import atomic_json

def cases():
 root=Path.cwd();out={}
 # Retained post-study inputs make reruns independent of disposable UI reports.
 frozen=Path(__file__).parent/'benchmark-2'
 if (frozen/'report.json').exists():
  for row in json.loads((frozen/'report.json').read_text())['rows']:
   folder=frozen/row['case']
   replay=frozen.parent/'replay_inputs'/(row['case']+'.json')
   src=replay if replay.exists() else folder/'source.json'
   out[row['case']]=(json.loads(src.read_text()),json.loads((folder/'reference.json').read_text()))
  return out
 for name in ('rest_to_R1','R1_to_R4','R4_to_rest'):
  base=root/'diagnostics/vulkan_feasibility_20261023/inputs'
  out[name]=(json.loads((base/(name+'.json')).read_text()),json.loads((base/(name+'_reference_law.json')).read_text()))
 for p in (root/'diagnostics/planner_performance_20261021/inputs').glob('*.json'):
  src=json.loads(p.read_text());out[p.stem]=(src,src['existing_law'])
 p=root/'planner_runs/optimization-20261022-ui-left/verification.json'
 d=json.loads(p.read_text());folder=Path(d['rows'][0]['plan']).parent
 obj=json.loads((folder/'plan.json').read_text())
 out['left_workspace']=(dict(path_waypoints=obj['path_waypoints'],timing=obj['source_timing'],request=json.loads((folder/'backend_request.json').read_text())),obj['timing'])
 return out

if __name__=='__main__':
 no_can();p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--cases');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);rows=[]
 for name,(source,ref) in cases().items():
  if a.cases and name not in a.cases.split(','):continue
  folder=a.output/name;folder.mkdir();atomic_json(folder/'source.json',source);atomic_json(folder/'reference.json',ref)
  try:
   start=time.perf_counter();law=optimize(source,ref,log=lambda s:print(name,s,flush=True));gpu=time.perf_counter()-start
   start=time.perf_counter();proof=benchmark.validate(law,source,folder/'check');verify=time.perf_counter()-start
   atomic_json(folder/'law.json',law);atomic_json(folder/'proof.json',proof)
   row=dict(case=name,gpu_and_continuous_check_s=gpu,final_verification_s=verify,duration_s=law['duration_s'],reference_duration_s=ref['duration_s'],ratio=law['duration_s']/ref['duration_s'],before=law['details']['quality_before'],after=law['details']['quality_after'],status='pass')
  except Exception as e:
   import traceback;traceback.print_exc();row=dict(case=name,status='error',error=str(e))
  rows.append(row);atomic_json(a.output/'report.json',dict(rows=rows,motor_commands=0,camera_access=False,
   implementation_sha256=device.fingerprint(),device=device.probe(),
   cpu_reference_qualification_included=False,continuous_and_scene_checks=True));print('RESULT',row,flush=True)
 if any(r['status']!='pass' for r in rows):raise SystemExit(1)
