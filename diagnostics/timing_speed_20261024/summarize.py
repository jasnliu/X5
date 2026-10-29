"""Independent post-run comparison, geometry identities and preservation audit."""
import json,sys,statistics
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));base=Path(__file__).resolve().parent
from drum_planner.common import digest,atomic_json
from drum_planner.timing import core,motion_quality
load=lambda p:json.loads(p.read_text())
rows=[]
for leg in ['checkpoint_1','checkpoint_2','return_to_start','left']:
 bp=base/('left-baseline' if leg=='left' else 'baseline1/'+leg)
 ap=base/('left-final2' if leg=='left' else 'final6/'+leg)
 before=load(bp/'plan.json');after=load(ap/'plan.json')
 br=load(bp/'backend_request.json');ar=load(ap/'backend_request.json')
 src=lambda p:dict(path_waypoints=p['path_waypoints'],timing=p['source_timing'])
 assert core.identity(src(before))==core.identity(src(after))
 assert before['path_waypoints']==after['path_waypoints'] and before['source_timing']==after['source_timing']
 assert before['experiment']['report']['selected']==after['experiment']['report']['selected']
 for key in ['start','goal','scene_robot_start','moving_side','left','right']:
  assert br.get(key)==ar.get(key),key
 for key in ['urdf','srdf']:assert digest(br[key])==digest(ar[key])
 assert br['scene']==ar['scene']
 law=after['timing'];peaks,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']))
 assert core.scale_needed(peaks)<=1
 np.testing.assert_allclose(peaks,law['peaks'],rtol=1e-8,atol=1e-9)
 quality=motion_quality.law_metrics(law);assert quality['internal_stops']==quality['deep_slowdowns']==0
 rows.append(dict(case=leg,old_movement_s=before['duration_outward_s'],new_movement_s=after['duration_outward_s'],
  reduction_percent=100*(1-after['duration_outward_s']/before['duration_outward_s']),same_exact_path=True,
  selected_candidate=after['experiment']['report']['selected'],continuous_peaks=peaks.tolist(),quality=quality,
  profile=after['experiment']['profile'],final_check=after['final_curve_check']))
summary=dict(movements=rows,right_movement_before_s=sum(r['old_movement_s'] for r in rows[:3]),
 right_movement_after_s=sum(r['new_movement_s'] for r in rows[:3]),software_only=True,camera_access=False,motor_commands=0,
 cold_before=[load(base/f'baseline{i}/cold.json')['elapsed_s'] for i in [1,2,3]],
 cold_after=[load(base/f'final{i}/cold.json')['elapsed_s'] for i in [4,5,6]],
 job_wall_before=[load(base/f'baseline{i}-job/job.json')['elapsed_s'] for i in [1,2,3]],
 job_wall_after=[load(base/f'final{i}-job/job.json')['elapsed_s'] for i in [4,5,6]],
 left_cold_before=load(base/'left-baseline/cold.json')['elapsed_s'],left_cold_after=load(base/'left-final2/cold.json')['elapsed_s'],
 full_cold_launch=load(base/'full-launch-final/cold_launch.json'),final_tradeoff=load(base/'final-tradeoff/report.json'))
atomic_json(base/'summary.json',summary)
before=load(base/'before_hashes.json');allowed={'drum_planner/experiment/fast.py','drum_planner/experiment/fast_config.py',
 'drum_planner/experiment/app.py','DRUM_PLANNER_EXPERIMENTS.md'}
changed=[]
for name,sha in before.items():
 if digest(ROOT/name)!=sha:
  assert name in allowed,name;changed.append(name)
new=['drum_planner/experiment/fast_timing.py','drum_planner/experiment/timing_extrema.py','tests/test_experiment_fast_timing.py',
 'tests/experiment_timing_preservation.json']
atomic_json(base/'preservation.json',dict(compared_files=len(before),changed_files=changed,unchanged_files=len(before)-len(changed),
 new_files={p:digest(ROOT/p) for p in new},unchanged_sha256={p:sha for p,sha in before.items() if p not in changed},
 hardware_unchanged=True,path_generator_unchanged=True,destinations_limits_and_calibration_unchanged=True))
print(json.dumps({k:v for k,v in summary.items() if k not in ['movements','final_tradeoff']},indent=2))
