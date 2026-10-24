"""Independent exact-law/geometry comparison; no optimizer or cache is used."""
import json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import ROOT,atomic_json,digest
from drum_planner.timing import core,motion_quality
from drum_planner.timing.benchmark import validate
base=ROOT/'diagnostics/fast_gpu_20261024';rows=[]
old=base/'old_cpu/plan'
if not (old/'plan.json').exists():old=ROOT/'diagnostics/gpu_cpu_full_20261024/cpu/plans/workspace_curve'
for leg in ('checkpoint_1','checkpoint_2','return_to_start'):
 new=json.loads((base/'verified3'/leg/'plan.json').read_text());ref=json.loads((old/leg/'plan.json').read_text())
 src=dict(path_waypoints=new['path_waypoints'],timing=new['source_timing']);oldsrc=dict(path_waypoints=ref['path_waypoints'],timing=ref['source_timing'])
 law=new['timing'];peak,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']))
 assert core.scale_needed(peak)<=1
 assert core.identity(src)==core.identity(oldsrc)
 quality=motion_quality.law_metrics(law);before=motion_quality.law_metrics(ref['timing'])
 assert not quality['internal_stops'] and not quality['deep_slowdowns']
 rows.append(dict(leg=leg,same_geometry=True,same_source_clock=new['source_timing']==ref['source_timing'],
     old_movement_s=ref['duration_outward_s'],new_movement_s=law['duration_s'],ratio=law['duration_s']/ref['duration_outward_s'],
     before_quality=before,after_quality=quality,continuous_peaks=peak.tolist(),profile=new['experiment']['profile'],
     checked_samples=new['sampled_interpolated_checks']))
left=json.loads((base/'left2/plan.json').read_text());lref=json.loads((ROOT/'diagnostics/gpu_timing_20261024/benchmark-2/left_workspace/reference.json').read_text())
lsource=dict(path_waypoints=left['path_waypoints'],timing=left['source_timing'])
assert core.identity(lsource)==lref['source_path_sha256']
left_report=dict(same_geometry=True,old_movement_s=lref['duration_s'],new_movement_s=left['duration_outward_s'],
    ratio=left['duration_outward_s']/lref['duration_s'],after_quality=motion_quality.law_metrics(left['timing']))
variations=[]
for name in ('right-minus','right-plus','left-minus','left-plus'):
 obj=json.loads((base/name/'plan.json').read_text());law=obj['timing'];quality=motion_quality.law_metrics(law)
 assert quality['internal_stops']==quality['deep_slowdowns']==0
 variations.append(dict(name=name,cold=json.loads((base/name/'cold.json').read_text()),movement_s=law['duration_s'],quality=quality,
     checked_samples=obj['sampled_interpolated_checks']))
summary=dict(right_legs=rows,left=left_report,variations=variations,
 cold=[json.loads((base/name/'cold.json').read_text()) for name in ('verified1','verified2','verified3')],
 cold_job_wall_s=[json.loads((base/(name+'-job')/'job.json').read_text())['elapsed_s'] for name in ('verified1','verified2','verified3')],
 cpu_fallback_job_wall_s=json.loads((base/'verified-cpu-job/job.json').read_text())['elapsed_s'],
 cpu_fallback_cold=json.loads((base/'verified-cpu/cold.json').read_text()),
 full_launch=json.loads((base/'launch2/cold_launch.json').read_text()),
 old_cpu= json.loads((base/'old_cpu/elapsed.json').read_text()) if (base/'old_cpu/elapsed.json').exists() else None,
 old_reference=str(old),software_only=True,motor_commands=0,camera_access=False)
atomic_json(base/'summary.json',summary)
print(json.dumps({k:v for k,v in summary.items() if k not in ('right_legs','variations','left')},indent=2))
