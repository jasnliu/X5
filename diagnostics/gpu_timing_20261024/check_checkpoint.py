"""Run the real experiment workspace method on saved R1 -> R4 checkpoints."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import ROOT,atomic_json
from drum_planner.experiment import worker,config
from drum_planner.experiment.checkpoints import ExperimentPlanDisplay
from drum_planner.timing import core
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();config.no_can()
original=ROOT/'planner_runs/20261024T002242Z-experiment-c440af'
context=json.loads((original/'context.json').read_text())
assert len(context['checkpoints'])==2
old=original/'plans/workspace_curve-66775b0c'
plan=json.loads((old/'plan.json').read_text())
params=plan['experiment']['checkpoint_reports'][0]['gesture_settings']
result=worker.plan(context,a.output,'workspace_curve',params,120)
assert result['status']=='planned' and not result['motion_authorized'] and result['motor_commands']==0
rows=[]
for name in ('checkpoint_1','checkpoint_2','return_to_start'):
    law=json.loads((a.output/name/'plan.json').read_text())
    prev=json.loads((old/name/'plan.json').read_text())
    meta=law['experiment']['retiming']
    assert meta['actual_backend']=='vulkan' and meta['duration_ratio']<=1.01
    assert law['final_curve_check']['samples_checked']>1
    assert law['outward'][0]['positions_rad']==prev['outward'][0]['positions_rad']
    assert law['outward'][-1]['positions_rad']==prev['outward'][-1]['positions_rad']
    x,y=core.decode(law['timing']['path']),core.decode(prev['timing']['path'])
    same=np.array_equal(x.x,y.x) and np.array_equal(x.c,y.c)
    rows.append(dict(leg=name,identical_to_saved_geometry=same,metadata=meta))
preview=ExperimentPlanDisplay(a.output/'plan.json')
assert preview.playable
for t in np.linspace(0,preview.duration,100):assert np.isfinite(preview.positions(t)).all()
assert np.array_equal(preview.positions(preview.duration),preview.req['start'])
assert result['experiment']['timing_workers']==2
assert 'GPU Vulkan' in result['experiment']['retiming']['label']
atomic_json(a.output/'verification.json',dict(passed=True,rows=rows,parallel_timing_workers=2,
    camera_access=False,motor_commands=0,exact_return_to_start=True,preview_duration_s=preview.duration))
print('PASS actual saved-input R1 -> R4 -> independently planned rest experiment',flush=True)
