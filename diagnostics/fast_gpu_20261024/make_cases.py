"""Saved-input variations only; never write user destination/capture files."""
import json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import atomic_json
from drum_planner.experiment.checkpoints import freeze
ROOT=Path.cwd();OUT=ROOT/'diagnostics/fast_gpu_20261024/cases';OUT.mkdir(exist_ok=True)
contexts={'right':ROOT/'planner_runs/20261024T002242Z-experiment-c440af/context.json',
          'left':ROOT/'planner_runs/gpu-20261024-ui-left/context.json'}
for side in contexts:
 base=json.loads(contexts[side].read_text())
 for name,delta in [('minus',-.01),('plus',.01)]:
  folder=OUT/(side+'-'+name);folder.mkdir(exist_ok=True)
  path=ROOT/'destinations'/side/('destinationR1.json' if side=='right' else 'destinationL1.json')
  obj=json.loads(path.read_text());obj['samples']=[obj['samples'][-1]]
  obj['samples'][0]['positions_rad'][0]+=delta
  atomic_json(folder/'endpoint.json',obj)
  ctx={k:v for k,v in base.items() if k not in ('checkpoints','checkpoint_sha256','destination_source','destination_sha256')}
  ctx.update(freeze([folder/'endpoint.json'],side,folder))
  atomic_json(folder/'context.json',ctx)
