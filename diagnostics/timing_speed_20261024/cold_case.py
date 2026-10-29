"""Fresh actual worker; optionally load the exact pre-change experiment module."""
import sys,time,json,argparse,importlib.util
from pathlib import Path
BEGAN=time.monotonic();ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
p=argparse.ArgumentParser();p.add_argument('--baseline',action='store_true');p.add_argument('--context',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
from drum_planner.experiment.config import no_can,settings
no_can()
if a.baseline:
 name='drum_planner.experiment.fast'
 path=ROOT/'diagnostics/timing_speed_20261024/before/drum_planner/experiment/fast.py'
 spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
from drum_planner.experiment.worker import plan
from drum_planner.common import atomic_json,digest
if a.output.exists():raise ValueError('Fresh output folder required')
context=json.loads(a.context.read_text())
for key in ('state','scene'):assert digest(context[key])==context[key+'_sha256']
a.output.mkdir(parents=True);atomic_json(a.output/'context.json',context)
result=plan(context,a.output,'workspace_curve',settings(),60.)
atomic_json(a.output/'cold.json',dict(elapsed_s=time.monotonic()-BEGAN,status=result['status'],baseline=a.baseline,result_cache_used=False))
assert result['status']=='planned'
