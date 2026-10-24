"""Fresh legacy CPU workspace only; original pre-GPU worker kept byte-identical."""
import sys,json,os,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import ROOT,atomic_json,digest
from drum_planner.experiment.runtime import PYTHON,Job
base=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else ROOT/'diagnostics/fast_gpu_20261024/old_cpu';base.mkdir(parents=True,exist_ok=False)
context=json.loads((ROOT/'planner_runs/20261024T002242Z-experiment-c440af/context.json').read_text())
context['pool_root']=str(base/'empty_pools');atomic_json(base/'context.json',context)
old=ROOT/'diagnostics/gpu_cpu_full_20261024/cpu_worker_before_gpu.py'
assert digest(old)=='dba10181a488b7d5f8758a2335d9b72a6a197358a1bb5813a44d0fe44daeead6'
command=['/usr/bin/env','X5_PLANNER_CACHE_DIR=off',str(PYTHON),str(old.with_name('cpu_entry.py')),
 'plan','--folder',str(base/'plan'),'--context',str(base/'context.json'),'--method','workspace_curve','--budget','60']
atomic_json(base/'command.json',dict(argv=command,original_worker_sha256=digest(old)))
start=time.monotonic();job=Job(command,base/'job',timeout=900)
try:
 while job.poll() is None:time.sleep(.1)
 if job.poll():raise RuntimeError('Legacy CPU calculation failed')
finally:job.close()
atomic_json(base/'elapsed.json',dict(elapsed_s=time.monotonic()-start))
