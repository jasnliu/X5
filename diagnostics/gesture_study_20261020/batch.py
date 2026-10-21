import os,sys,json,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.experiment.app import wait
from drum_planner.common import atomic_json
p=Path(__file__).resolve().parent
stage=sys.argv[1];cases=json.loads((p/'cases.json').read_text())
if stage=='refine':work=[(c['id'],s) for c in cases if c['split']=='development' for s in (19,127,509)]
elif stage=='heldout':work=[(c['id'],s) for c in cases if c['split']=='heldout' for s in (19,127,509)]
else:raise ValueError(stage)
started=time.monotonic();done=[]
for case,seed in work:
 folder=p/stage/f'{case}-seed{seed}'
 if (folder/'result.json').exists():continue
 print(f'BATCH {stage} {len(done)+1}/{len(work)}: {case}, seed {seed}',flush=True)
 wait(Job([PYTHON,'-m','drum_planner.experiment.study','case','--case',case,'--seed',str(seed),'--stage','2','--seconds','120','--root',str(p/stage)],p/(stage+'_jobs')/f'{case}-{seed}',timeout=210))
 done.append([case,seed]);atomic_json(p/(stage+'_progress.json'),dict(done=done,elapsed_s=time.monotonic()-started))
wait(Job([PYTHON,p/'evaluate.py',p/stage],p/(stage+'_evaluation'),timeout=300))
print('BATCH COMPLETE',stage,flush=True)
