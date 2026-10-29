"""Serial saved-input robustness checks; run inside the same no-CAN job wrapper."""
from pathlib import Path
import subprocess
import sys
import time
from study import HERE, configurations, read, save, artifact, sha

out=HERE/'checks';out.mkdir(exist_ok=False)
cfg={x['name']:x for x in configurations()}
cases=[]
for name in ['baseline','velocity_x2','acceleration_x2','jerk_x2','jerk_x8','all_x4','all_x16','time_compression_x8']:
    cases.append(('checkpoint_2',name,True))
for path in ['checkpoint_1','left']:
    for name in ['baseline','velocity_x2','acceleration_x2','jerk_x2','all_x4']:
        cases.append((path,name,False))
results=[]
for i,(path,name,refined) in enumerate(cases):
    target=out/(path+'_'+name+('_refined' if refined else ''))
    command=[sys.executable,str(HERE/'study.py'),'--worker','--name',name,'--path',path,
             '--factors',*map(str,cfg[name]['factors']),'--output',str(target)]
    if refined:command.append('--refined')
    start=time.monotonic()
    with artifact(target,'.log').open('w') as log:
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
    row=read(artifact(target,'.json'));row['cold_process_including_validation_s']=time.monotonic()-start
    results.append(row);save(out/'results.json',results)
    print(f'[{i+1}/{len(cases)}] {target.name}: {row["movement_s"]:.6f}s motion; '
          f'{row["optimizer_s"]:.3f}s compute; flags={row["details"]["warnings"]}',flush=True)
save(out/'provenance.json',dict(followup_sha256=sha(__file__),study_sha256=sha(HERE/'study.py'),
    cases=cases,hardware_access=False))
