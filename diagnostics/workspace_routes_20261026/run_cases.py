"""Serial cold CLI comparisons on preserved real-camera inputs; no acquisition."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
D=Path(__file__).resolve().parent
INPUTS={
    'incident':ROOT/'planner_runs/20261026T040710Z-experiment-f81b35/acquisition',
    'second':ROOT/'planner_runs/20261026T033257Z-experiment-225d96/acquisition',
    'reference':ROOT/'diagnostics/r1_r2_wrist_speed_20261024/session/acquisition',
}

def run(tag, cases):
    output=D/tag;output.mkdir(exist_ok=False)
    results=[]
    for scene,side,start,targets in cases:
        label=scene+'_'+side+'_'+Path(start).stem+'_'+('-'.join(Path(t).stem for t in targets))
        folder=output/label;source=INPUTS[scene]
        cmd=['./plan_drums.sh','experiment','--'+side,'--offline',
             '--state',str(source/'measured_start.json'),'--scene',str(source/'camera/scene.json'),
             '--start',start,'--destinations',*targets,'--headless','--output',str(folder)]
        print('RUN',label,flush=True);began=time.monotonic()
        with (output/(label+'.log')).open('w') as log:
            done=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=240,
                env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'))
        row=dict(case=label,scene=scene,side=side,start=start,targets=targets,command=cmd,
                 elapsed_s=time.monotonic()-began,exit_code=done.returncode)
        if (folder/'results.json').is_file():
            result=json.loads((folder/'results.json').read_text())['workspace_curve'];p=Path(result['plan'])
            data=json.loads(p.read_text());row.update(status=data['status'],plan=str(p),
                movement_s=data.get('duration_outward_s'),error=data.get('error'),
                reason=data.get('details',{}).get('reason'))
            selected=p.parent/'selected_candidate.json'
            if selected.exists():
                c=json.loads(selected.read_text());row['candidate']={k:c.get(k) for k in ('index','source','score','workspace','metrics')}
        results.append(row)
        (output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
        print('RESULT',label,row.get('status'),row.get('movement_s'),'cold_s',row['elapsed_s'],flush=True)
    return results

if __name__=='__main__':
    cases=[(scene,'right','default',[target]) for scene in INPUTS for target in ('R1','R2')]
    if len(sys.argv)>2:cases=json.loads(Path(sys.argv[2]).read_text())
    run(sys.argv[1],cases)
