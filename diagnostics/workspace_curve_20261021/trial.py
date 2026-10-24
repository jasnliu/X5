"""Bounded software-only proposal screening; no timing policy changes."""
import argparse,json,time
from pathlib import Path
from drum_planner.common import ROOT,atomic_json
from drum_planner.experiment.config import no_can,settings
from drum_planner.experiment.runtime import Oracle
from drum_planner.experiment import workspace_curve

def main():
    no_can();p=argparse.ArgumentParser();p.add_argument('--label',default='initial');p.add_argument('--options',default='{}');p.add_argument('--case');p.add_argument('--seconds',type=float,default=120);a=p.parse_args()
    cases=json.loads((Path(__file__).parent/'cases.json').read_text());results=[]
    for case in cases:
        if a.case and case['id']!=a.case:continue
        session=Path(case['session']);refs=json.loads((session/'reference_results.json').read_text())
        request=Path(refs['single_arc']['plan']).parent/'backend_request.json';req=json.loads(request.read_text())
        folder=session/('workspace-'+a.label);folder.mkdir(exist_ok=True)
        began=time.monotonic()
        def log(msg):print(f'[{case["id"]}/{a.label} +{time.monotonic()-began:.1f}s] {msg}',flush=True)
        oracle=Oracle(request,folder)
        try:
            result=workspace_curve.build(oracle,req,folder,settings(),a.seconds,log,json.loads(a.options))
            result.update(case=case,source_request=str(request),source_sha=__import__('hashlib').sha256(Path(workspace_curve.__file__).read_bytes()).hexdigest())
            atomic_json(folder/'result.json',result)
            print('RESULT',case['id'],len(result['candidates']),result['elapsed_s'],flush=True)
            results.append(dict(case=case['id'],candidates=len(result['candidates']),result=str(folder/'result.json')))
        finally:oracle.close()
    atomic_json(Path(__file__).parent/('trial-'+a.label+'.json'),results)
if __name__=='__main__':main()
