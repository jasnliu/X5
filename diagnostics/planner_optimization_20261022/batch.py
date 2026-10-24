"""Replay frozen selected curves through the real bounded timing pipeline.

Independent native full-world checks always run, including on math-cache hits.
No physical devices or fabricated motor authorization.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from verify import numerical
from drum_planner.common import atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.timing import pipeline
from drum_planner.baseline.curve import Curve
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--workers',type=int,default=2);p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--reuse-completed',action='store_true',help='Verify already-computed artifacts without solving again')
    a=p.parse_args();no_can();a.output.mkdir(parents=True,exist_ok=a.reuse_completed)
    base=ROOT/'planner_runs/20261022T215453Z-hardware-right-8993f5/calculate-2f665e35/plan/result'
    jobs=[];originals=[];names=['checkpoint_1','checkpoint_2','checkpoint_3','return_to_start']
    for name in names:
        obj=json.loads((base/name/'plan.json').read_text());originals.append(copy.deepcopy(obj))
        req=json.loads((base/name/'backend_request.json').read_text())
        winner=dict(path=obj['path_waypoints'],timing=obj['source_timing'],duration_s=obj['source_timing']['duration_s'])
        report=obj['experiment']['report'];ctx=json.loads((base/name/'context.json').read_text())
        ctx['pool_root']=str((a.output/'pool').resolve())
        folder=a.output/name;folder.mkdir(exist_ok=a.reuse_completed)
        if not a.reuse_completed:atomic_json(folder/'backend_request.json',req)
        for field in ('timing','source_timing','outward','cycle','retiming','final_curve_check'):
            obj.pop(field,None)
        jobs.append(pipeline.pending(obj,winner,report,req,ctx,folder,0.,hardware=True))
    if a.reuse_completed:
        results=[json.loads((a.output/name/'plan.json').read_text()) for name in names]
        elapsed=json.loads(Path(str(a.output)+'-job/job.json').read_text())['elapsed_s']
    else:
        began=time.monotonic();results=pipeline.finish_all(jobs,workers=a.workers);elapsed=time.monotonic()-began
    rows=[]
    for name,obj,old in zip(names,results,originals):
        reference=json.loads((a.reference/('right_'+name+'-law.json')).read_text())
        assert numerical(obj['timing'])==numerical(reference),name
        for key in ('outward','cycle','path_waypoints','source_timing','duration_outward_s'):
            assert obj[key]==old[key],(name,key)
        assert obj['final_curve_check']==old['final_curve_check']
        c=Curve(obj);times=np.r_[np.arange(0,c.duration,.002),c.duration]
        start=time.monotonic();scalar=np.array([c.at(t) for t in times]);scalar_s=time.monotonic()-start
        start=time.monotonic();batch=c.at_many(times);batch_s=time.monotonic()-start
        assert np.array_equal(scalar,batch)
        rows.append(dict(leg=name,elapsed_s=obj['retiming']['elapsed_s'],math_cache=obj['retiming']['math_cache'],
                         exact_law_and_samples=True,collision_check=obj['final_curve_check'],
                         scalar_evaluation_s=scalar_s,batch_evaluation_s=batch_s))
    atomic_json(a.output/'verification.json',dict(wall_s=elapsed,workers=a.workers,rows=rows,
        camera_access=False,motor_commands=0))
    print('BATCH EXACT PARITY PASS',elapsed,flush=True)


if __name__=='__main__':main()
