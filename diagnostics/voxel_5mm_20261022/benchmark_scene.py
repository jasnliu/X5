"""Warm native load + endpoint checks only; not a trajectory-speed benchmark."""
import copy,json,time
from pathlib import Path
import numpy as np
from drum_planner import common,planning
from drum_planner.experiment.config import no_can
no_can()
D=Path(__file__).resolve().parent;rows={}
for label in ('10mm','5mm'):
    req=json.loads((D/'native'/label/'backend_request.json').read_text());samples=[]
    for i in range(3):
        t=time.monotonic();result=planning.backend(req,D/'warm'/label/str(i));samples.append(time.monotonic()-t)
    rows[label]=dict(seconds=samples,median_s=float(np.median(samples)),point_count=req['scene']['point_count'],
        world_padding=result['world_padding'],valid=[c['valid'] for c in result['checks']])
common.atomic_json(D/'warm_report.json',dict(software_only=True,scope='warm native process load and two endpoint checks; no trajectory speed claim',rows=rows))
print(rows,flush=True)
