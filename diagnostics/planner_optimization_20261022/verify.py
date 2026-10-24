"""Full-policy, device-free timing regression against the pre-change code.

Run inside the normal experiment Job containment; artifacts go in planner_runs.
"""
import argparse
import cProfile
import importlib
import json
from pathlib import Path
import sys
import time
import types

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
HERE = Path(__file__).resolve().parent
from drum_planner.common import atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.timing import adaptive, benchmark, core


def inputs():
    base = ROOT/'planner_runs/20261022T215453Z-hardware-right-8993f5/calculate-2f665e35/plan/result'
    sources = {}
    for leg in ('checkpoint_1', 'checkpoint_2', 'checkpoint_3', 'return_to_start'):
        p = base/leg
        obj = json.loads((p/'plan.json').read_text())
        sources['right_'+leg] = dict(path_waypoints=obj['path_waypoints'], timing=obj['source_timing'],
                                    request=json.loads((p/'backend_request.json').read_text()))
    for kind in ('single_arc', 'clearance'):
        p=ROOT/'diagnostics/corner_smoothing_20261021/selected'/('left_'+kind)/'source.json'
        sources['left_'+kind] = json.loads(p.read_text())
    return sources


def numerical(law):
    law=json.loads(json.dumps(law))
    law['details'].pop('calculation_seconds',None)
    for row in law['details'].get('tuning',[]): row.pop('elapsed_s',None)
    return law


def main():
    p=argparse.ArgumentParser();p.add_argument('--version',choices=['reference','current'],required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--reference',type=Path)
    p.add_argument('--cases');p.add_argument('--profile',action='store_true');a=p.parse_args()
    no_can();a.output.mkdir(parents=True,exist_ok=True)
    engine=adaptive
    if a.version=='reference':
        package=types.ModuleType('timing_before_20261022');package.__path__=[str(HERE/'reference')]
        sys.modules[package.__name__]=package
        engine=importlib.import_module(package.__name__+'.adaptive')
    rows={}
    for case,source in inputs().items():
        if a.cases and case not in a.cases.split(','):continue
        cfg=json.loads((HERE/'reference'/('selected_v2.json' if case.startswith('left') else 'selected.json')).read_text())
        print('CASE',case,flush=True);prof=cProfile.Profile();began=time.monotonic()
        if a.profile:prof.enable()
        law=engine.optimize(source,levels=cfg['levels'],ftol=cfg['ftol'],maxiter=cfg['maxiter'],log=lambda x:print(x,flush=True))
        if a.profile:prof.disable();prof.dump_stats(str(a.output/(case+'.prof')))
        elapsed=time.monotonic()-began
        proof=benchmark.validate(law,source,a.output/case)
        atomic_json(a.output/(case+'-law.json'),law)
        if a.reference:
            old=json.loads((a.reference/(case+'-law.json')).read_text())
            assert numerical(old)==numerical(law),(case,'numerical drift')
        rows[case]=dict(elapsed_s=elapsed,duration_s=law['duration_s'],proof=proof,
                        exact_reference_match=bool(a.reference))
        atomic_json(a.output/'report.json',rows)
        print('PASS',case,elapsed,flush=True)


if __name__=='__main__':main()
