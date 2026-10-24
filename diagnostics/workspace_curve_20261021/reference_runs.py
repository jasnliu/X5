"""Freeze identical saved inputs; run the existing four methods without devices."""
import json,time
from pathlib import Path
from drum_planner.common import ROOT,atomic_json,digest,DEFAULT_CALIBRATION
from drum_planner.experiment import app,config,destinations
from drum_planner.experiment.worker import initialize

BASE=ROOT/'planner_runs/workspace-curve-study-20261021'
DOC=ROOT/'diagnostics/workspace_curve_20261021'

def main():
    config.no_can();BASE.mkdir(exist_ok=True)
    protected=[p for sub in ('drum_planner','destinations','recordings','left_recordings','gripper_models','right_zones','left_zones')
               for p in (ROOT/sub).rglob('*') if p.is_file() and p.suffix in ('.py','.cpp','.json','.stl','.STL') and '__pycache__' not in p.parts and 'build' not in p.parts]
    atomic_json(DOC/'before_hashes.json',{str(p.relative_to(ROOT)):digest(p) for p in protected})
    manifest=[]
    for side in ('right','left'):
        state,scene=config.saved_inputs(side)
        for destination in destinations.files(side):
            session=BASE/destination.stem;session.mkdir(exist_ok=True)
            ctx=initialize(session,state,scene,DEFAULT_CALIBRATION,side)
            ctx.update(destinations.freeze(destination,side,session));atomic_json(session/'context.json',ctx)
            manifest.append(dict(id=destination.stem,side=side,session=str(session),context=str(session/'context.json'),
                                 state_sha256=digest(state),scene_sha256=digest(scene),destination_sha256=digest(destination)))
    atomic_json(DOC/'cases.json',manifest)
    for case in manifest:
        print('REFERENCE CASE',case['id'],flush=True)
        session=Path(case['session']);outcomes={}
        for method in ('progress','sweep','single_arc','clearance'):
            target,job=app.start_plan(session,method,config.settings(),120)
            app.wait(job)
            obj=json.loads((target/'plan.json').read_text())
            outcomes[method]=dict(status=obj['status'],plan=str(target/'plan.json'))
            atomic_json(session/'reference_results.json',outcomes)
            print('REFERENCE RESULT',case['id'],method,obj['status'],obj.get('duration_outward_s'),flush=True)
    print('ALL REFERENCES COMPLETE',flush=True)

if __name__=='__main__':main()
