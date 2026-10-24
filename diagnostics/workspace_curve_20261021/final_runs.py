"""Full public workers, same frozen inputs, all five methods, serial/no devices."""
import json
from pathlib import Path
from drum_planner.common import atomic_json,digest
from drum_planner.experiment import app,config

def main():
    config.no_can();doc=Path(__file__).parent
    cases=json.loads((doc/'cases.json').read_text())
    for case in cases:
        session=Path(case['session']);out={}
        resultfile=session/'final_results.json'
        if resultfile.exists():out=json.loads(resultfile.read_text())
        for method in ['workspace_curve','progress','sweep','single_arc','clearance']:
            if method in out:continue
            print('FINAL CASE',case['id'],method,flush=True)
            target,job=app.start_plan(session,method,config.settings(),120)
            app.wait(job)
            obj=json.loads((target/'plan.json').read_text())
            out[method]=dict(status=obj['status'],plan=str(target/'plan.json'))
            atomic_json(resultfile,out)
            print('FINAL RESULT',case['id'],method,obj['status'],obj.get('duration_outward_s'),flush=True)
    atomic_json(doc/'final_sources.json',{str(p):digest(p) for p in [Path('drum_planner/experiment/workspace_curve.py'),Path('drum_planner/experiment/config.py'),Path('drum_planner/experiment/worker.py')]})
    print('ALL FINAL WORKERS COMPLETE',flush=True)
if __name__=='__main__':main()
