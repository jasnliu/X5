"""Bounded serial saved-camera regression, never fresh hardware eligibility."""
import importlib.util,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tests'))
from drum_planner.common import atomic_json
from drum_planner.experiment.config import no_can
from drum_planner import planning
from hardware_current_native_check import run


def verify_left(base):
    old=json.loads((base/'old_left/hardware/plan.json').read_text());new=json.loads((base/'new_left/hardware/plan.json').read_text())
    assert old['outward']==new['outward']
    # Solver diagnostics contain measured calculation times; compare the actual
    # law/coefficients, samples and movement duration, not wall-clock metadata.
    assert {k:v for k,v in old['timing'].items() if k!='details'}=={k:v for k,v in new['timing'].items() if k!='details'}
    assert (base/'old_left/hardware/model/robot.urdf').read_bytes()==(base/'new_left/hardware/model/robot.urdf').read_bytes()
    atomic_json(base/'left_preservation.json',dict(same_model_bytes=True,same_timed_output=True,
        timing_diagnostic_difference='history solve seconds and total calculation_seconds only',motor_commands=0))


def main():
    no_can();base=Path(__file__).resolve().parent
    capture=ROOT/'planner_runs/20261028T011228Z-hardware-right-56a27e/calculate-20319199/capture'
    current=planning.generate
    spec=importlib.util.spec_from_file_location('drum_planner._j2_reference_model',base/'before/drum_planner/model.py')
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    cases=[('final_fault','right',['dt1'],capture,False),
        ('old_r1','right',['destinationR1'],capture,True),('new_r1','right',['destinationR1'],capture,False),
        ('old_r2','right',['destinationR2'],capture,True),('new_r2','right',['destinationR2'],capture,False),
        ('new_r1_r2','right',['destinationR1','destinationR2'],capture,False),
        ('old_left','left',['destinationL1'],None,True),('new_left','left',['destinationL1'],None,False)]
    results={}
    for name,side,selected,source,historical in cases:
        print('\nSTUDY CASE',name,'HISTORICAL NOMINAL LIMIT REFERENCE ONLY' if historical else 'CURRENT OPERATING LIMIT',flush=True)
        planning.generate=old.generate if historical else current
        paths=[x if x=='dt1' else str(ROOT/'destinations'/side/(x+'.json')) for x in selected]
        try:
            run(base/name,side,paths,source)
            results[name]=json.loads((base/name/'report.json').read_text())
        except Exception as exc:
            results[name]=dict(status='failed',error=str(exc));atomic_json(base/'study_results.json',results);raise
        finally:planning.generate=current
        atomic_json(base/'study_results.json',results)
    verify_left(base)


if __name__=='__main__':
    if sys.argv[1:]==['--verify-left-only']:verify_left(Path(__file__).resolve().parent)
    else:main()
