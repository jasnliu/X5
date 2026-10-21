import sys,json,time,os
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.experiment.app import wait
p=Path(__file__).resolve().parent
while not (p/'heldout_evaluation/job.json').exists():time.sleep(5)
if json.loads((p/'heldout_evaluation/job.json').read_text())['exit_code']!=0:raise RuntimeError('Held-out evaluation failed')
for stage in ('refine','heldout'):
 print('VERIFY SELECTED:',stage,flush=True)
 wait(Job([PYTHON,p/'verify_selected.py',p/stage,p/('verification_'+stage)],p/('verify_'+stage+'_job'),timeout=1800))
print('ACTUAL CLI / DEFAULT FROZEN INPUTS / ALL SURVIVORS',flush=True)
session=p/'ui_session'
wait(Job(['/usr/bin/python3','-m','drum_planner.cli','experiment','--headless','--method','all','--output',session],p/'cli_job',timeout=360))
print('ACTUAL NATIVE GUI FEATURES / SAVED SCENE / OFFLINE RECAPTURE FIXTURE',flush=True)
os.environ['PYTHONPATH']=str(p.parents[1])+os.pathsep+os.environ.get('PYTHONPATH','')
wait(Job([PYTHON,'tests/check_gesture_ui.py',session,p/'ui_test'],p/'ui_job',timeout=1800))
print('ALL VALIDATION STAGES COMPLETE',flush=True)
