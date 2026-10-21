import sys,json,os
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.experiment.app import wait
p=Path(__file__).resolve().parent
for stage in ('refine','heldout'):
 wait(Job([PYTHON,p/'evaluate.py',p/stage],p/(stage+'_precision_evaluation'),timeout=180))
 wait(Job([PYTHON,p/'verify_selected.py',p/stage,p/('verification_'+stage)],p/('verify_'+stage+'_precision_job'),timeout=900))
session=p/'ui_session_final'
wait(Job(['/usr/bin/python3','-m','drum_planner.cli','experiment','--headless','--method','all','--output',session],p/'cli_final_job',timeout=360))
os.environ['PYTHONPATH']=str(p.parents[1])+os.pathsep+os.environ.get('PYTHONPATH','')
wait(Job([PYTHON,'tests/check_gesture_ui.py',session,p/'ui_test'],p/'ui_final_job',timeout=1800))
wait(Job([PYTHON,p/'plot_results.py'],p/'plot_final_job',timeout=120))
print('FINAL PRECISION / CLI / UI / PLOTS PASSED',flush=True)
