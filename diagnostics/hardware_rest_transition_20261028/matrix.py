"""Serial no-device native parity matrix on saved real camera captures."""
from pathlib import Path
import time,json
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.common import ROOT,atomic_json
D=ROOT/'diagnostics/hardware_rest_transition_20261028/verified'
D.mkdir(exist_ok=True)
latest='planner_runs/20261028T023247Z-hardware-right-ae273b/calculate-7f013e4d/capture'
earlier='planner_runs/20261028T023205Z-hardware-right-98a266/calculate-5b1b95c4/capture'
positive='planner_runs/20261028T011228Z-hardware-right-56a27e/calculate-20319199/capture'
cases=[('latest_dt1','right',latest,['dt1']),('earlier_dt1','right',earlier,['dt1']),('latest_r1','right',latest,['destinationR1']),
 ('latest_r2','right',latest,['destinationR2']),('latest_r1_r2','right',latest,['destinationR1','destinationR2']),
 ('latest_dt1_r2','right',latest,['dt1','destinationR2']),('positive_dt1','right',positive,['dt1']),
 ('left_l1','left',latest,['destinationL1'])]
rows=[]
for name,side,capture,goals in cases:
 print('\nSTART',name,flush=True)
 job=Job([PYTHON,'tests/hardware_current_native_check.py','--folder',D/name,'--side',side,'--capture',ROOT/capture,*goals,*(['--independent-timing'] if len(goals)>1 else [])],D/(name+'_job'),timeout=240)
 try:
  while job.poll() is None:time.sleep(.3)
  rows.append(dict(name=name,exit_code=job.process.returncode))
  atomic_json(D/'matrix_status.json',rows)
  if job.process.returncode:raise RuntimeError(name+' failed')
 finally:job.close()
print('ALL NATIVE MATRIX CASES PASSED',flush=True)
