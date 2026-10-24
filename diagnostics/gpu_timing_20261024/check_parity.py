"""Saved-input comparison against unchanged hardware CPU results, never live."""
import json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import ROOT,atomic_json
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.experiment.config import no_can
import time
base=Path(__file__).parent;no_can();inputs=[]
for side,name in [('right','hardware-workspace-ui-right-verified-20261022'),('left','hardware-workspace-ui-left-20261022')]:
 obj=json.loads((ROOT/'planner_runs'/name/'verification.json').read_text())
 obj['rows']=[r for r in obj['rows'] if r['checkpoints']==1]
 folder=base/'parity-input'/side;folder.mkdir(parents=True,exist_ok=True)
 atomic_json(folder/'verification.json',obj);inputs.append(folder)
job=Job([PYTHON,ROOT/'tests/hardware_workspace_parity_check.py','--ui',*inputs,'--output',base/'parity-final'],base/'job-parity-final',timeout=600)
try:
 while job.poll() is None:time.sleep(.2)
 if job.poll():raise SystemExit(job.poll())
finally:job.close()
