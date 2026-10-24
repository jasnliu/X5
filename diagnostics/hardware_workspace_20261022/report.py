"""Collect small verification evidence; never retains owner IPC credentials."""
import copy,json,shutil
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2];D=Path(__file__).resolve().parent
sessions={'right':ROOT/'planner_runs/hardware-workspace-ui-right-verified-20261022',
          'left':ROOT/'planner_runs/hardware-workspace-ui-left-20261022'}
uis={};parity=[]
for side,folder in sessions.items():
    result=json.loads((folder/'verification.json').read_text())
    result['report']['owner'].pop('token',None)
    result['report']['owner'].pop('socket',None)
    uis[side]=result
    for name in ('controls','preview','live'):
        shutil.copy2(folder/(name+'.png'),D/(side+'_'+name+'.png'))
    parityfolder=ROOT/('planner_runs/hardware-workspace-parity-'+side+'-final-20261022')
    parity.extend(json.loads((parityfolder/'verification.json').read_text())['cases'])
verification=dict(software_only=True,camera_access=False,physical_motor_commands=0,
    ui=uis,experiment_parity=parity,
    regression_log='regression.log',
    same_generators_timing_models_zones_recordings=True,
    notes=['Right native UI reused current-turn generated plans after validating controller startup fix.',
           'Left has one saved destination; two small J1 variants were created ONLY inside its ignored test run.',
           'Exact full-mission native collision/zone proofs use unchanged approved margins.',
           'Simulated owner cycles are not evidence of physical motion performance.'])
(D/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
shutil.copy2('/tmp/x5-hardware-workspace-verified-regression.log',D/'regression.log')
