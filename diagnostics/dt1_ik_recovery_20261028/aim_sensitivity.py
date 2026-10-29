"""NON-EXPORTABLE offline aiming sensitivity. No application policy changes."""
import json
from pathlib import Path
import numpy as np
from cymbal_destination import solver,aim
from drum_planner.experiment import config
from drum_planner.experiment.runtime import Oracle
config.no_can()
root=Path('planner_runs/generated-beat-20261028T083720-5ac2c7a8/plan-dd96e4f6/right/dt1/checkpoint_1')
r=json.loads((root/'backend_request.json').read_text());s=json.loads((root/'search.json').read_text());kin=solver.Kinematics(r['urdf'],'right')
out=Path('diagnostics/dt1_ik_recovery_20261028/aim_sensitivity');out.mkdir(exist_ok=True)
# Diagnostic process ONLY. The production constant is still 20 degrees.
solver.MAX_INWARD_YAW_DEG=25.;aim.MAX_INWARD_YAW_DEG=25.
oracle=Oracle(root/'backend_request.json',out)
try:
 rows,report=solver.candidates(kin,oracle,np.array(r['start']),np.array(s['target_m']),np.array(s['disc_normal']),np.array(s['inward_reference_world']),print)
 report.update(diagnostic_only=True,not_a_destination=True,motion_authorized=False,application_policy_unchanged=True)
 if rows:
  row=rows[0];report['example']=dict(q=row['q'].tolist(),geometry=row['geometry'],approach=solver.approach(oracle,r,row['q'],print))
 (out/'report.json').write_text(json.dumps(report,indent=2))
 print('CHECKED FEASIBLE AT 25:',bool(rows and report['example']['approach']['valid']),flush=True)
finally:oracle.close()
