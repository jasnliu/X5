"""Preserve all pre-existing bytes except the explicitly scoped wiring/docs/tests."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
base=Path(__file__).parent
before=json.loads((base/'before_hashes.json').read_text())
allowed={'.gitignore','config/git_retained_artifacts.json','AGENTS.md','BASELINE_HARDWARE.md',
 'DRUM_PLANNER.md','DRUM_PLANNER_EXPERIMENTS.md','drum_planner/timing/README.md',
 'drum_planner/experiment/app.py','drum_planner/experiment/worker.py','drum_planner/timing/dispatch.py',
 'tests/test_experiment_checkpoints.py','tests/test_sparse_hardware.py','tests/hardware_workspace_parity_check.py'}
changed=[];missing=[]
for name,sha in before.items():
 p=ROOT/name
 if not p.is_file():missing.append(name)
 elif hashlib.sha256(p.read_bytes()).hexdigest()!=sha:changed.append(name)
assert not missing,missing
assert not set(changed)-allowed,sorted(set(changed)-allowed)
protected=[p for p in before if p.startswith(('drum_planner/baseline/','model/','recordings/','left_recordings/','destinations/','calibration_sessions/'))]
assert not set(protected)&set(changed)
result=dict(passed=True,preexisting_files=len(before),unchanged_files=len(before)-len(changed),
 changed_existing_files=sorted(changed),protected_hardware_model_capture_files=len(protected),
 no_baseline_or_actuator_source_changes=True,no_path_generator_or_cpu_solver_changes=True,
 no_existing_recording_destination_calibration_changes=True)
(base/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
