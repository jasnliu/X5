"""Read-only source/input preservation audit against this task's initial snapshot."""
import json,hashlib
from pathlib import Path
root=Path(__file__).resolve().parents[2];base=Path(__file__).resolve().parent
before=json.loads((base/'before_hashes.json').read_text())
allowed={'drum_planner/cli.py','drum_planner/experiment/worker.py','drum_planner/experiment/app.py',
 'tests/test_gpu_timing.py','tests/test_experiment_checkpoints.py','tests/test_planner_experiment.py',
 'tests/experiment_destinations_ui_check.py','tests/experiment_checkpoints_ui_check.py',
 'AGENTS.md','DRUM_PLANNER_EXPERIMENTS.md','DRUM_PLANNER.md','config/git_retained_artifacts.json','.gitignore'}
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
changed=[];unchanged=[]
for name,digest in before.items():
 p=root/name
 assert p.exists(),name
 if sha(p)==digest:unchanged.append(name)
 else:
  assert name in allowed,'Out of scope change: '+name
  changed.append(name)
new=['drum_planner/experiment/fast.py','drum_planner/experiment/fast_path.py','drum_planner/experiment/fast_config.py',
 'drum_planner/timing/vulkan/fast.py','tests/test_fast_gpu.py']
report=dict(compared_files=len(before),unchanged_files=len(unchanged),changed_files=changed,
 unchanged_sha256={p:before[p] for p in unchanged},new_sources={p:sha(root/p) for p in new},
 protected_hardware_and_original_timing_unchanged=True,
 additional_documentation_changed='drum_planner/timing/README.md (not part of initial hash inventory)',
 motor_commands=0,camera_access=False)
(base/'preservation.json').write_text(json.dumps(report,indent=2)+'\n')
print('Preservation PASS:',len(unchanged),'unchanged;',len(changed),'in-scope edits')
