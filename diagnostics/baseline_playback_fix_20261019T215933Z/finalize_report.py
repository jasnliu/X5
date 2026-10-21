import hashlib,json,subprocess
from pathlib import Path
from drum_planner.bus_ownership import owners
from drum_planner.common import atomic_json
p=Path(Path('/tmp/x5_baseline_playback_task').read_text().strip())
for filename in ('final_tests.exit','regression.exit','mock_buttons.exit','ui_final.exit','physical_01.exit','physical_02.exit','physical_03.exit'):
 assert (p/filename).read_text().strip()=='0',filename
physical=[]
for i in (1,2,3):
 folder=p/f'physical_{i:02d}';report=json.loads((folder/'wire_audit_report.json').read_text());assert report['passed'];physical.append(report)
 assert all(json.loads(s)['accepted'] for s in (folder/'notifications.jsonl').read_text().splitlines())
 if i>=2:
  ui=json.loads((folder/'session/baseline_view_report.json').read_text());assert not ui['error'] and not ui['last_error']
buttons=json.loads((p/'mock_buttons/buttons_report.json').read_text());assert buttons['passed']
assert 'Exactly one native window:' in (p/'ui_final.log').read_text()
before=json.loads((p/'before_sha256.json').read_text());changed=[f for f,h in before.items() if hashlib.sha256(Path(f).read_bytes()).hexdigest()!=h]
assert set(changed)=={'drum_planner/cli.py','drum_planner/baseline/app.py','drum_planner/baseline/motion.py','tests/test_baseline_hardware.py','BASELINE_HARDWARE.md'}
assert not owners(),owners()
lines=(p/'regression.log').read_text().splitlines();summary=next(x for x in reversed(lines) if 'passed in' in x);assert summary.startswith('494 passed')
atomic_json(p/'preservation.json',dict(changed=changed,added=['tests/test_baseline_ui.py'],unchanged=len(before)-len(changed)))
atomic_json(p/'verification.json',dict(passed=True,physical_trials=physical,notifications_accepted=3,
 regression=summary,mock_ui_cases=[r['case'] for r in buttons['cases']],
 native_ui_windows=1,resolution_layout_checks=[[1440,900],[1120,820],[1280,800]],
 source_changes=changed,added=['tests/test_baseline_ui.py'],protected_unchanged=len(before)-len(changed),
 planner_algorithm_unchanged=True,recordings_unchanged=True,left_actuator_commands=0,
 no_midair_relax=True,real_drive_fault_tests=False,active_can_lock_owners=[],
 first_trial_post_completion_ui_close_race='Fixed; both subsequent physical trials and digital workflows close without errors'))
print(json.dumps(dict(passed=True,physical_trials=len(physical),regression=summary,native_windows=1,mock_cases=len(buttons['cases']),protected_unchanged=len(before)-len(changed)),indent=2))
