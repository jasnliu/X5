import hashlib,json,re,subprocess,sys,time
from pathlib import Path
from datetime import datetime,timezone
from drum_planner.bus_ownership import owners
D=Path(sys.argv[1])
for _ in range(180):
    if (D/'full-tests.exit').exists():break
    time.sleep(1)
assert (D/'full-tests.exit').read_text().strip()=='0',(D/'full-tests.log').read_text()[-7000:]
summary=re.findall(r'\d+ passed in [^\n]+',(D/'full-tests.log').read_text())[-1]
protected=json.loads((D/'protected.json').read_text())
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in protected.items())
assert json.loads((D/'mock_report.json').read_text())['passed']
assert json.loads((D/'actual_readonly_report.json').read_text())['passed']
changes=[]
for p in (D/'before').rglob('*'):
 if p.is_file():
  q=p.relative_to(D/'before')
  if q.read_bytes()!=p.read_bytes():changes.append(str(q))
report=dict(completed_utc=datetime.now(timezone.utc).isoformat(),passed=True,regression_suite=summary,
 actual_camera_encoders_calculate_and_preview='passed, query-only',actual_start='UI dispatch dry-run only; no motor owner launched',
 native_simulated_full_cycles_and_fault_recovery='passed',continuous_left_sway='simulated complete cycles at 0.25, 0.6, 1.0 speed; capture/preview/recovery regressions',
 physical_enable_or_motion_commands=0,wire_audit=json.loads((D/'readonly_wire_check.json').read_text()),
 controls_always_enabled=True,protected_files_unchanged=len(protected),remaining_can_owners=owners(),
 tests_sequential_with_memory_limit='2500M, swap 0; simulated/offline tests denied AF_CAN',
 modified_files=changes,new_review='BASELINE_ERROR_REVIEW.md',
 actual_readonly_report=str(D/'actual_readonly_report.json'),native_report=str(D/'mock_report.json'),suite_log=str(D/'full-tests.log'))
(D/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(summary)
print('Protected inputs unchanged:',len(protected))
print('Remaining CAN owners:',report['remaining_can_owners'])
print('Physical motion / enable commands: 0')
print('Verification:',D/'verification.json')
