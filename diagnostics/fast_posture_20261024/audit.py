"""No-device provenance/preservation audit for --fast implementation and study."""
import json,hashlib,subprocess
from pathlib import Path
ROOT=Path.cwd();HERE=ROOT/'diagnostics/fast_posture_20261024'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
before=read(HERE/'before_hashes.json')
changed=[p for p,h in before.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
allowed={'drum_planner/cli.py','config/git_retained_artifacts.json','.gitignore'}
assert not set(changed)-allowed,changed
protected=[p for p in before if p.startswith(('drum_planner/experiment/','drum_planner/timing/','drum_planner/baseline/','destinations/','recordings/','left_recordings/','model/','camera_calibration/','calibration_sessions/'))]
assert not set(protected)&set(changed)
state=read(ROOT/'diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/measured_start.json')
checked=[]
for row in read(HERE/'verified/results.json'):
 assert row['returncode']==0,row
 plan=read(row['plan']);report=read(row['report']);req=read(Path(row['plan']).with_name('backend_request.json'))
 assert plan['status']=='planned' and plan['motion_authorized'] is False and plan['hardware_execution_available'] is False
 assert req['execute'] is False and req['start']==state[row['side']+'_rad']
 assert req['scene_robot_start']==state[row['side']+'_rad']
 passive='left' if row['side']=='right' else 'right'
 assert req[passive]==state[passive+'_rad']
 assert all(p['valid'] for p in report['task_proofs'])
 laws=([s['timing'] for s in plan['checkpoint_sequence']['segments']]+[plan['checkpoint_sequence']['return_leg']['timing']]
       if 'checkpoint_sequence' in plan else [plan['timing']])
 assert all(law['limits']==[.6,1.2,6.] for law in laws)
 assert plan['cycle'][0]['positions_rad']==plan['cycle'][-1]['positions_rad']==state[row['side']+'_rad']
 job=read(Path(row['plan']).parent/'job/job.json');limits=job['containment']
 assert job['exit_code']==0 and limits['MemoryMax']=='3221225472'
 assert 'AF_CAN' not in limits['RestrictAddressFamilies'].split()
 checked.append(dict(case=row['case'],passed=True,peak_memory_bytes=limits.get('MemoryPeak'),
                     native_fk_max_error_m=max(report['kinematics_native_parity'].values()),
                     max_tip_error_m=max(p['tip_error_m'] for p in report['task_proofs']),
                     task_constraint=report['task_constraint']))
assert len(checked)==9
assert all(r['passed'] for r in read(HERE/'ui_results.json'))
assert all(r['returncode']==0 for r in read(HERE/'cold_results.json'))
for row in read(HERE/'limit_followup/results.json'):
 assert row['installed'] is False and row['motor_commands']==0 and not row['warnings']
input_provenance=read(HERE/'inputs/provenance.json')
for p,meta in input_provenance.items():assert sha(ROOT/p)==meta['sha256'],p
result=dict(passed=True,existing_files_audited=len(before),modified_existing_files=changed,
 protected_files_unchanged=len(protected),experiment_program_unchanged=True,hardware_programs_unchanged=True,
 timing_program_unchanged=True,calibration_models_destinations_recordings_unchanged=True,
 existing_motion_limits_unchanged=True,hardware_access_this_task=False,motor_commands=0,
 fresh_camera_or_encoder_test_this_task=False,tests_used_explicit_saved_inputs=True,
 checked_cases=checked,native_ui_passed_sides=['right','left'],
 evidence_warning='Software and native UI only; no physical tracking, torque, thermal or acoustic validation',
 code_sha256={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'drum_planner/fast_mode').glob('*.py')})
(HERE/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
(HERE/'git_after.txt').write_text(subprocess.check_output(['git','status','--short']).decode())
print('PASS:',len(before),'existing files audited;',len(protected),'protected files unchanged; 9 real planner runs verified')
