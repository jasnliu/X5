"""Read-only evidence audit; no devices, transport, acquisition or execution."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from drum_planner.common import atomic_json, read_state, endpoint
from drum_planner.experiment.motion_settings import FIELDS, identity
from drum_planner.timing import core

H=Path(__file__).resolve().parent
read=lambda p:json.loads(Path(p).read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
before=read(H/'before_hashes.json')
allowed={'.gitignore','config/git_retained_artifacts.json','AGENTS.md',
    'DRUM_PLANNER_EXPERIMENTS.md','drum_planner/timing/README.md','drum_planner/cli.py',
    'drum_planner/experiment/app.py','drum_planner/experiment/fast.py',
    'drum_planner/experiment/fast_config.py','tests/test_experiment_fast_timing.py',
    'tests/test_planner_experiment.py','tests/test_experiment_checkpoints.py',
    'tests/test_checkpoint_performance.py'}
changed=[p for p,v in before.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=v]
assert not set(changed)-allowed, sorted(set(changed)-allowed)

state=ROOT/'diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/measured_start.json'
scene=state.parent/'camera/scene.json'
cal=ROOT/'calibration_sessions/20261024T211629Z-1227b7b1-reprocessed/calibration.json'
actual=read_state(state)
rows=read(H/'headless_final_results.json')
left=H/'left_sequence_final'
left_result=read(left/'results.json')['workspace_curve']
rows.append(dict(case='left_sequence_custom',side='left',plan=left_result['plan'],passed=True))
paths=[];jobs=[];cases=[]
old_source=read(ROOT/'diagnostics/fast_posture_20261024/inputs/timing_sources.json')['old_workspace']
path_parity=[];independent_checks=0
for row in rows:
    assert row['passed'],row
    plan_path=Path(row['plan']);p=read(plan_path)
    folder=left if row['case']=='left_sequence_custom' else H/'runs_final'/row['case']
    context=read(folder/'context.json');side=row['side'];other='left' if side=='right' else 'right'
    profile=context['motion_settings'];limits=np.array([profile[k] for k in FIELDS])
    start=context['experiment_start']
    spawn=(actual[side+'_rad'] if start['kind']=='default' else
           endpoint(start['goal_recording'],side=side).tolist())
    assert not p['motion_authorized'] and not p['hardware_execution_available']
    assert p['can_access'] is False and p['motor_commands']==0
    assert p['calibration_sha256']==sha(cal)
    assert p['experiment']['motion_settings_sha256']==identity(profile)
    assert not p['experiment']['result_cache_used']
    req=read(plan_path.parent/'backend_request.json')
    assert req['start']==spawn and req['scene_robot_start']==actual[side+'_rad']
    job=read(plan_path.parent/'job.json');contain=job['containment']
    assert job['exit_code']==0 and not job['cancelled']
    assert int(contain['MemoryMax'])==3*1024**3
    assert 'AF_CAN' not in contain['RestrictAddressFamilies'].split()
    jobs.append(dict(case=row['case'],**job))
    if row['case']=='left_default_blocked':
        assert p['status']=='no_feasible_route'
        cases.append(dict(case=row['case'],passed=True,expected_status=p['status']))
        continue
    assert p['status']=='planned'
    assert p['cycle'][0]['positions_rad']==p['cycle'][-1]['positions_rad']==spawn
    seq=p.get('checkpoint_sequence')
    if seq:
        assert seq['hold_at_each_checkpoint_s']==2
        assert all(x['hold_s']==2 for x in seq['segments'])
        assert seq['return_leg']['goal']==spawn
        legs=[Path(s['plan']) for s in seq['segments']]+[Path(seq['return_leg']['plan'])]
        movement=seq['movement_outward_s'];back=seq['return_leg']['timing']['duration_s']
    else:
        assert p['hold_at_goal_s']==2
        legs=[plan_path];movement=back=p['duration_outward_s']
    for lp in legs:
        obj=read(lp);rq=read(lp.parent/'backend_request.json');law=obj['timing']
        assert rq['scene_robot_start']==actual[side+'_rad']
        assert rq[other]==actual[other+'_rad']
        assert obj['model']==p['model']
        assert law['limits']==limits.tolist()
        assert not law['details']['result_cache_used']
        assert np.all(np.array(law['peaks'])<=limits)
        if not law.get('stationary'):
            source=dict(timing=obj['source_timing'],path_waypoints=obj['path_waypoints'],request=rq)
            assert law['path']==core.encode(core.original(source))
            peaks,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']))
            assert np.all(peaks<=limits*(1+1e-8))
            np.testing.assert_allclose(peaks,law['peaks'],rtol=1e-7,atol=1e-8)
            independent_checks+=1
            if side=='right' and rq['start']==endpoint(H/'inputs/R1.json').tolist() and rq['goal']==endpoint(H/'inputs/R2.json').tolist():
                assert obj['source_timing']==old_source['timing']
                assert obj['path_waypoints']==old_source['path_waypoints']
                path_parity.append(str(lp))
        paths.append(str(lp))
    cases.append(dict(case=row['case'],passed=True,status=p['status'],movement_s=movement,
                      return_s=back,legs=len(legs)))

ui=read(H/'ui_results.json');assert len(ui)==2 and all(r['passed'] for r in ui)
assert all(r['default_start_target_checked'] and r['external_targets_preserved_on_recapture'] for r in ui)
cli_ui=read(H/'cli_ui_final/experiment_view_report.json')
assert cli_ui['generations']==1 and cli_ui['method_states']=={'workspace_curve':'ready'}
assert cli_ui['selected_checkpoints']==[str(H/'inputs/R2.json')]
assert cli_ui['simulated_start']['destination_source']==str(H/'inputs/R1.json')
assert cli_ui['mesh_builds']==1 and cli_ui['can_access'] is False and cli_ui['motor_commands']==0
math=read(H/'timing_final.json');assert len(math)==16 and all(r['verified'] for r in math)
assert len(path_parity)==4
supplemental=dict(left_sequence=cases[-1],external_path_cli_ui=dict(passed=True,
    selected_start='R1',selected_targets=['R2'],generations=1,scene_mesh_builds=1,
    plan=cli_ui['plan_file'],motor_commands=0))
atomic_json(H/'supplemental_results.json',supplemental)
legacy=[]
for name,previous in [('destinations/right/destinationR1.json','bd3b74c97588c948ab99118ae052c4c7910eab0ca48a3909a32ba5bd838c43a2'),
    ('drum_planner/common.py','079cccfdeca5a75b681bd40506bd04f06c16ca2d38a26bcb5d500cc702ef98f7')]:
    assert sha(ROOT/name)==before[name] and sha(ROOT/name)!=previous
    legacy.append(dict(file=name,historical_expected_sha256=previous,before_task_sha256=before[name],
                       after_task_sha256=sha(ROOT/name),changed_by_this_task=False))
atomic_json(H/'legacy_check_notes.json',dict(deselected_tests=[
    'tests/test_experiment_checkpoints.py::test_hardware_timing_generators_and_saved_data_unchanged',
    'tests/test_experiment_checkpoints.py::test_single_leg_worker_and_pool_identity_unchanged_without_checkpoint_fields',
    'tests/test_checkpoint_performance.py::test_policies_geometry_inputs_and_motor_control_are_unchanged',
    'tests/test_experiment_fast_timing.py::test_only_experiment_sources_changed_since_this_task',
    'tests/test_fast_gpu.py::test_protected_hardware_and_original_policies_unchanged'],
    reason='Historical preserved-byte fixtures predate the user reset of destinations and accepted calibration. Original failure logs retained; expectations not replaced to manufacture passes.',evidence=legacy))
inputs=[state,scene,cal,*sorted((H/'inputs').glob('*.json'))]
atomic_json(H/'provenance.json',dict(inputs={str(p.relative_to(ROOT)):sha(p) for p in inputs},
    saved_inputs=True,fresh_acquisition_this_task=False,physical_motion=False,
    source={str(p.relative_to(ROOT)):sha(p) for p in sorted((ROOT/'drum_planner/experiment').glob('*.py'))}))
result=dict(passed=True,preservation=dict(snapshot_files=len(before),unchanged_files=len(before)-len(changed),
    authorized_modified_files=changed,unexpected_changes=[]),headless_cases=cases,
    continuously_rechecked_nonstationary_laws=independent_checks,
    all_leg_actual_acquisition_masks_preserved=True,passive_arm_unchanged=True,
    source_geometry_bit_identical_to_prior_workspace=path_parity,
    job_containment=jobs,math_comparisons=16,native_ui_sides=['right','left'],
    external_path_cli_ui_passed=True,motor_commands=0)
atomic_json(H/'audit.json',result)
print(json.dumps({k:v for k,v in result.items() if k not in ('job_containment','source_geometry_bit_identical_to_prior_workspace')},indent=2))
