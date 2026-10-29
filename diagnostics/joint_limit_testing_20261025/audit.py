"""Current-task scope and saved-input CLI evidence audit; no devices."""
import ast,json,re,hashlib
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2];H=Path(__file__).resolve().parent;R=H/'release'
read=lambda p:json.loads(Path(p).read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
before=read(H/'before_hashes.json')
changed=[p for p,h in before.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
assert not set(changed)-{'.gitignore','config/git_retained_artifacts.json'},changed
rows=read(R/'cli_results.json');assert len(rows)==11 and all(r['passed'] for r in rows)
jobs=[]
for row in rows:
    logfile=R/(row['case']+'.log')
    path=re.findall(r'Contained software-only job: (.+)',logfile.read_text())[-1]
    job=read(Path(path)/'job.json');limits=job['containment']
    assert not job['cancelled'] and job['exit_code']==0
    assert limits['MemoryMax']==str(3*1024**3)
    assert 'AF_CAN' not in limits['RestrictAddressFamilies'].split()
    output=Path(row['command'][row['command'].index('--output')+1])
    assert read(output/'completion.json')['completed'] is True
    jobs.append(dict(case=row['case'],folder=path,**job))
counts=[]
for side in ('right','left'):
    c=read(R/('prepared_'+side)/'campaign.json');a=read(R/('analysis_'+side)/'report.json')
    forecast=read(R/('route_'+side)/'report.json')
    assert c['hardware_execution_available'] is False and c['recorded_world_only']
    assert all(v==sha(k) for k,v in c['input_files_sha256'].items())
    source=read(c['source_plan']);req=read(Path(c['source_plan']).parent/'backend_request.json')
    assert c['request']==req  # source's actual-acquisition self-mask/passive arm unchanged
    assert a['profile'] is None and not a['hardware_limits_known'] and a['synthetic']
    assert a['condition_gaps']==['payload_id','controller_id','supply_voltage_v']
    assert all(s['passed'] for s in a['scores'])
    assert forecast['geometric_path_unchanged'] and not forecast['motion_authorized']
    assert not forecast['hardware_execution_available'] and not forecast['result_cache_used']
    assert abs(forecast['predicted_movement_s']-source['duration_outward_s'])<1e-8
    counts.append(dict(side=side,prepared_trials=len(c['trials']),blocked_excursions=len(c['blocked']),
        synthetic_traces_passed=len(a['scores']),missing_cohorts=a['missing_coverage'],
        export_refused_for_unspecified_conditions=True,
        same_profile_route_duration_s=forecast['predicted_movement_s'],
        fresh_timing_validation_s=forecast['calculation_s']))
for name in ('demo_right','demo_left','demo_failure'):
    a=read(R/name/'analysis/report.json')
    assert a['complete_probe_coverage'] and a['synthetic'] and not a['hardware_limits_known']
    assert Path(a['profile']).name.startswith('SYNTHETIC_NOT_FOR_HARDWARE_')
    if name=='demo_failure':assert all(g['highest_complete_level']==.75 for g in a['groups'])
assert read(H/'cancel_result.json')['passed']
assert '107 passed' in (H/'tests_final.log').read_text()
sources=[*sorted((ROOT/'joint_limit_lab').glob('*.py')),ROOT/'test_joint_limits.sh',ROOT/'tests/test_joint_limit_lab.py',ROOT/'JOINT_LIMIT_TESTING.md']
for path in sources:
    if path.suffix=='.py':ast.parse(path.read_text(),filename=str(path))
result=dict(passed=True,software_only=True,hardware_commands=0,hardware_limits_measured=False,
    live_runner_implemented=False,existing_files_checked=len(before),authorized_existing_changes=changed,
    original_programs_data_calibration_unchanged=True,cli_cases=11,tests_passed=107,
    cases=counts,contained_jobs=jobs,cancel_cleanup_passed=True,
    sources={str(p.relative_to(ROOT)):sha(p) for p in sources})
(H/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('contained_jobs','sources')},indent=2))
