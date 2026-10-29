"""Device-free integrity checks for the completed, diagnostic-only study."""
import copy
from pathlib import Path
import numpy as np
import pytest
from study import HERE, ROOT, read, sha, configurations, artifact, guard
from drum_planner.timing import core


def results(): return read(HERE/'primary2/results.json')+read(HERE/'checks/results.json')


def test_decimal_case_names_do_not_overwrite_each_other():
    paths=[artifact(Path(x['name']),'.json') for x in configurations()]
    assert len(paths)==len(set(paths))==29
    assert artifact(Path('velocity_x1.1'),'.json')!=artifact(Path('velocity_x1.5'),'.json')


@pytest.mark.parametrize('event,args', [('socket.__new__',(None,29,0,0)),('open',('/dev/video0',)),('open',('/dev/ttyUSB0',))])
def test_hardware_guard_without_opening_a_device(event,args):
    with pytest.raises(PermissionError):guard(event,args)


def test_all_47_results_have_independent_continuous_checks_and_original_inputs():
    rows=results();assert len(rows)==47
    for r in rows:
        assert r['source_file_sha256']==sha(r['source_file'])
        source=read(r['source_file'])['source']
        assert r['source_path_sha256']==core.identity(source)
        assert r['same_exact_geometry'] and r['endpoint_velocity_acceleration_zero']
        assert r['motor_commands']==0 and not r['camera_access'] and not r['denied_device_attempts']
        assert not r['recording_used_for_route_generation'] and r['reserve']==core.RESERVE
        assert r['quality']['internal_stops']==0
        assert np.allclose(r['limits'],np.array([.6,1.2,6.])*r['factors'])
        assert np.all(np.array(r['original_checker_peaks'])<=np.array(r['limits'])[:,None]*(1+1e-9))
        assert r['combined_lower_bound_s']<=r['movement_s']
        assert not r['details']['result_cache_used'] and not r['details']['cpu_reference_used']


def test_exact_law_geometry_and_endpoint_values_not_just_report_flags():
    for folder in [HERE/'primary2',HERE/'checks']:
        for file in folder.glob('*.law.json'):
            law=read(file);row=read(Path(str(file).replace('.law.json','.json')))
            source=read(row['source_file'])['source'];path=core.original(source)
            assert law['path']==core.encode(path)
            e=core.Evaluator(law);q,v,a,_=e(np.array([0.,e.duration]))
            assert np.allclose(q,path([0.,1.]),atol=1e-10,rtol=0)
            assert np.max(abs(v))<1e-7 and np.max(abs(a))<1e-6
            assert law['diagnostic_only'] and law['motion_authorized'] is False


def test_homogeneous_time_scaling_and_monotone_sensitivity():
    rows=read(HERE/'primary2/results.json');by={r['name']:r for r in rows};baseline=by['baseline']['movement_s']
    for speed in [2,4,8]:
        assert by[f'time_compression_x{speed}']['movement_s']==pytest.approx(baseline/speed,rel=2e-5)
    for axis in ['velocity','acceleration','jerk']:
        values=[baseline]+[by[f'{axis}_x{x:g}']['movement_s'] for x in [1.1,1.5,2,4,8,64]]
        assert np.max(np.diff(values))<2e-5


def test_higher_resolution_does_not_overturn_main_findings():
    primary={r['name']:r for r in read(HERE/'primary2/results.json')}
    refined=[r for r in read(HERE/'checks/results.json') if r['refined']]
    assert len(refined)==8
    for r in refined:
        assert not r['details']['warnings']
        assert r['movement_s']<=primary[r['name']]['movement_s']
        assert 1-r['movement_s']/primary[r['name']]['movement_s']<.01


def test_targets_are_labeled_nonminimal_nonhardware_and_raw_recording_untouched():
    targets=read(HERE/'target_times.json')
    for r in targets:
        assert np.all(np.array(r['actual_same_clock_peaks'])<r['sufficient_limits_same_clock'])
        assert np.all(np.array(r['necessary_individual_lower_limits'])<r['sufficient_limits_same_clock'])
        assert 'not jointly sufficient' in r['necessary_limits_caveat']
        assert 'hardware-approved' in r['sufficient_limits_caveat']
    recording=read(HERE/'record4_analysis.json')
    assert recording['sha256']==sha(ROOT/'recordings/record4.json')
    assert recording['source']=='query-only disabled-motor encoder feedback'


def test_all_original_programs_inputs_calibration_and_installed_limits_preserved():
    before=read(HERE/'before_hashes.json')
    assert len(before)==386
    assert all(sha(ROOT/name)==value for name,value in before.items())
    assert np.array_equal(core.LIMITS,[.6,1.2,6.])


def test_systemd_studies_excluded_can_and_bounded_memory():
    for folder in ['primary2-job','checks-job','analysis-job']:
        result=read(HERE/folder/'job.json');limits=result['containment']
        assert result['exit_code']==0 and not result['cancelled']
        assert limits['MemoryMax']=='3221225472'
        assert 'AF_CAN' not in limits['RestrictAddressFamilies']
