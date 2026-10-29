"""Experiment movement optimization: original geometry, limits and safe fallback."""
import json
from pathlib import Path
import numpy as np
import pytest
from scipy.interpolate import PPoly
from drum_planner.common import ROOT,digest
from drum_planner.experiment import fast_timing as timing
from drum_planner.timing import core,motion_quality
from drum_planner.timing.benchmark import validate


def fixture(name='checkpoint_1'):
    return json.loads((ROOT/'diagnostics/fast_gpu_20261024/evidence'/f'{name}.json').read_text())


@pytest.mark.parametrize('name',['checkpoint_1','checkpoint_2','return_to_start','left',
                                 'right-minus','right-plus','left-minus','left-plus'])
def test_actual_sparse_improves_both_arms_without_geometry_or_limit_change(name):
    data=fixture(name);source=data['source'];logs=[];law=timing.optimize(source,logs.append)
    assert law['details']['actual_backend']=='cpu'
    assert law['details']['requested_backend']=='cpu_sparse'
    assert not law['details']['result_cache_used'] and not law['details']['cpu_reference_used']
    assert law['details']['selected_stage']=='sparse'
    assert any('intentional CPU backend' in line for line in logs)
    assert law['duration_s']<data['law']['duration_s']
    assert law['limits']==[.6,1.2,6.] and law['source_path_sha256']==core.identity(source)
    assert law['path']==core.encode(core.original(source))
    peaks,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']))
    np.testing.assert_allclose(peaks,law['peaks'],atol=1e-9,rtol=1e-8)
    assert core.scale_needed(peaks)<=1
    validate(law,source,None,collision=False)
    quality=motion_quality.law_metrics(law)
    assert quality['internal_stops']==quality['deep_slowdowns']==0


@pytest.mark.parametrize('failure',['missing','nonfinite','worse'])
def test_bad_or_unavailable_refinement_keeps_checked_clock_and_reports(monkeypatch,failure):
    def solve(B,q,w,x):
        if failure=='missing':raise ImportError('test: CasADi unavailable')
        return np.full_like(x,np.nan if failure=='nonfinite' else 5.),dict(converged=True,status='test',iterations=0)
    monkeypatch.setattr(timing,'sparse_solve',solve)
    from drum_planner.timing.vulkan import fast as old
    monkeypatch.setattr(old,'Device',lambda *a:pytest.fail('Intentional CPU path opened GPU'))
    source=fixture()['source'];logs=[];law=timing.optimize(source,logs.append)
    assert law['details']['fallback_used'] and law['details']['selected_stage'] in ('seed','uniform')
    assert 'FALLBACK' in law['details']['backend_label']
    assert any('FALLBACK' in line for line in logs)
    assert law['duration_s']<=source['timing']['duration_s']*(1+1e-9)
    validate(law,source,None,collision=False)


def test_both_solvers_fail_still_validate_uniform_and_log(monkeypatch):
    monkeypatch.setattr(timing,'cpu_solve',lambda *a:(_ for _ in ()).throw(ValueError('seed failed')))
    monkeypatch.setattr(timing,'sparse_solve',lambda *a:(_ for _ in ()).throw(RuntimeError('solver failed')))
    source=fixture()['source'];logs=[];law=timing.optimize(source,logs.append)
    assert law['details']['selected_stage']=='uniform'
    assert len(law['details']['warnings'])==2 and any('CPU SEED FALLBACK' in s for s in logs)
    validate(law,source,None,collision=False)


def test_extrema_at_zero_never_add_negative_sample():
    path=core.original(fixture()['source'])
    knots=np.r_[np.zeros(4),.5,np.ones(4)]
    grid=timing.grid_for(knots,path,[np.nextafter(0.,-np.inf),0.,1.,np.nan])
    assert grid[0]==0 and grid[-1]==1 and np.isfinite(grid).all()


@pytest.mark.parametrize('coefficients',[[[1.,2.]], [[np.nan,1.]], [[-1.,1.]]])
def test_nonmonotone_discontinuous_nonfinite_clocks_rejected(coefficients):
    source=fixture()['source'];path=core.original(source)
    with pytest.raises(ValueError):timing.certify(path,PPoly(np.array(coefficients),[0.,.5,1.]),source)


def test_limits_and_source_validation_not_bypassed():
    source=fixture()['source'];source['timing']['stop_at_corners']=True
    with pytest.raises(ValueError,match='continuous Workspace'):timing.optimize(source)


def test_only_experiment_sources_changed_since_this_task():
    before=json.loads((ROOT/'tests/experiment_timing_preservation.json').read_text())
    allowed={'drum_planner/experiment/fast.py','drum_planner/experiment/fast_config.py',
             'drum_planner/experiment/app.py','DRUM_PLANNER_EXPERIMENTS.md',
             # Subsequent authorized startup acquisition change, NOT timing/path
             # changes. Its CLI/offline behavior has dedicated regression tests.
             'drum_planner/cli.py','DRUM_PLANNER.md',
             'tests/test_planner_arms.py','tests/test_planner_experiment.py',
             # Subsequent authorized per-joint experiment controls: docs only.
             # Hardware/core/legacy timing remain protected by both audits.
             'AGENTS.md','drum_planner/timing/README.md'}
    for path,sha in before.items():
        if path not in allowed:assert digest(ROOT/path)==sha,path


@pytest.mark.parametrize('seed',range(20))
def test_batched_roots_match_original_on_random_positive_splines(seed):
    from scipy.interpolate import BSpline
    from drum_planner.experiment.timing_extrema import density_peaks
    rng=np.random.default_rng(seed);n=8+seed%9
    knots=np.r_[np.zeros(4),np.linspace(0,1,n-2)[1:-1],np.ones(4)]
    h=PPoly.from_spline(BSpline(knots,rng.uniform(.1,5,n),3))
    path=core.original(fixture(['checkpoint_1','checkpoint_2','return_to_start','left'][seed%4])['source'])
    actual,_=density_peaks(path,h);expected,_=core.density_peaks(path,h)
    np.testing.assert_allclose(actual,expected,rtol=1e-8,atol=1e-8)


def test_batched_zero_polynomials_and_interior_nonmonotonicity():
    from drum_planner.experiment.timing_extrema import density_peaks
    zero=PPoly(np.zeros((1,1,7)),[0.,1.]);clock=PPoly(np.ones((1,1)),[0.,1.])
    peaks,_=density_peaks(zero,clock);assert np.array_equal(peaks,np.zeros((3,7)))
    h=PPoly(np.array([[1.],[-1.],[.24]]),[0.,1.])
    with pytest.raises(ValueError,match='Nonmonotone'):density_peaks(zero,h)


def test_batched_checker_eigensolver_failure_falls_back_to_original(monkeypatch):
    source=fixture()['source'];path=core.original(source)
    monkeypatch.setattr(timing,'density_peaks',lambda *a:(_ for _ in ()).throw(np.linalg.LinAlgError('test')))
    logs=[];law=timing.certify(path,PPoly(np.ones((1,1)),[0.,1.]),source,log=logs.append)
    assert 'original polynomial roots' in law['details']['continuous_checker']
    assert any('CONTINUOUS CHECK FALLBACK' in s for s in logs)
    validate(law,source,None,collision=False)


def test_refinement_is_bounded_and_log_does_not_promise_third_solve(monkeypatch):
    calls=[]
    def solve(B,q,w,x):
        calls.append(1)
        return np.full_like(x,.1),dict(converged=True,status='test',iterations=0)
    monkeypatch.setattr(timing,'sparse_solve',solve)
    logs=[];law=timing.optimize(fixture()['source'],logs.append)
    assert len(calls)==2
    assert sum('one bounded extrema refinement' in s for s in logs)==1
    assert law['details']['fallback_used']
