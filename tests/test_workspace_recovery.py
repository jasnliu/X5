"""Experiment-only path recovery; no device or motor access."""
import copy
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.interpolate import PPoly
from drum_planner.experiment import fast, workspace_recovery as recovery
from drum_planner.timing import core
from drum_planner.timing.rounding import as_source


class Kinematics:
    lower=np.full(7,-1.5)
    upper=np.full(7,1.5)
    def call(self,op,**kwargs):
        assert op=='fk'
        q=np.asarray(kwargs['q'])
        return dict(wrist_xyz_m=(q[:,:3]*.2).tolist(),
                    tip_xyz_m=(q[:,:3]*.2+np.sin(q[:,3:6])*.1).tolist())


def fixture():
    req=dict(start=[-.4,-.2,0.,.1,.2,.3,.4],goal=[.3,.1,.2,.3,-.2,.1,-.4])
    return recovery.CurveFit(Kinematics(),req,np.zeros((2,3))),req


@pytest.mark.parametrize('seed',range(8))
def test_entire_quintic_is_bounded_with_exact_rest_endpoints(seed):
    fit,req=fixture();rng=np.random.default_rng(seed)
    controls=fit.nodes(np.r_[rng.uniform(-1.5,1.5,fit.nq),np.zeros(6)])
    pp=fit.polynomial(controls);samples=pp(np.linspace(0,1,10001))
    assert recovery.position_bounds(pp,fit.oracle.lower,fit.oracle.upper)
    assert samples.min()>=-1.5 and samples.max()<=1.5
    np.testing.assert_allclose(pp(fit.s),fit.B@controls,atol=2e-14)
    np.testing.assert_allclose(pp([0.,1.]),[req['start'],req['goal']],atol=1e-14)
    for d in (1,2):np.testing.assert_allclose(pp([0.,1.],nu=d),0.,atol=2e-10)
    cuts=np.unique(pp.x)[1:-1]
    for d in range(5):np.testing.assert_allclose(pp(np.nextafter(cuts,-np.inf),nu=d),pp(cuts,nu=d),atol=1e-7)
    source=as_source(pp,dict(request=req));actual=core.original(source)
    np.testing.assert_allclose(actual(np.linspace(0,1,99)),pp(np.linspace(0,1,99)),atol=1e-14)
    assert source['timing']['stop_at_corners'] is False


def test_batched_jacobian_matches_finite_difference_for_new_basis():
    fit,_=fixture();x=fit.initial();analytic=fit.jac(x)
    numerical=np.empty_like(analytic);h=1e-6
    for i in range(len(x)):
        plus=x.copy();minus=x.copy();plus[i]+=h;minus[i]-=h
        numerical[:,i]=(fit.fun(plus)-fit.fun(minus))/(2*h)
    np.testing.assert_allclose(analytic,numerical,rtol=2e-6,atol=2e-8)


def test_virtual_posture_never_changes_played_endpoint_or_departure():
    fit,req=fixture();virtual=copy.deepcopy(req);virtual['goal'][5]-=.5;virtual['goal'][6]+=.3
    other=recovery.CurveFit(Kinematics(),virtual,np.zeros((2,3)))
    guided=other.nodes(other.initial());nominal=fit.nodes(fit.initial())
    actual=recovery.restore_goal(guided,req['goal'])
    assert not np.array_equal(guided[-1],actual[-1])
    np.testing.assert_array_equal(actual[-3:],np.tile(req['goal'],(3,1)))
    for alpha in (0.,.25,.5,.75,1.):
        blended=recovery.blend_tail(other,actual,nominal,alpha)
        np.testing.assert_array_equal(blended[:8],actual[:8])
        pp=other.polynomial(blended)
        assert recovery.position_bounds(pp,other.oracle.lower,other.oracle.upper)
        np.testing.assert_allclose(pp([0.,1.]),[req['start'],req['goal']],atol=1e-14)
        assert as_source(pp,dict(request=req))['request'] is req
    assert req['goal']!=virtual['goal']


def test_continuous_check_finds_overshoot_interior_not_only_endpoints():
    # 8*s*(1-s), endpoints zero but middle=2 > 1.5.
    pp=PPoly(np.tile(np.array([-8.,8.,0.])[:,None,None],(1,1,7)),[0.,1.])
    assert not recovery.position_bounds(pp,[-1.5]*7,[1.5]*7)


def report(**kw):
    return dict(candidates=[],searches=[],pool_elapsed_s=.1,profile=dict(fit_s=.1),**kw)


def test_legacy_success_does_not_run_recovery_or_change_candidate(monkeypatch):
    winner=object()
    monkeypatch.setattr(fast.fast_path,'candidates',lambda *a,**kw:([winner],report()))
    monkeypatch.setattr(recovery,'stages',lambda *a:pytest.fail('Successful primary path changed'))
    choices=fast.proposal_stages(None,{}, {},10.,lambda _:None,{})
    assert next(choices) is winner
    choices.close()


@pytest.mark.parametrize('reason,seconds',[('nominal_endpoint_invalid',10.),(None,.05)])
def test_invalid_endpoint_or_exhausted_budget_never_recovers(monkeypatch,reason,seconds):
    monkeypatch.setattr(fast.fast_path,'candidates',lambda *a,**kw:([],report(reason=reason)))
    monkeypatch.setattr(recovery,'stages',lambda *a:pytest.fail('Must stop'))
    assert list(fast.proposal_stages(None,{}, {},seconds,lambda _:None,{}))==[]


def test_recovery_runs_only_after_both_legacy_families_fail(monkeypatch):
    called=[];winner=object()
    def primary(*a,**kw):called.append(kw['prior_weight']);return [],report()
    def fallback(*a):
        assert called==[2.,5.]
        yield [winner],report()
        pytest.fail('Accepted recovery must not keep fitting')
    monkeypatch.setattr(fast.fast_path,'candidates',primary)
    monkeypatch.setattr(recovery,'stages',fallback)
    result={};choices=fast.proposal_stages(None,{}, {},10.,lambda _:None,result)
    assert next(choices) is winner
    assert result['recovery_policy']==recovery.VERSION
    assert result['pool_elapsed_s']==pytest.approx(.3)
    choices.close()


def test_recovery_rechecks_actual_endpoints_before_virtual_goals():
    class Invalid:
        def check(self,q):return dict(valid=[True,False])
        def call(self,*a,**kw):pytest.fail('Fitting invalid endpoint')
    req=dict(start=[0.]*7,goal=[9.]*7)
    assert list(recovery.stages(Invalid(),req,{},5.,lambda _:None))==[]


def test_reverse_construction_preserves_exact_curve_and_real_endpoints():
    fit,req=fixture();reverse_req=copy.deepcopy(req)
    reverse_req['start'],reverse_req['goal']=reverse_req['goal'],reverse_req['start']
    reverse_fit=recovery.CurveFit(Kinematics(),reverse_req,np.zeros((2,3)))
    controls=reverse_fit.nodes(reverse_fit.initial())
    original=reverse_fit.polynomial(controls);played=reverse_fit.polynomial(controls[::-1])
    grid=np.linspace(0,1,1001)
    np.testing.assert_allclose(played(grid),original(1-grid),atol=1e-14)
    as_source(played,dict(request=req))
    assert recovery.position_bounds(played,fit.oracle.lower,fit.oracle.upper)


@pytest.mark.parametrize('target',['R1','R2'])
def test_real_camera_regression_laws_geometry_extrema_and_no_internal_stops(target):
    import json
    from drum_planner.common import ROOT
    from drum_planner.timing import motion_quality
    from drum_planner.timing.benchmark import check_geometry
    p=ROOT/f'diagnostics/workspace_routes_20261026/evidence/incident_right_default_{target}.json'
    data=json.loads(p.read_text());source=data['source'];law=data['law']
    assert source['request']['scene']['synthetic'] is False
    assert source['request']['execute'] is False
    assert data['candidate']['source'] in ('bounded_workspace_curve','wrist_guided_workspace_curve')
    assert check_geometry(law,source)['max_error_rad']==0.
    path=core.decode(law['path']);h=core.decode(law['density'])
    assert recovery.position_bounds(path,data['limits_lower'],data['limits_upper'])
    peaks,_=core.density_peaks(path,h)
    assert (peaks<=np.array(law['limits'])).all()
    np.testing.assert_allclose(path([0.,1.]),[source['request']['start'],source['request']['goal']],atol=1e-12)
    quality=motion_quality.law_metrics(law)
    assert quality['internal_stops']==quality['deep_slowdowns']==0
