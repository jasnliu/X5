"""Versioned, bounded left-only smoothing. All device access is forbidden."""
from test_checkpoint_performance import OPTIMIZED
import copy,json,socket
from pathlib import Path
import numpy as np
import pytest
from drum_planner import common,planning
from drum_planner.timing import core,rounding,motion_quality,smooth_v2,dispatch,integration


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    original=socket.socket
    def guard(family=socket.AF_INET,*a,**kw):
        if family==socket.AF_CAN:pytest.fail('No CAN in smoothing tests')
        return original(family,*a,**kw)
    monkeypatch.setattr(socket,'socket',guard)
    from drum_planner import depth
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**k:pytest.fail('No camera'))


def source(q):
    q=np.asarray(q,float);_,_,t=planning.smooth_samples(q,True)
    return dict(path_waypoints=q.tolist(),timing=t,request=dict(start=q[0].tolist(),goal=q[-1].tolist(),moving_side='left'))


def bent():
    q=np.zeros((4,7));q[:,0]=[0,.2,.4,.6];q[:,1]=[0,0,.06,.09];return source(q)


class EmptyOracle:
    lower=[-5.]*7;upper=[5.]*7
    def check(self,q,**kw):return dict(valid=[True]*len(q))
    def call(self,op,q):
        q=np.asarray(q);assert op=='fk'
        return dict(wrist_xyz_m=(q[:,:3]*.1).tolist(),tip_xyz_m=(q[:,:3]*.2).tolist())


@pytest.mark.parametrize('kind',['local_c2','local_c3','overlap_c3','quintic'])
def test_smoothing_preserves_endpoints_and_c2_without_stops(kind):
    s=bent();original=copy.deepcopy(s)
    p,_=rounding.propose(s,kind,.04 if kind=='overlap_c3' else .49)
    new=rounding.as_source(p,s)
    assert s==original
    assert np.allclose(p([0,1]),[s['request']['start'],s['request']['goal']],atol=1e-13)
    assert np.max(abs(p([0,1],nu=1)))<1e-10 and np.max(abs(p([0,1],nu=2)))<1e-9
    _,jumps=core.polynomial_peaks(p);assert np.all(jumps<[1e-9,1e-8,1e-7])
    assert np.min(np.linalg.norm(p(np.linspace(.01,.99,3001),nu=1),axis=1))>0
    assert not new['timing']['stop_at_corners']
    if kind.endswith('c3'):
        for x in np.unique(p.x)[1:-1]:assert np.max(abs(p(x,nu=3)-p(np.nextafter(x,-np.inf),nu=3)))<1e-6


def test_tube_bound_encloses_dense_curve_and_collision_is_mandatory():
    s=bent();p,_=rounding.propose(s,'overlap_c3',.04)
    ok,proof=rounding.inspect(p,s,EmptyOracle());assert ok
    _,distance,_=rounding.closest(p(np.linspace(0,1,20001)),np.asarray(s['path_waypoints']))
    assert distance.max()<=proof['certified_joint_tube_rad']<=np.radians(1)
    assert not rounding.inspect(p,s,EmptyOracle(),joint_tube_rad=1e-7)[0]
    class Collision(EmptyOracle):
        def check(self,q,**kw):return dict(valid=[False],details={'reason':'fixture collision'})
    ok,proof=rounding.inspect(p,s,Collision());assert not ok and proof['reason']=='collision_or_zone'


@pytest.mark.parametrize('width',[0,-.01,.25,1.])
def test_invalid_kernel_width_rejected(width):
    with pytest.raises(ValueError):rounding.propose(bent(),'overlap_c3',width)


def test_frozen_left_geometry_and_v1_stop_comparison():
    for case in ('single_arc','clearance','progress','sweep'):
        s=json.loads((common.ROOT/f'diagnostics/corner_smoothing_20261021/inputs/left_{case}.json').read_text())
        p,_=rounding.propose(s,'overlap_c3',.03 if case=='sweep' else .04)
        rounding.as_source(p,s)
        assert np.min(np.linalg.norm(p(np.linspace(.01,.99,5001),nu=1),axis=1))>0
        assert motion_quality.law_metrics(s['v1_law'])['internal_stops']>0


@pytest.mark.parametrize('side',['left','right'])
def test_dispatch_preserves_v1_right_arguments_exactly(monkeypatch,side):
    calls=[];result=object()
    def old(*args):calls.append(('v1',args));return result
    def new(*args):calls.append(('v2',args));return result
    monkeypatch.setattr(integration,'retime',old);monkeypatch.setattr(smooth_v2,'retime',new)
    args=({}, {}, {'moving_side':side}, {},Path('/unused'),print)
    assert dispatch.retime(*args) is result
    assert calls==[('v1' if side=='right' else 'v2',args)]


def test_no_new_stops_can_be_hidden_by_uniform_slowdown():
    law=core.local_law(bent(),'scurve');q=motion_quality.law_metrics(law)
    t=np.linspace(0,law['duration_s'],5001);p,v,_,_=core.evaluate(law,t)
    slower=motion_quality.measure(t*10,p,v/10)
    assert slower['internal_stops']==q['internal_stops']>0
    assert slower['interior_speed_cv']==pytest.approx(q['interior_speed_cv'],rel=.001)
    assert not smooth_v2.acceptable(slower,q,smooth_v2.policy())


def test_v1_source_and_policy_files_are_unchanged():
    base=common.ROOT/'diagnostics/corner_smoothing_20261021/preserved_sources.json'
    # The later Single arc correction intentionally changes shared selection
    # and its logging, not the preserved timing engines or proposal geometry.
    # test_single_arc_priority separately checks the unchanged generator bodies.
    updated_selection={'drum_planner/experiment/search.py','drum_planner/experiment/algorithms.py',
                       'drum_planner/planning.py'}  # later occupied-volume request accounting, not timing
    # Authorized performance work preserves the old engine snapshot and checks
    # exact numerical parity in test_planner_performance; policy stays unchanged.
    performance_update={'drum_planner/timing/core.py','drum_planner/timing/adaptive.py',
                        'drum_planner/timing/integration.py','drum_planner/experiment/runtime.py'}
    for name,sha in json.loads(base.read_text()).items():
        if name not in updated_selection|performance_update|OPTIMIZED:assert common.digest(common.ROOT/name)==sha,name


def test_right_cannot_enter_v2():
    with pytest.raises(ValueError,match='left-only'):
        smooth_v2.retime({}, {},{'moving_side':'right'}, {},'/unused',print)


def test_continuous_left_path_retains_v1(monkeypatch):
    monkeypatch.setattr(integration,'retime',lambda *a:123)
    assert smooth_v2.retime({'timing':{'stop_at_corners':False}}, {},{'moving_side':'left'}, {},'/unused',lambda s:None)==123


def test_v2_cache_reuse_corruption_and_fingerprint_invalidation(tmp_path,monkeypatch):
    s=bent();winner=dict(path=s['path_waypoints'],timing=s['timing'],duration_s=s['timing']['duration_s'])
    original=copy.deepcopy(winner);calls=[]
    def choose(source,folder,log,cfg):
        calls.append(1);p,_=rounding.propose(source,'overlap_c3',.04)
        rounded=rounding.as_source(p,source);law=core.global_law(rounded)
        law.update(method='adaptive_c3_v2',original_path_sha256=core.identity(source))
        before=motion_quality.law_metrics(core.local_law(source,'scurve'));after=motion_quality.law_metrics(law)
        return rounded,law,{'collision_zone':{'valid':True}},dict(before=before,after=after,
            rounding={'tip_deviation_m':.001},params={},attempts=[])
    monkeypatch.setattr(smooth_v2,'choose',choose)
    args=(winner,dict(pool_key='fixture'),s['request'],dict(pool_root=str(tmp_path/'pool')),tmp_path/'output',lambda s:None)
    a=smooth_v2.retime(*args);b=smooth_v2.retime(*args)
    assert not a[3]['cached'] and b[3]['cached'] and len(calls)==1
    assert a[:3]==b[:3] and winner==original
    assert a[0][0]['positions_rad']==s['request']['start'] and a[0][-1]['positions_rad']==s['request']['goal']
    assert np.all(np.diff([p['time_s'] for p in a[0]])>0)
    assert np.all(np.diff([p['time_s'] for p in planning.cycle(a[0])])>0)
    path=next((tmp_path/'pool/timing_v2').glob('*/law.json'));path.write_text('{}')
    assert not smooth_v2.retime(*args)[3]['cached'] and len(calls)==2
    monkeypatch.setattr(smooth_v2,'fingerprint',lambda:'changed-v2')
    assert not smooth_v2.retime(*args)[3]['cached'] and len(calls)==3


def test_failed_rounding_explicitly_falls_back_to_unchanged_v1(tmp_path,monkeypatch):
    s=bent();winner=dict(path=s['path_waypoints'],timing=s['timing'],duration_s=s['timing']['duration_s'])
    calls=[]
    monkeypatch.setattr(smooth_v2,'choose',lambda *a:(None,None,None,{'reason':'fixture collision'}))
    def old(*args):calls.append(args);return [1],{'method':'adaptive'},{'valid':True},{'version':1}
    monkeypatch.setattr(integration,'retime',old)
    args=(winner,dict(pool_key='fixture'),s['request'],dict(pool_root=str(tmp_path/'pool')),tmp_path/'output',lambda s:None)
    points,law,proof,meta=smooth_v2.retime(*args)
    assert calls==[args] and points==[1] and law['method']=='adaptive' and proof['valid']
    assert not meta['smoothing_applied'] and meta['label'].startswith('V1 fallback')


def test_only_known_solver_trial_warning_is_reclassified(monkeypatch):
    import warnings
    def solver(*a,**k):
        warnings.warn_explicit('Values in x were outside bounds during a minimize step, clipping to bounds',
            RuntimeWarning,'/usr/lib/python3/dist-packages/scipy/optimize/_optimize.py',404)
        warnings.warn('unrelated numerical issue',RuntimeWarning)
        return {'fixture':True}
    monkeypatch.setattr(smooth_v2.adaptive,'optimize',solver);logs=[]
    with pytest.warns(RuntimeWarning,match='unrelated numerical issue') as caught:
        law,count=smooth_v2.optimize_rounded({},smooth_v2.policy(),logs.append)
    assert len(caught)==1 and count==1 and law['fixture'] and 'no robot joints clipped' in logs[0]


def test_real_solver_failure_is_not_suppressed(monkeypatch):
    def solver(*a,**k):raise ValueError('Real failure')
    monkeypatch.setattr(smooth_v2.adaptive,'optimize',solver)
    with pytest.raises(ValueError,match='Real failure'):
        smooth_v2.optimize_rounded({},smooth_v2.policy(),lambda s:None)


@pytest.mark.parametrize('method',['single_arc','clearance','progress','sweep'])
def test_retained_v2_laws_limits_fluidity_and_analytic_preview(tmp_path,method):
    from drum_planner.timing import benchmark
    from drum_planner.trajectory_view import PlanDisplay
    folder=common.ROOT/'diagnostics/corner_smoothing_20261021/selected'/('left_'+method)
    s=json.loads((folder/'source.json').read_text());law=json.loads((folder/'law.json').read_text())
    original=json.loads((folder.parent.parent/'inputs'/('left_'+method+'.json')).read_text())
    assert law['original_path_sha256']==core.identity(original)
    assert not law['geometry_preserved'] and law['retiming_geometry_preserved']
    benchmark.validate(law,s,None,collision=False)
    assert smooth_v2.acceptable(motion_quality.law_metrics(law),motion_quality.law_metrics(original['v1_law']),smooth_v2.policy())
    req=benchmark.benchmark_request(s);points=core.samples(law,s);T=law['duration_s']
    obj=dict(schema='x5-moveit-result-v1',status='planned',motion_authorized=False,
        hardware_execution_available=False,moving_side='left',model=s['model'],
        outward=points,cycle=planning.cycle(points),timing=law,duration_outward_s=T,hold_at_goal_s=2.)
    common.atomic_json(tmp_path/'plan.json',obj);common.atomic_json(tmp_path/'backend_request.json',req)
    preview=PlanDisplay(tmp_path/'plan.json');initial=preview.frames(0)
    fixed=[n for n in initial if n.startswith('openarmx_right_') or n=='right_drumstick']
    assert fixed and preview.duration==pytest.approx(2*T+2)
    for t in np.linspace(.01,T-.01,37):
        assert np.allclose(preview.positions(t),core.evaluate(law,t)[0],atol=1e-12)
        assert np.allclose(preview.positions(2*T+2-t),preview.positions(t),atol=1e-12)
        for name in fixed:assert np.array_equal(preview.frames(t)[name],initial[name])
    for t in (T,T+1,T+2):assert np.array_equal(preview.positions(t),req['goal'])
    for t in (0,2*T+2):assert np.array_equal(preview.positions(t),req['start'])
