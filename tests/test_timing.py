"""Exact-path retiming, limits, serialization and software-only integration."""
import copy,json,socket
from pathlib import Path
import numpy as np
import pytest
from drum_planner import common,planning
from drum_planner.timing import core,adaptive,benchmark


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    real=socket.socket
    def socket_guard(family=socket.AF_INET,*a,**k):
        if family==socket.AF_CAN:pytest.fail('No CAN in timing tests')
        return real(family,*a,**k)
    monkeypatch.setattr(socket,'socket',socket_guard)
    from drum_planner import depth
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**k:pytest.fail('No camera in timing tests'))


def fixture(name):return json.loads((common.ROOT/'diagnostics/timing_experiment_20261021/inputs'/f'{name}.json').read_text())


@pytest.mark.parametrize('case',['right_single_arc','right_clearance','left_single_arc','left_clearance'])
@pytest.mark.parametrize('method',['global','local_quintic','scurve'])
def test_frozen_paths_limits_exact_geometry_and_json_roundtrip(case,method):
    source=fixture(case);before=json.dumps(source,sort_keys=True)
    law=core.global_law(source) if method=='global' else core.local_law(source,method)
    report=benchmark.validate(law,source,None,collision=False)
    assert report['geometry']['max_error_rad']<core.PATH_TOL
    assert np.array_equal(law['limits'],[.6,1.2,6])
    assert np.all(np.asarray(law['peaks'])<=core.LIMITS[:,None])
    assert json.dumps(source,sort_keys=True)==before
    restored=json.loads(json.dumps(law,allow_nan=False));t=np.linspace(0,law['duration_s'],311)
    for a,b in zip(core.evaluate(law,t),core.evaluate(restored,t)):assert np.array_equal(a,b)


@pytest.mark.parametrize('distance',[1e-8,1e-5,.001,.01,.095,.096,.1,.2,.42,.5,1.,10.])
def test_scurve_all_phase_regimes(distance):
    c,x=core.scurve(distance);p=core.PPoly(c,x);grid=np.linspace(0,x[-1],10001)
    assert p(x[-1])==pytest.approx(distance,abs=1e-12)
    assert abs(p(x[-1],nu=1))<1e-12 and abs(p(x[-1],nu=2))<1e-12
    assert np.all(np.diff(p(grid))>=-1e-12)
    for k,limit in enumerate(core.LIMITS,1):assert np.max(abs(p(grid,nu=k)))<=limit+1e-10
    for knot in x[1:-1]:
        for k in range(3):assert p(knot,nu=k)==pytest.approx(p(np.nextafter(knot,-np.inf),nu=k),abs=1e-11)


def test_collinear_passage_preserves_every_point_and_does_not_round_tiny_corners():
    q=np.zeros((5,7));q[:,0]=[0,.1,.2,.3,.4];q[3:,1]=[1e-7,1e-7]
    _,_,t=planning.smooth_samples(q,True);source=dict(path_waypoints=q.tolist(),timing=t)
    law=core.local_law(source,'scurve');groups=law['details']['blocks']
    assert [(r['first'],r['last']) for r in groups]==[(0,2),(2,3),(3,4)]
    events=law['details']['waypoint_times_s'];positions,v,a,_=core.evaluate(law,events)
    assert np.allclose(positions,q,atol=1e-12) and abs(v[1,0])>.01
    assert np.max(abs(v[[2,3]]))<1e-10 and np.max(abs(a[[2,3]]))<1e-10
    assert len(events)==len(q)


def test_adaptive_curve_is_exact_original_spline_and_continuously_limited():
    source=fixture('right_single_arc');law=adaptive.optimize(source,levels=(8,12))
    assert law['duration_s']<source['duration_outward_s']*.7
    assert law['kind']=='time_density'
    p=core.original(source);saved=core.decode(law['path'])
    assert np.array_equal(p.c,saved.c) and np.array_equal(p.x,saved.x)
    report=benchmark.validate(law,source,None,collision=False)
    assert report['geometry']['max_error_rad']==0
    h=core.decode(law['density']);peaks,_=core.density_peaks(p,h)
    assert np.all(peaks<=core.LIMITS[:,None]*(1+1e-9))


def test_bad_corner_stop_flag_cannot_straighten_a_curved_path():
    source=fixture('right_clearance');source['timing']['stop_at_corners']=True
    with pytest.raises(ValueError,match='Corner-stop flag'):
        core.local_law(source,'scurve')


def test_dense_derivatives_match_finite_differences_away_from_switches():
    source=fixture('right_single_arc');law=adaptive.optimize(source,levels=(8,))
    t=np.linspace(.1,law['duration_s']-.1,100);eps=1e-5
    center=core.evaluate(law,t);lo=core.evaluate(law,t-eps);hi=core.evaluate(law,t+eps)
    for k in range(3):assert np.allclose((hi[k]-lo[k])/(2*eps),center[k+1],atol=1e-5,rtol=1e-5)


def test_straight_analytic_branch_and_invalid_inputs():
    source=fixture('left_single_arc');law=adaptive.optimize(source)
    assert law['method']=='adaptive' and 'analytic' in law['details']['branch']
    for value in (0,-1,float('nan')):
        with pytest.raises(ValueError):core.scurve(value)
    with pytest.raises(ValueError,match='Unknown timing law'):
        core.Evaluator(dict(kind='unknown',duration_s=1))


@pytest.mark.parametrize('side',['right','left'])
def test_analytic_preview_exact_hold_return_and_static_other_arm(tmp_path,side):
    from drum_planner.trajectory_view import PlanDisplay
    source=fixture(side+'_single_arc');req=benchmark.benchmark_request(source)
    law=adaptive.optimize(source,levels=(8,));points=core.samples(law,source)
    obj=dict(schema='x5-moveit-result-v1',status='planned',motion_authorized=False,
        hardware_execution_available=False,moving_side=side,model=source['model'],
        outward=points,cycle=planning.cycle(points),timing=law,
        duration_outward_s=law['duration_s'],hold_at_goal_s=2.)
    common.atomic_json(tmp_path/'plan.json',obj);common.atomic_json(tmp_path/'backend_request.json',req)
    preview=PlanDisplay(tmp_path/'plan.json');T=law['duration_s']
    assert preview.retimed is not None
    assert preview.duration==pytest.approx(2*T+2)
    initial=preview.frames(0);other='left' if side=='right' else 'right'
    names=[n for n in initial if n.startswith('openarmx_'+other+'_') or n==other+'_drumstick']
    assert names
    for t in np.linspace(.01,T-.01,29):
        assert np.allclose(preview.positions(t),core.evaluate(law,t)[0],atol=1e-12)
        assert np.allclose(preview.positions(2*T+2-t),preview.positions(t),atol=1e-12)
        for n in names:assert np.array_equal(preview.frames(t)[n],initial[n])
    for t in (T,T+1,T+2):
        assert np.array_equal(preview.positions(t),req['goal'])
        assert preview.phase(t).startswith('HOLD')
    for t in (0,preview.duration):assert np.array_equal(preview.positions(t),req['start'])
    obj['duration_outward_s']+=1;common.atomic_json(tmp_path/'plan.json',obj)
    with pytest.raises(ValueError,match='duration mismatch'):PlanDisplay(tmp_path/'plan.json')


def test_separate_timing_cache_reuse_invalidation_and_no_input_mutation(tmp_path,monkeypatch):
    from drum_planner.timing import integration
    source=fixture('left_clearance');req=source['request']
    winner=dict(path=source['path_waypoints'],timing=source['timing'],duration_s=source['duration_outward_s'])
    before=copy.deepcopy(winner);calls=[];real=integration.validate
    def validate(law,s,folder):
        calls.append(law['source_path_sha256']);proof=real(law,s,None,collision=False)
        proof['collision_zone']={'valid':True,'samples_checked':proof['dense_samples']}
        return proof
    monkeypatch.setattr(integration,'validate',validate)
    args=(winner,dict(pool_key='fixture'),req,dict(pool_root=str(tmp_path/'pool')),tmp_path/'output',lambda s:None)
    first=integration.retime(*args);second=integration.retime(*args)
    assert len(calls)==1 and not first[3]['cached'] and second[3]['cached']
    assert first[:3]==second[:3] and winner==before
    assert np.array_equal(second[0][0]['positions_rad'],req['start'])
    assert np.array_equal(second[0][-1]['positions_rad'],req['goal'])
    file=next((tmp_path/'pool/timing').glob('*/law.json'));file.write_text('{}')
    assert not integration.retime(*args)[3]['cached'] and len(calls)==2
    monkeypatch.setattr(integration,'fingerprint',lambda:'changed-timing-code')
    assert not integration.retime(*args)[3]['cached'] and len(calls)==3


def test_retained_benchmark_models_and_selected_policy():
    from drum_planner.timing.integration import policy
    for side in ('left','right'):
        source=fixture(side+'_clearance');req=benchmark.benchmark_request(source)
        assert common.digest(req['urdf'])==source['model']['calibrated_urdf_sha256']
        assert common.digest(req['srdf'])==source['model']['srdf_sha256']
    cfg=policy();assert cfg['method']=='adaptive' and cfg['levels'][-1]==192
    assert np.array_equal(cfg['limits'],core.LIMITS)
