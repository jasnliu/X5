"""Device-free tests for endpoint-only gesture proposals and metrics."""
import inspect
import numpy as np
import pytest
from drum_planner.experiment import gesture


def features(x):
    return dict(wrist_xyz_m=x,tip_xyz_m=np.asarray(x)*1.5)


def test_zero_parameters_are_exact_direct_paths():
    a=np.linspace(-.2,.3,7);b=np.linspace(.4,-.1,7)
    for family,n in [('phase',7),('arch',14)]:
        q=gesture.curves(a,b,np.zeros((3,n)),family)
        assert q.shape==(3,49,7)
        assert np.array_equal(q[:,0],np.tile(a,(3,1)))
        assert np.array_equal(q[:,-1],np.tile(b,(3,1)))
        assert np.allclose(q[0],np.linspace(a,b,49))


def test_phase_proposals_have_no_motor_reversals_before_timing():
    rng=np.random.default_rng(4);a=rng.normal(size=7);b=rng.normal(size=7)
    q=gesture.curves(a,b,rng.uniform(-1.3,1.3,(40,7)),'phase')
    assert np.all(np.diff(q,axis=1)*np.sign(b-a)>=-1e-12)
    assert np.all(q>=np.minimum(a,b)-1e-12) and np.all(q<=np.maximum(a,b)+1e-12)


def test_backtracking_magnitude_and_endpoint_overshoot():
    q=np.zeros((101,7));q[:,0]=np.linspace(0,1,101)
    x=np.column_stack([np.linspace(0,1,101),np.zeros((101,2))])
    g=gesture.geometry(q,features(x))
    assert g['joint_excess_deg']==pytest.approx(0)
    assert g['workspace_detour']==pytest.approx(0)
    assert g['workspace_overshoot']==pytest.approx(0)
    bad=q.copy();bad[:,1]=.3*np.sin(np.linspace(0,np.pi,101))
    b=gesture.geometry(bad,features(x))
    assert b['joint_excess_deg']==pytest.approx(np.degrees(.6))
    assert gesture.objective(b)>gesture.objective(g)
    bend=x.copy();bend[:,1]=.3*np.sin(np.linspace(0,np.pi,101))
    c=gesture.geometry(q,features(bend))
    assert c['workspace_overshoot']>.2 and c['workspace_detour']>0
    assert gesture.objective(c)>gesture.objective(g)


def test_geometry_is_time_free_and_duplicate_sample_stable():
    s=np.linspace(0,1,201);q=np.column_stack([s]*7)
    x=np.column_stack([s,.2*np.sin(np.pi*s),s*0])
    a=gesture.geometry(q,features(x))
    b=gesture.geometry(np.repeat(q,3,axis=0),features(np.repeat(x,3,axis=0)))
    for key in ('joint_excess_deg','workspace_detour','workspace_overshoot','broad_turning_rad','progress_disagreement'):
        assert a[key]==pytest.approx(b[key],abs=1e-8)


def test_no_recording_or_hardware_dependency_in_generator():
    text=inspect.getsource(gesture)
    assert 'read_text' not in text and 'DEFAULT_RECORDING' not in text
    assert 'motor_transport' not in text and 'socket' not in text
    from drum_planner.experiment import search,config
    assert config.DEFAULT==search.DEFAULTS
    assert 'DEFAULT_RECORDING' not in inspect.getsource(search)


def test_resampling_for_refinement_preserves_all_original_corners():
    path=np.array([[0.]*7,[.04,0,0,0,0,0,0],[.04,.12,0,0,0,0,0],[.3,.15,0,0,0,0,0]])
    q=gesture.with_vertices(path,9)
    assert all(any(np.allclose(point,row,atol=1e-12) for row in q) for point in path)


def test_blocked_endpoint_is_an_expected_search_result(tmp_path):
    from drum_planner.experiment import search
    class Blocked:
        def check(self,q,**kwargs):return dict(valid=[True,False],details={'reason':'collision'})
    result=search.build(Blocked(),dict(start=[0]*7,goal=[1]*7),tmp_path,search.DEFAULTS,10,lambda _:None)
    assert result['reason']=='nominal_endpoint_invalid' and result['candidates']==[]


def test_numerical_copy_does_not_replace_earlier_route_with_artificial_improvement():
    from drum_planner.experiment import search
    import copy
    q=np.linspace(np.zeros(7),np.ones(7),81)
    g=gesture.geometry(q,features(q[:,:3]))
    g['joint_excess_ratio']=.1
    first=dict(index=8,source='corridor',duration_s=51.,gesture=g,
               metrics={'baseline':{'components':{'obstacle_proximity':.1}}})
    duplicate=copy.deepcopy(first);duplicate.update(index=24,source='sweep',duration_s=114.)
    duplicate['gesture']['joint_excess_ratio']-=1e-9
    rows,meta=search.rank([duplicate,first],'sweep')
    assert rows[0]['index']==8 and meta['score_round_digits']==8
    first['duration_s']=200.
    rows,_=search.rank([duplicate,first],'sweep')
    assert rows[0]['index']==8  # Still time-independent, not secretly fastest.


def test_objectives_are_explicit_and_clearance_only_affects_comfort():
    q=np.linspace(np.zeros(7),np.ones(7),81)
    g=gesture.geometry(q,features(q[:,:3]))
    assert gesture.objective(g,'economy',.8)==gesture.objective(g,'economy',0)
    assert gesture.objective(g,'comfort',.8)-gesture.objective(g,'comfort',0)==pytest.approx(2.4)
    with pytest.raises(ValueError):gesture.objective(g,'unknown')
