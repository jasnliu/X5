"""Cartesian prototype math, isolation and failure contracts; no devices."""
from test_checkpoint_performance import OPTIMIZED
import inspect
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize._numdiff import approx_derivative
from drum_planner.experiment import workspace_curve as wc,config
from drum_planner.common import ROOT,digest

class Oracle:
    lower=[-3.]*7;upper=[3.]*7
    def call(self,op,**kwargs):
        assert op=='fk'
        q=np.asarray(kwargs['q'])
        # Smooth nonlinear mock FK exercises the derivative chain, not just a
        # linear Cartesian toy where missing factors might go unnoticed.
        p=np.column_stack((np.sin(q[:,0])+.2*q[:,3],q[:,1]+.1*q[:,4]**2,q[:,2]+.2*np.sin(q[:,5])))
        tip=p+np.column_stack((.3*np.cos(q[:,6]),.3*np.sin(q[:,6]),.1*np.cos(q[:,4])))
        return dict(wrist_xyz_m=p.tolist(),tip_xyz_m=tip.tolist())

def problem():
    return dict(start=[-.1,.1,.2,.1,-.2,.3,-.4],goal=[.4,.25,.4,.7,.2,-.1,.8])

def test_bezier_endpoint_exact_and_direction_frame_degenerate_cases():
    assert np.array_equal(wc.bezier_basis([0.,1.]),np.zeros((2,2)))
    for b in ([0,0,0],[0,0,1],[1,0,0],[0,1,0]):
        offsets=wc.arc_offsets(np.zeros(3),np.array(b))
        assert len(offsets)==7 and all(x.shape==(2,3) and np.isfinite(x).all() for x in offsets)
        assert np.array_equal(offsets[0],np.zeros((2,3)))
        assert all(np.allclose(x[0],x[1]) for x in offsets)

def test_batched_chain_jacobian_matches_independent_numeric_difference():
    p=wc.CurveFit(Oracle(),problem(),np.full((2,3),.02))
    x=p.initial();x[0]+=.01
    actual=p.jac(x)
    expected=approx_derivative(p.fun,x,method='3-point')
    assert np.allclose(actual,expected,atol=2e-7,rtol=2e-6)

def test_fit_keeps_full_joint_endpoints_and_no_hardware_or_recording_reader():
    q,d=wc.CurveFit(Oracle(),problem(),np.zeros((2,3))).solve()
    assert np.array_equal(q[0],problem()['start']) and np.array_equal(q[-1],problem()['goal'])
    assert d['fit_max_m']<.025
    assert np.isfinite(q).all()
    text=inspect.getsource(wc)
    assert 'recordings/' not in text and 'goal_recording' not in text
    assert 'socket' not in text and 'telemetry' not in text

def test_blocked_endpoint_never_falls_back_to_other_method(tmp_path):
    class Blocked:
        def check(self,*args,**kwargs):return dict(valid=[True,False],details={'reason':'collision'})
    result=wc.build(Blocked(),problem(),tmp_path,config.settings(),60,lambda _:None)
    assert result['reason']=='nominal_endpoint_invalid' and not result['candidates']

def test_deadline_before_fit_has_clear_diagnostic():
    q,d=wc.CurveFit(Oracle(),problem(),np.zeros((2,3)),deadline=0).solve()
    assert q is None and d['reason']=='fit_budget_exhausted'

def test_geometric_score_does_not_include_time_and_clearance_is_soft():
    q=np.linspace(problem()['start'],problem()['goal'],81)
    f=Oracle().call('fk',q=q)
    m=wc.path_metrics(q,f)
    assert wc.score(m,.5,{'clearance_weight':3})-wc.score(m,0,{'clearance_weight':3})==pytest.approx(1.5)
    m['duration_s']=10000
    assert wc.score(m,0,{})==wc.score({k:v for k,v in m.items() if k!='duration_s'},0,{})

def test_original_generators_timing_collision_inputs_and_actuator_sources_unchanged():
    before=json.loads((ROOT/'diagnostics/workspace_curve_20261021/before_hashes.json').read_text())
    authorized={'drum_planner/experiment/config.py','drum_planner/experiment/worker.py',
                'drum_planner/experiment/app.py','drum_planner/experiment/algorithms.py',
                'drum_planner/cpp/experiment_oracle.cpp'}
    # Authorized hardware Workspace curve/checkpoint rollout; new tests cover
    # its exact mission playback and unchanged experiment/timing/input sources.
    authorized.update({'drum_planner/baseline/curve.py', 'drum_planner/baseline/transport.py', 'drum_planner/baseline/__init__.py', 'drum_planner/cli.py', 'drum_planner/baseline/computation.py', 'drum_planner/baseline/motion.py', 'drum_planner/baseline/windows.py', 'drum_planner/baseline/work.py', 'drum_planner/baseline/workspace.py', 'drum_planner/baseline/service.py', 'drum_planner/baseline/planner.py', 'drum_planner/baseline/tasks.py'})
    # Approved 5 mm map wiring; test_depth_resolution protects all generation/timing/data.
    authorized.update({'drum_planner/depth.py','drum_planner/planning.py','drum_planner/cpp/moveit_plan.cpp','drum_planner/experiment/worker.py','drum_planner/experiment/algorithms.py','drum_planner/baseline/work.py','drum_planner/live_view.py'})
    for path,sha in before.items():
        if path not in authorized|OPTIMIZED:assert digest(ROOT/path)==sha,path

def test_failure_is_visible_cached_and_never_a_legacy_fallback(tmp_path,monkeypatch):
    from drum_planner.experiment import algorithms,runtime
    calls=[]
    class Fake:
        meta={'joint_limit_check':{'valid':True}}
        def __init__(self,*args):calls.append('oracle')
        def close(self):calls.append('closed')
    monkeypatch.setattr(runtime,'Oracle',Fake)
    monkeypatch.setattr(algorithms,'pool_key',lambda *a:'same_inputs')
    monkeypatch.setattr(wc,'build',lambda *a:dict(candidates=[],attempts=[],elapsed_s=1.,
        options=wc.OPTIONS,reason='no_valid_workspace_curve_within_budget'))
    args=({}, {'saved_not_live':False},tmp_path,tmp_path/'request.json',config.settings(),10,lambda _:None)
    winner,report=wc.run(*args)
    assert winner is None and not report['pool_reused'] and report['selected'] is None
    assert report['reason']=='no_valid_workspace_curve_within_budget'
    assert report['normalization']['fallback_used'] is False
    again,second=wc.run(*args)
    assert again is None and second['pool_reused'] and calls==['oracle','closed']

def test_smooth_cartesian_fit_never_bypasses_collision_rejection(tmp_path,monkeypatch):
    from drum_planner.experiment import algorithms
    class Fake(Oracle):
        def check(self,*a,**k):return {'valid':[True,True]}
    checks=[]
    def reject(*args,**kwargs):checks.append(args[1]);return None
    monkeypatch.setattr(algorithms,'timed_candidate',reject)
    result=wc.build(Fake(),problem(),tmp_path,config.settings(),60,lambda _:None,
                    {'proposal_count':1,'prior_weights':[5.]})
    assert len(checks)==1 and result['candidates']==[]
    assert result['attempts'][0]['reason']=='continuous_curve_bounds_collision_or_zone'


def test_existing_four_routing_and_labels_are_byte_identical_after_removing_fifth_entry():
    import hashlib
    from drum_planner.experiment import worker,app
    before=json.loads((ROOT/'diagnostics/workspace_curve_20261021/before_hashes.json').read_text())
    old_config=''.join(line for line in Path(config.__file__).read_text().splitlines(True)
                       if not line.lstrip().startswith("'workspace_curve':"))
    assert hashlib.sha256(old_config.encode()).hexdigest()==before['drum_planner/experiment/config.py']
    # Authorized staged timing orchestration; generators/ranking/policies are
    # retained, with exact mathematical parity checked against preserved code.
    from test_checkpoint_performance import test_policies_geometry_inputs_and_motor_control_are_unchanged
    test_policies_geometry_inputs_and_motor_control_are_unchanged()
