"""Fresh experiment pipeline and actual GPU math; no devices except compute GPU."""
import copy
import json
from pathlib import Path
import numpy as np
import pytest
from drum_planner.common import ROOT,digest
from drum_planner.experiment import fast,fast_config,worker
from drum_planner.timing import core,motion_quality
from drum_planner.timing.vulkan import fast as timing
from drum_planner.timing.benchmark import validate


def source(name='rest_to_R1'):
    return json.loads((ROOT/'diagnostics/gpu_timing_20261024/replay_inputs'/f'{name}.json').read_text())


def test_only_new_experiment_menu_and_bypass(monkeypatch,tmp_path):
    from drum_planner.experiment import app,config
    assert app.METHODS==fast_config.METHODS and list(app.METHODS)==['workspace_curve']
    assert app.DEFAULT_METHOD=='workspace_curve'
    assert len(config.METHODS)==5  # Retained historical implementations/policies.
    monkeypatch.setattr(fast,'plan',lambda *args:('new',args))
    assert worker.plan({},tmp_path,'workspace_curve',{},60)[0]=='new'


@pytest.mark.parametrize('side',['right','left'])
def test_world_reuse_is_exact_except_selected_start_goal(tmp_path,side):
    req=copy.deepcopy(source()['request']);req['moving_side']=side
    for key in ('urdf','srdf'):
        p=tmp_path/key;p.write_text(key);req[key]=str(p)
    if side=='left':req['left']=req['start']
    a=fast.world_identity(req)
    changed=copy.deepcopy(req);changed['scene_robot_start']=req['start']
    changed['start']=[q+.01 for q in req['start']];changed['goal']=[q-.01 for q in req['goal']]
    if side=='left':changed['left']=changed['start']
    assert fast.world_identity(changed)==a
    for key in ('collision_policy','world_padding_policy','scene','left_opening_m'):
        wrong=copy.deepcopy(changed);wrong[key]='different'
        assert fast.world_identity(wrong)!=a
    passive='right' if side=='left' else 'left'
    wrong=copy.deepcopy(changed);wrong[passive]=[0.]*7
    assert fast.world_identity(wrong)!=a
    Path(req['urdf']).write_text('new model')
    assert fast.world_identity(req)!=a


def test_problem_gradient_matches_finite_differences():
    from drum_planner.timing.vulkan.optimizer import arrays
    s=source();_,_,_,B,q,w=arrays(core.original(s),s['timing']['time_scale']/core.RESERVE,12)
    p=timing.Problem(B,q,w);x=np.linspace(.7,1.1,12)
    f,g,_=p.calc(x);step=1e-6
    numerical=[]
    for i in range(len(x)):
        dx=np.eye(len(x))[i]*step
        numerical.append((p.calc(x+dx)[0]-p.calc(x-dx)[0])/(2*step))
    np.testing.assert_allclose(g,numerical,rtol=2e-5,atol=2e-5)


def test_stronger_proposals_are_lazy_and_not_repeated(monkeypatch):
    called=[]
    def candidates(*a,**kw):
        weight=kw['prior_weight'];called.append(weight)
        return [weight],dict(candidates=[weight],searches=[weight],pool_elapsed_s=.1,profile={'fit_s':.1})
    monkeypatch.setattr(fast.fast_path,'candidates',candidates)
    report={};choices=fast.proposal_stages(None,{}, {},60,lambda _:None,report)
    assert next(choices)==2. and called==[2.]
    # If final dense checking fails, advancing the generator explores stage 2.
    assert list(choices)==[5.] and called==[2.,5.]
    assert report['candidates']==[2.,5.]
    called.clear();report={};choices=fast.proposal_stages(None,{}, {},60,lambda _:None,report)
    next(choices);report['expanded_alternatives']=True
    assert list(choices)==[] and called==[2.]


def test_invalid_endpoint_never_reaches_fitting_or_timing(monkeypatch):
    from types import SimpleNamespace
    oracle=SimpleNamespace(meta={'joint_limit_check':{}},check=lambda q:{'valid':[True,False]})
    monkeypatch.setattr(fast.fast_path.workspace,'CurveFit',lambda *a:pytest.fail('Fit before endpoint check'))
    rows,report=fast.fast_path.candidates(oracle,{'start':[0.]*7,'goal':[10.]*7},{},60,lambda _:None)
    assert rows==[] and report['reason']=='nominal_endpoint_invalid'


def test_unavailable_gpu_is_explicit_fast_cpu_not_reference(monkeypatch):
    from drum_planner.timing import adaptive,integration,gpu_integration
    def unavailable(*a):raise timing.GPUUnavailable('unit: no compute GPU')
    monkeypatch.setattr(timing,'Device',unavailable)
    for module,name in ((adaptive,'optimize'),(integration,'retime'),(gpu_integration,'cpu_retime')):
        monkeypatch.setattr(module,name,lambda *a,**k:pytest.fail('Old CPU reference was called'))
    logs=[];s=source();law=timing.optimize(s,logs.append)
    assert law['details']['actual_backend']=='cpu' and not law['details']['gpu_native_optimizer']
    assert law['details']['fallback_reason']=='unit: no compute GPU'
    assert not law['details']['cpu_reference_used'] and not law['details']['result_cache_used']
    assert any('GPU UNAVAILABLE' in line and 'CPU TIMING FALLBACK' in line for line in logs)
    validate(law,s,None,collision=False)


def test_gpu_execution_error_is_not_hidden_by_fallback(monkeypatch,tmp_path):
    class Broken:
        name='test';identity='test'
        def __init__(self,*a):pass
        def __enter__(self):return self
        def __exit__(self,*a):pass
        def step(self):raise timing.GPUExecutionError('device execution failed')
    monkeypatch.setattr(timing,'Device',Broken)
    monkeypatch.setattr(timing,'BUILD',tmp_path)
    monkeypatch.setattr(timing,'cpu_solve',lambda *a:pytest.fail('Unexpected fallback'))
    with pytest.raises(timing.GPUExecutionError):timing.optimize(source())


def test_bad_bounded_solution_cannot_make_uniform_clock_worse(monkeypatch):
    def unavailable(*a):raise timing.GPUUnavailable('test')
    monkeypatch.setattr(timing,'Device',unavailable)
    monkeypatch.setattr(timing,'cpu_solve',lambda B,q,w,x:(np.full_like(x,5.),1))
    s=source();logs=[];law=timing.optimize(s,logs.append)
    assert law['details']['uniform_clock_selected']
    assert law['duration_s']<=s['timing']['duration_s']*(1+1e-10)
    assert any('uniform clock' in line for line in logs)
    validate(law,s,None,collision=False)


@pytest.mark.parametrize('name',['rest_to_R1','R1_to_R4','R4_to_rest','left_workspace'])
def test_real_gpu_continuous_limits_geometry_and_offline_quality(name):
    s=source(name);law=timing.optimize(s)
    assert law['details']['actual_backend']=='vulkan'
    assert not law['details']['cpu_reference_used'] and not law['details']['result_cache_used']
    peaks,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']))
    np.testing.assert_allclose(peaks,law['peaks'],rtol=1e-8,atol=1e-9)
    assert core.scale_needed(peaks)<=1
    validate(law,s,None,collision=False)
    reference=json.loads((ROOT/'diagnostics/gpu_timing_20261024/benchmark-2'/name/'reference.json').read_text())
    assert law['duration_s']<=reference['duration_s']*1.10
    quality=motion_quality.law_metrics(law)
    assert quality['internal_stops']==quality['deep_slowdowns']==0


def test_protected_hardware_and_original_policies_unchanged():
    before=json.loads((ROOT/'diagnostics/fast_gpu_20261024/before_hashes.json').read_text())
    protected=[p for p in before if p.startswith(('drum_planner/baseline/','model/','destinations/','destination_capture/'))]
    protected += ['drum_planner/timing/'+p for p in ('core.py','dispatch.py','adaptive.py','integration.py',
        'smooth_v2.py','rounding.py','pipeline.py','sparse.py','sparse_integration.py','gpu_integration.py')]
    protected += ['drum_planner/experiment/'+p for p in ('workspace_curve.py','config.py','algorithms.py','search.py','runtime.py','checkpoints.py')]
    for name in protected:assert digest(ROOT/name)==before[name],name
