"""Experiment-only GPU dispatch, strict quality, explicit fallback, real FP64 math."""
import copy
import json
from pathlib import Path
import numpy as np
import pytest

from drum_planner.common import ROOT,digest
from drum_planner.timing import core,dispatch,gpu_integration as adapter
from drum_planner.timing.vulkan import device,optimizer


@pytest.fixture(autouse=True)
def offline(monkeypatch,tmp_path):
    from drum_planner.experiment.config import no_can
    no_can()
    monkeypatch.setenv('X5_PLANNER_CACHE_DIR',str(tmp_path/'cache'))


def source():
    return json.loads((ROOT/'diagnostics/planner_performance_20261021/inputs/right_single_arc.json').read_text())


@pytest.mark.parametrize('side',['right','left'])
def test_gpu_dispatch_requires_explicit_experiment_marker(monkeypatch,side):
    monkeypatch.setattr(adapter,'retime',lambda *a:'gpu')
    with pytest.raises(ValueError,match='experiment-only'):
        dispatch.retime({}, {}, {'moving_side':side}, {'timing_backend':'vulkan'}, '.', print)
    assert dispatch.retime({}, {}, {'moving_side':side},
        {'timing_backend':'vulkan','experiment_gpu_timing':True}, '.', print)=='gpu'


@pytest.mark.parametrize('side',['right','left'])
def test_cpu_fallback_is_unambiguous_and_routes_correct_arm(monkeypatch,tmp_path,side):
    def unavailable():raise device.GPUUnavailable('test: no compatible Vulkan GPU')
    monkeypatch.setattr(device,'probe',unavailable)
    called=[]
    def cpu(*args):
        called.append(args[2]['moving_side'])
        return [],{'duration_s':1.},{'samples_checked':1},{'label':'original CPU'}
    selected=adapter.smooth_v2 if side=='left' else adapter.integration
    monkeypatch.setattr(selected,'retime',cpu)
    logs=[]
    result=adapter.retime({}, {}, {'moving_side':side,'execute':False},
        {'experiment_gpu_timing':True},tmp_path,logs.append)
    assert called==[side]
    assert result[3]['actual_backend']=='cpu' and not result[3]['gpu_used']
    assert result[3]['requested_backend']=='vulkan'
    assert 'CPU FALLBACK' in result[3]['label']
    assert any('GPU UNAVAILABLE' in s and 'CPU TIMING FALLBACK' in s for s in logs)
    assert any('NOT optimized on the GPU' in s for s in logs)


def test_hardware_request_is_rejected_before_gpu_or_cpu(monkeypatch,tmp_path):
    monkeypatch.setattr(device,'probe',lambda:pytest.fail('Hardware path reached GPU'))
    with pytest.raises(ValueError,match='experiment mode'):
        adapter.retime({}, {}, {'execute':True},{'experiment_gpu_timing':True},tmp_path,print)


def test_reference_cannot_be_for_another_curve_or_limit():
    s=source();ref=copy.deepcopy(s['existing_law'])
    optimizer.reference_check(s,ref)
    ref['source_path_sha256']='not-the-source'
    with pytest.raises(ValueError,match='exact fixed-path'):
        optimizer.reference_check(s,ref)
    ref=copy.deepcopy(s['existing_law']);ref['limits'][0]*=2
    with pytest.raises(ValueError,match='limits differ'):optimizer.reference_check(s,ref)


@pytest.mark.parametrize('method',['single_arc','clearance','progress','sweep'])
def test_left_c3_reference_reconstructs_only_the_approved_rounded_geometry(method):
    base=ROOT/'diagnostics/corner_smoothing_20261021'
    original=json.loads((base/'inputs'/('left_'+method+'.json')).read_text())
    reference=json.loads((base/'selected'/('left_'+method)/'law.json').read_text())
    source=adapter.reference_source(dict(path=original['path_waypoints'],timing=original['timing']),
                                    original['request'],reference)
    optimizer.reference_check(source,reference)
    a,b=core.original(source),core.decode(reference['path'])
    assert np.array_equal(a.c,b.c) and np.array_equal(a.x,b.x)
    assert not source['timing']['stop_at_corners']


def test_one_percent_is_an_acceptance_gate_not_just_a_log():
    s=source();ref=s['existing_law'];law=copy.deepcopy(ref)
    law['duration_s']=ref['duration_s']*1.010001
    assert optimizer.quality_gate(law,ref)==(False,None)
    ok,quality=optimizer.quality_gate(ref,ref)
    assert ok and quality['duration_s']==ref['duration_s']


def test_numerical_failure_is_not_disguised_as_cpu_fallback(monkeypatch,tmp_path):
    s=source();ref=s['existing_law'];req=dict(s['request'],execute=False)
    winner=dict(path=s['path_waypoints'],timing=s['timing'])
    monkeypatch.setattr(device,'probe',lambda:{'name':'test AMD','identity':'test'})
    monkeypatch.setattr(device,'BUILD',tmp_path)
    calls=[]
    def cpu(*a):
        calls.append('reference')
        return [],copy.deepcopy(ref),{},dict(cached=True,label='CPU reference')
    monkeypatch.setattr(adapter,'cpu_retime',cpu)
    monkeypatch.setattr(adapter.math_cache,'solve',lambda s,c,fn,log:(fn(),{'cached':False}))
    def failure(*a,**k):raise device.GPUExecutionError('quality gate failed')
    monkeypatch.setattr(optimizer,'optimize',failure)
    logs=[]
    with pytest.raises(device.GPUExecutionError,match='quality gate'):
        adapter.retime(winner,{},req,{'experiment_gpu_timing':True},tmp_path,logs.append)
    assert calls==['reference']
    assert not any('CPU FALLBACK' in x for x in logs)


def test_cached_gpu_math_still_checks_new_scene_and_rejects_collision(monkeypatch,tmp_path):
    s=source();ref=s['existing_law'];req=dict(s['request'],execute=False)
    winner=dict(path=s['path_waypoints'],timing=s['timing'])
    monkeypatch.setattr(device,'probe',lambda:{'name':'mock GPU','identity':'mock'})
    monkeypatch.setattr(adapter,'cpu_retime',lambda *a:([],copy.deepcopy(ref),{},dict(cached=True,label='reference')))
    # Unit-only cached-law fixture. Real Vulkan output is tested independently.
    fixture=copy.deepcopy(ref);fixture['method']='vulkan_al_lbfgs'
    fixture['details']['gpu_native_optimizer']=True
    monkeypatch.setattr(adapter.math_cache,'solve',lambda *a:({'law':copy.deepcopy(fixture)},{'cached':True}))
    seen=[]
    def validate(law,src,folder):
        seen.append(src['request']['test_scene_revision'])
        if seen[-1]=='blocked':raise ValueError('new scene collision')
        return dict(collision_zone={'valid':True,'samples_checked':123})
    monkeypatch.setattr(adapter,'validate',validate)
    for revision in ('first','new','blocked'):
        req['test_scene_revision']=revision
        if revision=='blocked':
            with pytest.raises(ValueError,match='new scene collision'):
                adapter.retime(winner,{},req,{'experiment_gpu_timing':True},tmp_path,lambda _:None)
        else:
            result=adapter.retime(winner,{},req,{'experiment_gpu_timing':True},tmp_path,lambda _:None)
            assert result[3]['cached'] and result[3]['actual_backend']=='vulkan'
    assert seen==['first','new','blocked']


def test_missing_build_tools_are_gpu_unavailable(monkeypatch,tmp_path):
    monkeypatch.setattr(device,'BUILD',tmp_path)
    monkeypatch.setattr(device.shutil,'which',lambda name:None)
    with pytest.raises(device.GPUUnavailable,match='build tools missing'):
        device.build('test-unavailable-tools')


@pytest.mark.parametrize('backend,expected',[
    ('vulkan','GPU Vulkan FP64'),('cpu','CPU FALLBACK'),('analytic_cpu','Analytic CPU')])
def test_checkpoint_summary_identifies_backend_without_changing_hardware(monkeypatch,tmp_path,backend,expected):
    from drum_planner.experiment import worker,checkpoints
    calls=[]
    def sequence(context,*args,**kwargs):
        calls.append(context)
        return dict(status='planned',experiment=dict(checkpoint_reports=[{'retiming':{'actual_backend':backend}}],
            return_report={'retiming':{'actual_backend':backend}}))
    monkeypatch.setattr(checkpoints,'plan_sequence',sequence)
    # Retained historical adapter; the new fast workspace bypass is tested
    # separately and intentionally has no CPU qualification phase.
    result=worker.plan({'checkpoints':[{},{}]},tmp_path,'progress',{},120)
    assert calls[0]['timing_backend']=='vulkan' and calls[0]['experiment_gpu_timing']
    assert expected in result['experiment']['retiming']['label']
    assert json.loads((tmp_path/'plan.json').read_text())==result


def cpu_augmented_lagrangian(B,q,w,x):
    H,H1,H2=[(b@x)[:,None] for b in B];q1,q2,q3=q
    v=[q1/H,q2/H**2-q1*H1/H**3,
       q3/H**3-3*q2*H1/H**4-q1*H2/H**4+3*q1*H1**2/H**5]
    parts=[(-q1/H**2,None,None),(-2*q2/H**3+3*q1*H1/H**4,-q1/H**3,None),
           (-3*q3/H**4+12*q2*H1/H**5+4*q1*H2/H**5-15*q1*H1**2/H**6,
            -3*q2/H**4+6*q1*H1/H**5,-q1/H**4)]
    grad=w.copy();f=float(w@x)
    for d in range(3):
        z=np.maximum(0,np.abs(v[d])/core.LIMITS[d]-1)
        f+=np.sum(z*z)/2
        multiplier=z*np.sign(v[d])/core.LIMITS[d]
        for partial,b in zip(parts[d],B):
            if partial is not None:grad+=b.T@np.sum(multiplier*partial,axis=1)
    return f,grad


def test_real_gpu_optimizer_state_matches_independent_fp64_gradient():
    s=source();path=core.original(s);base=s['timing']['time_scale']/core.RESERVE
    _,_,_,B,q,w=optimizer.arrays(path,base,12)
    try:gpu=device.Device(B,q,w,np.ones(12))
    except device.GPUUnavailable as exc:pytest.skip(str(exc))
    with gpu:
        for _ in range(3):
            x,meta=gpu.step();f,g=cpu_augmented_lagrangian(B,q,w,x)
            assert 'AMD' in gpu.name
            np.testing.assert_allclose(meta[5],f,rtol=1e-8,atol=1e-9)
            np.testing.assert_allclose(gpu.snapshot[76:88],g,rtol=2e-7,atol=2e-8)
            assert np.all((x>=.005)&(x<=5))
        assert meta[6]==96
        assert not np.array_equal(x,np.ones(12))


def test_hardware_implementation_and_motion_policies_remain_byte_identical():
    # Compare all protected files to the post-sparse, pre-GPU snapshot.
    current=json.loads((ROOT/'diagnostics/gpu_timing_20261024/before_hashes.json').read_text())
    protected=[p for p in current if p.startswith('drum_planner/baseline/')]
    protected+=['drum_planner/timing/'+p for p in ('adaptive.py','core.py','selected.json',
        'selected_v2.json','integration.py','smooth_v2.py','rounding.py','pipeline.py','sparse.py','sparse_integration.py')]
    protected+=['drum_planner/experiment/'+p for p in ('algorithms.py','search.py','workspace_curve.py','config.py')]
    for name in protected:assert digest(ROOT/name)==current[name],name
