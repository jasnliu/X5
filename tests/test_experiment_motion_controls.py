"""Custom experiment timing/spawn contracts. No physical devices or commands."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.interpolate import PPoly

from drum_planner import cli
from drum_planner.common import ROOT,atomic_json,digest,endpoint,read_state,goal_recording
from drum_planner.experiment import motion_settings as cfg, motion_timing as timing, scenario, stationary
from drum_planner.timing import core


def source():
    d=np.array([.22,-.15,.1,.28,-.17,.3,.11])
    c=np.array([6*d,-15*d,10*d,np.zeros(7),np.zeros(7),np.zeros(7)])[:,None,:]
    return dict(path_waypoints=[np.zeros(7).tolist(),d.tolist()],request={},
        timing=dict(stop_at_corners=False,time_scale=2.,duration_s=2.,
            coefficients=np.moveaxis(c,-1,0).tolist(),polynomial_breaks_unscaled_s=[[0.,1.]]*7))


@pytest.mark.parametrize('side',['right','left'])
def test_defaults_and_profile_roundtrip(tmp_path,side):
    p=cfg.settings(side);assert p['timing_quality']=='balanced'
    assert np.array_equal(cfg.matrix(p),cfg.DEFAULT_LIMITS)
    path=tmp_path/'profile.json';cfg.save(path,p);assert cfg.load(path,side)==p
    assert cfg.identity(p)==cfg.identity(dict(p,name='Display name only'))
    with pytest.raises(ValueError,match='other arm'):cfg.load(path,'left' if side=='right' else 'right')


@pytest.mark.parametrize('bad',[0.,-1.,float('nan'),float('inf'),True])
def test_reject_bad_limits_without_clipping(bad):
    x=cfg.DEFAULT_LIMITS.tolist();x[1][4]=bad
    with pytest.raises(ValueError):cfg.checked_limits(x)


@pytest.mark.parametrize('data',[
    {'velocity_rad_s':[1.]*7},{'timing_quality':'unbounded'},
    {'current_limit':99},{'simulation_only':False},{'name':''},
    {'velocity_rad_s':[1.]*6,'acceleration_rad_s2':[1.]*7,'jerk_rad_s3':[1.]*7}])
def test_profiles_reject_incomplete_unknown_or_unsafe_semantics(data):
    with pytest.raises(ValueError):cfg.settings('right',data)


@pytest.fixture(params=['right','left'])
def session(tmp_path,request):
    from drum_planner.experiment.config import saved_inputs
    side=request.param;state_path,_=saved_inputs(side);state=read_state(state_path)
    initial=tmp_path/'initial';initial.mkdir()
    req=dict(start=state[side+'_rad'],left=state['left_rad'],right=state['right_rad'],
             urdf=str(ROOT/'model/openarmx.urdf'),scene={'immutable':True},execute=False)
    plan=dict(status='not_generated',start_state=state,provenance=dict(state=str(state_path),state_sha256=digest(state_path)))
    context=dict(state=str(state_path),state_sha256=digest(state_path),moving_side=side,scene='unchanged')
    for p,data in [(initial/'plan.json',plan),(initial/'backend_request.json',req),(tmp_path/'context.json',context)]:atomic_json(p,data)
    return tmp_path,side,state


def test_spawn_roundtrip_never_changes_perception_or_passive_arm(session):
    folder,side,original=session;opposite='left' if side=='right' else 'right'
    record=ROOT/f'destinations/{side}'/('destinationR1.json' if side=='right' else 'destinationL1.json')
    before=digest(record);context=scenario.set_start(folder,record);derived=read_state(context['state'])
    req=json.loads((folder/'initial/backend_request.json').read_text())
    assert req['start']==endpoint(record,side=side).tolist()==derived[side+'_rad']
    assert derived[opposite+'_rad']==original[opposite+'_rad']
    assert req['scene_robot_start']==original[side+'_rad'] and req['scene']=={'immutable':True}
    assert not req['execute'] and derived['preview_only'] and 'motor_telemetry' not in derived
    old_path=Path(context['state']);old_bytes=old_path.read_bytes()
    restored=scenario.set_start(folder,None)
    assert read_state(restored['state'])==original and old_path.read_bytes()==old_bytes and digest(record)==before
    assert restored['experiment_start']['kind']=='default'


def test_default_gui_start_has_acquisition_mask_without_teleport(session):
    folder,side,original=session
    context=scenario.ensure(folder)
    assert context['experiment_start']['kind']=='default'
    assert read_state(context['state'])==original
    assert context['scene_robot_start']==original[side+'_rad']
    assert scenario.ensure(folder)==context


def test_invalid_start_does_not_change_current_context(session):
    folder,side,_=session;context=scenario.set_start(folder,None);before=(folder/'context.json').read_bytes()
    bad=json.loads(goal_recording(side).read_text());bad['samples'][-1]['positions_rad'][0]=100.
    path=folder/'bad.json';atomic_json(path,bad)
    with pytest.raises(ValueError,match='position limits'):scenario.set_start(folder,path)
    assert (folder/'context.json').read_bytes()==before
    with pytest.raises(ValueError):scenario.set_start(folder,goal_recording('left' if side=='right' else 'right'))


def test_acquired_state_tamper_detected(session):
    folder,side,original=session;local=folder/'measured.json';atomic_json(local,original)
    context=json.loads((folder/'context.json').read_text());context.update(state=str(local),state_sha256=digest(local))
    atomic_json(folder/'context.json',context);scenario.ensure(folder)
    original[side+'_rad'][0]+=.01;atomic_json(local,original)
    with pytest.raises(ValueError,match='Acquired state changed'):scenario.set_start(folder,None)


def test_per_joint_seed_gradient():
    from drum_planner.timing.vulkan.optimizer import arrays
    _,_,_,B,q,w=arrays(core.original(source()),1.,10)
    limits=cfg.DEFAULT_LIMITS*np.linspace(.7,1.6,7)
    p=timing.SeedProblem(B,q,w,limits);x=np.linspace(.6,1.1,10);_,g,_=p.calc(x)
    numerical=[]
    for e in np.eye(10)*1e-6:numerical.append((p.calc(x+e)[0]-p.calc(x-e)[0])/(2e-6))
    np.testing.assert_allclose(g,numerical,rtol=2e-5,atol=2e-5)


@pytest.mark.parametrize('profile',['original','unequal','faster','slower'])
def test_actual_optimizer_custom_limits_continuous_and_dense(profile):
    limits=cfg.DEFAULT_LIMITS.copy()
    if profile=='unequal':limits*=np.array([.7,1.2,.8,2.,.6,1.7,1.1])[None,:]**np.arange(1,4)[:,None]
    elif profile=='faster':limits*=2.**np.arange(1,4)[:,None]
    elif profile=='slower':limits*=.5**np.arange(1,4)[:,None]
    original=core.LIMITS.copy();s=source();law=timing.optimize(s,limits=limits,quality='fast')
    assert np.array_equal(core.LIMITS,original) and law['limits']==limits.tolist()
    assert law['path']==core.encode(core.original(s))
    peaks,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']))
    np.testing.assert_allclose(peaks,law['peaks'],rtol=1e-7,atol=1e-8)
    assert np.all(peaks<=limits) and np.max(peaks/limits)>.99
    timing.validate(law,s,None,collision=False)


def test_homogeneous_limits_halve_timing_not_change_path():
    s=source();a=timing.optimize(s,limits=cfg.DEFAULT_LIMITS,quality='fast')
    b=timing.optimize(s,limits=cfg.DEFAULT_LIMITS*2.**np.arange(1,4)[:,None],quality='fast')
    assert b['duration_s']==pytest.approx(a['duration_s']/2,rel=3e-5)
    assert a['path']==b['path']


@pytest.mark.parametrize('failure',['missing','invalid','worse'])
def test_solver_failure_checked_fallback_explicit(monkeypatch,failure):
    def solve(B,q,w,x,*args):
        if failure=='missing':raise ImportError('unavailable test solver')
        if failure=='invalid':return np.full_like(x,np.nan),{}
        return np.ones_like(x),dict(converged=True,status='test',iterations=0)
    monkeypatch.setattr(timing,'sparse_solve',solve)
    logs=[];law=timing.optimize(source(),logs.append,limits=cfg.DEFAULT_LIMITS,quality='balanced')
    assert law['details']['fallback_used'] and any('FALLBACK' in s for s in logs)
    timing.validate(law,source(),None,collision=False)


def test_constant_clock_scaled_down_to_use_requested_limits():
    s=source();p=core.original(s)
    law=timing.certify(p,PPoly(np.array([[100.]]),[0.,1.]),s,cfg.DEFAULT_LIMITS)
    assert law['duration_s']<100 and np.max(np.array(law['peaks'])/cfg.DEFAULT_LIMITS)>.99


def test_stationary_pose_is_zero_movement_but_checked():
    oracle=SimpleNamespace(meta={'joint_limit_check':{}},check=lambda q:{'valid':[True]})
    req={'start':[.1]*7};report={};c=stationary.candidate(req,oracle,report)[0]
    s=dict(path_waypoints=c['path'],timing=c['timing'],request={})
    law=stationary.law(s,cfg.DEFAULT_LIMITS,'balanced')
    assert law['duration_s']==0 and np.array_equal(core.evaluate(law,[0.,1.])[0],[[.1]*7]*2)
    timing.validate(law,s,None,collision=False)
    oracle.check=lambda q:{'valid':[False]}
    assert stationary.candidate(req,oracle,{})==[]


@pytest.mark.parametrize('args',[
    ['--start','does-not-exist'],['--destinations','default'],['--destinations','R1','R1'],
    ['--start',str(ROOT/'destinations/left/destinationL1.json')]])
def test_invalid_cli_options_do_not_acquire_devices(monkeypatch,args):
    from drum_planner.experiment import app
    monkeypatch.setattr(app,'prepare',lambda *a,**k:pytest.fail('Invalid input reached acquisition'))
    assert cli.main(['experiment','--right',*args])==1


def test_invalid_profile_before_acquisition(monkeypatch,tmp_path):
    from drum_planner.experiment import app
    path=tmp_path/'bad.json';atomic_json(path,{'velocity_rad_s':[0.]*7})
    monkeypatch.setattr(app,'prepare',lambda *a,**k:pytest.fail('Invalid profile reached acquisition'))
    assert cli.main(['experiment','--motion-profile',str(path)])==1


def test_custom_spawn_cannot_replace_actual_camera_mask(session):
    from drum_planner.experiment import fast
    folder,side,original=session
    record=ROOT/f'destinations/{side}'/('destinationR1.json' if side=='right' else 'destinationL1.json')
    context=scenario.set_start(folder,record)
    context['scene_robot_start']=endpoint(record,side=side).tolist()
    if context['scene_robot_start']==original[side+'_rad']:context['scene_robot_start'][0]+=.01
    with pytest.raises(ValueError,match='self-mask'):
        fast.plan(context,folder/'bad','workspace_curve',{},10)


@pytest.mark.parametrize('coefficients',[[[1.,2.]],[[np.nan,1.]],[[-1.,1.]]])
def test_invalid_densities_rejected(coefficients):
    s=source()
    with pytest.raises(ValueError):
        timing.certify(core.original(s),PPoly(np.array(coefficients),[0.,.5,1.]),s,cfg.DEFAULT_LIMITS)


def test_original_hardware_fast_mode_and_path_generator_preserved():
    before=json.loads((ROOT/'diagnostics/experiment_controls_20261025/before_hashes.json').read_text())
    prefixes=('drum_planner/baseline/','drum_planner/fast_mode/','model/','destinations/',
              'recordings/','left_recordings/','calibration_sessions/','camera_calibration/',
              'safe_zone/','centering/','camera_playback/')
    # The later authorized right-only dt1 integration changes acquisition.py
    # and inputs.py (paired RGB/depth reader), not paths, clocks or hardware.
    # test_experiment_dt1/acquisition cover that routing and unchanged fusion.
    exact=['drum_planner/experiment/'+n for n in ('fast_path.py','fast_timing.py','workspace_curve.py',
            'config.py','algorithms.py','checkpoints.py','checkpoint_ui.py','runtime.py')]
    protected=[p for p in before if p.startswith(prefixes) or p in exact or
               (p.startswith('drum_planner/timing/') and not p.endswith('README.md'))]
    assert protected
    for name in protected:assert digest(ROOT/name)==before[name],name
