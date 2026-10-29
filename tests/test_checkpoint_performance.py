"""Exact math reuse, per-job inputs, and vector evaluation; no device access."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from drum_planner.common import ROOT,DEFAULT_CALIBRATION,digest,read_state
from drum_planner import planning
from drum_planner.timing import core,math_cache,integration
from drum_planner.experiment import cache,config,inputs
from drum_planner.baseline.curve import Curve

D=ROOT/'diagnostics/planner_optimization_20261022'
OPTIMIZED={'drum_planner/'+p for p in (
    'timing/core.py','timing/adaptive.py','timing/integration.py','timing/smooth_v2.py','timing/benchmark.py','timing/README.md',
    'baseline/planner.py','baseline/tasks.py','experiment/checkpoints.py','experiment/worker.py','experiment/cache.py')}
# New fork adds opt-in plumbing only; its own before-hash audit keeps every
# original numerical/model/actuator implementation byte-identical.
from test_sparse_hardware import WIRING
OPTIMIZED |= WIRING


@pytest.fixture(autouse=True)
def isolated(tmp_path,monkeypatch):
    monkeypatch.setenv('X5_PLANNER_CACHE_DIR',str(tmp_path/'math-cache'))
    from drum_planner.experiment.config import no_can
    no_can()


def source():
    return json.loads((ROOT/'diagnostics/planner_performance_20261021/inputs/right_single_arc.json').read_text())


def test_cache_key_includes_complete_clock_path_policy_and_environment(monkeypatch):
    s=source();cfg=integration.policy();key=math_cache.identity(s,cfg)
    other=copy.deepcopy(s);other['request']={'scene':'another','moving_side':'left','goal':'different'}
    assert math_cache.identity(other,cfg)==key # geometry is already expressed in exact joint coordinates
    for field in ('time_scale','stop_at_corners','coefficients','knots_unscaled_s'):
        changed=copy.deepcopy(s)
        if field=='time_scale':changed['timing'][field]+=1e-12
        elif field=='stop_at_corners':changed['timing'][field]=not changed['timing'][field]
        elif field=='coefficients':changed['timing'][field][0][0][0]+=1e-12
        else:changed['timing'][field][0]+=1e-12
        assert math_cache.identity(changed,cfg)!=key
    changed=copy.deepcopy(s);changed['path_waypoints'][0][0]+=1e-12
    assert math_cache.identity(changed,cfg)!=key
    assert math_cache.identity(s,dict(cfg,maxiter=999))!=key
    monkeypatch.setenv('OMP_NUM_THREADS','2');assert math_cache.identity(s,cfg)!=key


def test_pure_cache_corruption_off_and_failed_calculation_are_misses(monkeypatch):
    s=source();cfg=integration.policy();calls=[]
    def calculate():calls.append(1);return dict(law=copy.deepcopy(s['existing_law']))
    a,info=math_cache.solve(s,cfg,calculate);b,reuse=math_cache.solve(s,cfg,calculate)
    assert a==b and len(calls)==1 and not info['cached'] and reuse['cached']
    entry=cache.root()/info['key'];(entry/'math.json').write_text('{}')
    _,info=math_cache.solve(s,cfg,calculate);assert not info['cached'] and len(calls)==2
    monkeypatch.setenv('X5_PLANNER_CACHE_DIR','off')
    assert not math_cache.solve(s,cfg,calculate)[1]['cached']
    with pytest.raises(ValueError,match='solver failure'):
        math_cache.solve(s,cfg,lambda:(_ for _ in ()).throw(ValueError('solver failure')))


def test_brief_parallel_cache_lock_does_not_trigger_recalculation():
    import threading,time,fcntl
    s=source();cfg=integration.policy()
    math_cache.solve(s,cfg,lambda:dict(law=copy.deepcopy(s['existing_law'])))
    ready=threading.Event()
    def other_worker():
        with (cache.root()/'.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX);ready.set();time.sleep(.1)
    thread=threading.Thread(target=other_worker);thread.start();assert ready.wait(1)
    try:
        _,meta=math_cache.solve(s,cfg,lambda:pytest.fail('Brief lock contention must not redo timing'))
        assert meta['cached']
    finally:thread.join(2)


def test_live_timing_reuse_always_validates_new_scene_and_reports_collisions(tmp_path,monkeypatch):
    s=source();calls=[]
    winner=dict(path=s['path_waypoints'],timing=s['timing'],duration_s=s['timing']['duration_s'])
    def calculate(*a,**k):calls.append('solve');return copy.deepcopy(s['existing_law'])
    def validate(law,source,folder):
        revision=source['request']['scene']['test_revision'];calls.append(revision)
        if revision=='blocked':raise ValueError('current scene collision')
        return dict(collision_zone={'samples_checked':100})
    monkeypatch.setattr(integration.adaptive,'optimize',calculate)
    monkeypatch.setattr(integration,'validate',validate)
    def run(revision):
        req=copy.deepcopy(s['request']);req['scene']['test_revision']=revision
        return integration.retime(winner,{'pool_key':revision},req,
            {'pool_root':str(tmp_path/revision/'pool')},tmp_path/revision,lambda _:None)
    a=run('first');b=run('new-camera-frame')
    assert a[:3]==b[:3] and calls==['solve','first','new-camera-frame']
    assert not a[3]['math_cache']['cached'] and b[3]['math_cache']['cached']
    assert not b[3]['cached'] # no old-world collision proof reused
    with pytest.raises(ValueError,match='current scene collision'):run('blocked')
    assert calls==['solve','first','new-camera-frame','blocked']
    manifest=json.loads((cache.root()/b[3]['math_cache']['key']/'manifest.json').read_text())
    assert set(manifest['files'])=={'math.json'}


def test_cached_geometry_keeps_all_extrema_exact_and_memory_bounded():
    s=source();path=core.original(s);h=core.decode(s['existing_law']['density'])
    geometry=core.DensityGeometry(path)
    for scale in (1.,1.01,1.2):
        law=core.decode(core.encode(h));law.c*=scale
        a=core.density_peaks(path,law);b=core.density_peaks(path,law,geometry=geometry)
        assert all(np.array_equal(x,y) for x,y in zip(a,b))
    assert len(geometry.rows)==len(geometry.bounds)-1
    with pytest.raises(ValueError,match='another curve'):
        core.density_peaks(core.original(s),h,geometry=geometry)


@pytest.mark.parametrize('side',['right','left'])
def test_sequence_request_reuse_is_byte_identical_to_independent_request(tmp_path,monkeypatch,side):
    state,scene=config.saved_inputs(side);state=read_state(state)
    # This archived scene predates the camera move. Compare both request paths
    # against its actual calibration, never relabel it with today's default.
    calibration=ROOT/'calibration_sessions/20261017T211949Z-762f9908-reprocessed/calibration.json'
    ctx=dict(moving_side=side,scene=str(scene),calibration=str(calibration),_sequence_inputs={})
    calls=[];original=planning.scene
    def checked(*a,**kw):calls.append(1);return original(*a,**kw)
    monkeypatch.setattr(planning,'scene',checked)
    first,manifest=inputs.request(ctx,tmp_path/'first',state)
    state=copy.deepcopy(state) # actual legs read distinct saved/derived states
    state[side+'_rad'][0]+=.01
    second,second_manifest=inputs.request(ctx,tmp_path/'second',state)
    assert len(calls)==1 and second_manifest==manifest
    independent,m=planning.request(tmp_path/'independent',state,original(scene,calibration),calibration,moving_side=side)
    assert m==manifest
    for kind in ('urdf','srdf'):
        assert Path(second[kind]).read_bytes()==Path(independent[kind]).read_bytes()
        independent[kind]=second[kind]
    assert independent==second
    assert first['start']!=second['start']


@pytest.mark.parametrize('side',['right','left'])
def test_batched_evaluation_is_exactly_the_same_positions(side):
    p=ROOT/'diagnostics/corner_smoothing_20261021/selected'/(side+'_single_arc')
    s=json.loads((p/'source.json').read_text());law=json.loads((p/'law.json').read_text())
    c=Curve(dict(timing=law,outward=core.samples(law,s)))
    times=np.r_[np.arange(0,c.duration,.002),c.duration]
    assert np.array_equal(c.at_many(times),np.array([c.at(t) for t in times]))


def test_policies_geometry_inputs_and_motor_control_are_unchanged():
    before=json.loads((D/'before_hashes.json').read_text())
    for name,sha in before.items():
        if name not in OPTIMIZED:assert digest(ROOT/name)==sha,name
    for name in ('core.py','adaptive.py','selected.json','selected_v2.json','integration.py','smooth_v2.py'):
        assert digest(D/'reference'/name)==before['drum_planner/timing/'+name]
