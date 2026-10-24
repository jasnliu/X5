"""Exact-output numerical optimizations and optional persistent cache isolation."""
from test_checkpoint_performance import OPTIMIZED
import copy
import fcntl
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from drum_planner.common import ROOT, digest
from drum_planner.experiment import cache
from drum_planner.timing import core, adaptive, integration

D = ROOT / 'diagnostics/planner_performance_20261021'
spec = importlib.util.spec_from_file_location('performance_comparison', D / 'compare.py')
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)
reference = comparison.reference()


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    monkeypatch.setenv('X5_PLANNER_CACHE_DIR', str(tmp_path / 'persistent'))
    import socket
    original = socket.socket
    def guard(family=socket.AF_INET, *a, **kw):
        if family == socket.AF_CAN: pytest.fail('No physical CAN in performance tests')
        return original(family, *a, **kw)
    monkeypatch.setattr(socket, 'socket', guard)


def source(case='right_clearance'):
    return json.loads((D / 'inputs' / (case + '.json')).read_text())


def numerical(law):
    law = copy.deepcopy(law)
    law['details'].pop('calculation_seconds', None)
    for step in law['details'].get('tuning', []): step.pop('elapsed_s', None)
    return law


@pytest.mark.parametrize('case', ['right_clearance', 'right_single_arc', 'left_single_arc', 'left_clearance'])
def test_identical_optimizer_iterates_and_law(case):
    s = source(case)
    args = dict(levels=(8, 12), ftol=1e-11, maxiter=1000)
    old = reference.optimize(s, **args)
    new = adaptive.optimize(s, **args)
    assert json.dumps(numerical(old),sort_keys=True) == json.dumps(numerical(new),sort_keys=True)
    t = np.linspace(0, old['duration_s'], 501)
    for before, after in zip(core.evaluate(old, t), core.evaluate(new, t)):
        assert np.array_equal(before, after)


def test_interval_derivative_hoisting_is_bit_identical():
    old_core = __import__('x5_timing_reference.core', fromlist=[''])
    for case in ('right_clearance', 'left_single_arc'):
        s = source(case); path = core.original(s); h = core.decode(s['existing_law']['density'])
        for a, b in zip(old_core.density_peaks(path, h), core.density_peaks(path, h)):
            assert np.array_equal(a, b)
        for a, b in zip(old_core.polynomial_peaks(path), core.polynomial_peaks(path)):
            assert np.array_equal(a, b)
    with pytest.raises(ValueError): core.local(path,0,.1)


@pytest.mark.parametrize('case', ['single_arc', 'clearance'])
def test_rounded_left_curves_keep_identical_timing(case):
    s=json.loads((ROOT/'diagnostics/corner_smoothing_20261021/selected'/('left_'+case)/'source.json').read_text())
    args=dict(levels=(8,16),ftol=1e-9,maxiter=400)
    assert numerical(reference.optimize(s,**args)) == numerical(adaptive.optimize(s,**args))


def test_preserved_generation_policies_and_hardware_sources():
    before = json.loads((D / 'before_hashes.json').read_text())
    authorized = {'drum_planner/experiment/algorithms.py', 'drum_planner/experiment/runtime.py',
                  'drum_planner/timing/adaptive.py', 'drum_planner/timing/core.py',
                  'drum_planner/timing/integration.py', 'drum_planner/timing/smooth_v2.py'}
    # Later authorized fifth experiment method, isolated from the shared pool.
    # Its own before/after audit retains the original generation/timing hashes.
    authorized.update({'drum_planner/experiment/config.py','drum_planner/experiment/worker.py',
                       'drum_planner/experiment/app.py','drum_planner/cpp/experiment_oracle.cpp'})
    # Subsequent hardware Workspace/checkpoint adaptation. The new audit in
    # test_hardware_workspace retains every generator, timing, model and input
    # hash from immediately before that adaptation (no quality policy changes).
    authorized.add('drum_planner/cli.py')
    authorized.update('drum_planner/baseline/'+n+'.py' for n in
        ('__init__','planner','curve','motion','tasks','workspace','windows','work','computation','service','transport'))
    # Approved 5 mm map wiring; test_depth_resolution protects all generation/timing/data.
    authorized.update({'drum_planner/depth.py','drum_planner/planning.py','drum_planner/cpp/moveit_plan.cpp','drum_planner/experiment/worker.py','drum_planner/experiment/algorithms.py','drum_planner/baseline/work.py','drum_planner/live_view.py'})
    for path, sha in before.items():
        if path not in authorized|OPTIMIZED: assert digest(ROOT / path) == sha, path
    for name in ('core.py', 'adaptive.py', 'integration.py', 'selected.json'):
        assert digest(D / 'reference' / name) == before['drum_planner/timing/' + name]


def test_persistent_roundtrip_checks_every_file_before_restoring(tmp_path):
    src = tmp_path / 'src'; src.mkdir()
    (src / 'law.json').write_text('{"exact": 1.23456789012345}')
    (src / 'verification.json').write_text('{"valid": true}')
    key = 'a' * 64; names = ['law.json', 'verification.json']
    assert cache.publish(key, src, names)
    target = tmp_path / 'target'
    assert cache.restore(key, target)
    for n in names: assert (src / n).read_bytes() == (target / n).read_bytes()
    (cache.root() / key / 'law.json').write_text('{}')
    assert not cache.restore(key, tmp_path / 'bad')
    assert not (tmp_path / 'bad').exists()


def test_cache_full_busy_off_unwritable_and_incomplete_are_misses(tmp_path, monkeypatch):
    src = tmp_path / 'src'; src.mkdir(); (src / 'law.json').write_text('{}')
    key = 'b' * 64
    assert not cache.restore(key, tmp_path / 'missing')
    cache.root().mkdir(exist_ok=True)
    with (cache.root() / '.lock').open('w') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert not cache.publish(key, src, ['law.json'])
        assert not cache.restore(key, tmp_path / 'busy')
    monkeypatch.setattr(cache, 'MAX_ENTRY_BYTES', 1)
    assert not cache.publish(key, src, ['law.json'])
    monkeypatch.setenv('X5_PLANNER_CACHE_DIR', 'off')
    assert not cache.publish(key, src, ['law.json'])
    assert not cache.restore(key, tmp_path / 'disabled')
    monkeypatch.setenv('X5_PLANNER_CACHE_DIR', str(src / 'law.json'))
    assert not cache.publish(key, src, ['law.json'])


def test_cache_limits_and_abandoned_stages(tmp_path, monkeypatch):
    monkeypatch.setattr(cache, 'MAX_ENTRIES', 2)
    src = tmp_path / 'src'; src.mkdir(); (src / 'law.json').write_text('{}')
    for i in range(4): assert cache.publish(str(i) * 64, src, ['law.json'])
    assert len([p for p in cache.root().iterdir() if len(p.name) == 64]) == 2
    stale = cache.root() / '.stage-cancelled'; stale.mkdir()
    assert cache.restore('3' * 64, tmp_path / 'hit')
    assert not stale.exists()
    assert not cache.publish('c' * 64, src, ['../law.json'])


def test_cache_identity_changes_for_every_relevant_input_and_not_output_folder(tmp_path):
    ctx = dict(saved_not_live=True, can_access=False); req = source()['request']
    key = cache.identity(ctx, req, 'test', 'algorithm-and-policy-sha')
    assert key is not None
    moved = copy.deepcopy(req)
    for kind in ('urdf', 'srdf'):
        target = tmp_path / kind; target.write_bytes(Path(req[kind]).read_bytes()); moved[kind] = str(target)
    assert cache.identity(ctx, moved, 'test', 'algorithm-and-policy-sha') == key
    for field in ('start', 'goal', 'left'):
        changed = copy.deepcopy(req); changed[field][0] += 1e-12
        assert cache.identity(ctx, changed, 'test', 'algorithm-and-policy-sha') != key
    for field in ('collision_policy', 'world_padding_policy', 'scene'):
        changed = copy.deepcopy(req); changed[field] = 'changed'
        assert cache.identity(ctx, changed, 'test', 'algorithm-and-policy-sha') != key
    assert cache.identity(ctx, req, 'test', 'changed-policy') != key
    assert cache.identity(ctx, req, 'other', 'algorithm-and-policy-sha') != key
    assert cache.identity({}, req, 'test', 'algorithm-and-policy-sha') is None
    assert cache.identity(dict(saved_not_live=True, can_access=True), req, 'test', 'algorithm-and-policy-sha') is None


def test_timing_cache_survives_new_session_and_never_reuses_other_scene(tmp_path, monkeypatch):
    s = source(); req = s['request']
    winner = dict(path=s['path_waypoints'], timing=s['timing'], duration_s=s['timing']['duration_s'])
    calls = []
    def calculate(*a, **kw): calls.append('calculate'); return copy.deepcopy(s['existing_law'])
    def validate(*a, **kw): calls.append('validate'); return {'collision_zone': {'samples_checked': 100}}
    monkeypatch.setattr(integration.adaptive, 'optimize', calculate)
    monkeypatch.setattr(integration, 'validate', validate)
    def run(name, request=req):
        context = dict(saved_not_live=True, can_access=False, pool_root=str(tmp_path / name / 'pool'))
        return integration.retime(winner, {'pool_key': 'same'}, request, context, tmp_path / name, lambda _: None)
    first = run('session1'); second = run('session2')
    assert calls == ['calculate', 'validate']
    assert not first[3]['cached'] and second[3]['cached'] and first[:3] == second[:3]
    changed = copy.deepcopy(req); changed['scene']['synthetic'] = True
    assert not run('session3', changed)[3]['cached']
    assert calls == ['calculate', 'validate', 'validate'] # reuse math, recheck the changed scene
