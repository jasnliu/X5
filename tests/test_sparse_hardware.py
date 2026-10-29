"""Opt-in fork routing, unchanged original numerical policy, no real devices."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from drum_planner.common import ROOT,atomic_json,digest
from drum_planner.timing import core,dispatch,sparse,integration

WIRING={'drum_planner/'+p for p in ('cli.py','baseline/workspace.py','baseline/work.py',
    'baseline/computation.py','baseline/tasks.py','baseline/planner.py','timing/dispatch.py')}
# Authorized experiment-only GPU routing and its CPU-qualification timeout.
# test_gpu_timing separately guards unchanged hardware/numerical implementations.
WIRING |= {'drum_planner/experiment/worker.py','drum_planner/experiment/app.py'}
# Authorized new camera calibration default only. The strict normalized source
# hash in test_calibration_default forbids any other common.py change.
WIRING |= {'drum_planner/common.py'}


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    import socket
    original=socket.socket
    def guarded(family=socket.AF_INET,*a,**kw):
        if family==socket.AF_CAN:pytest.fail('No physical CAN')
        return original(family,*a,**kw)
    monkeypatch.setattr(socket,'socket',guarded)
    from drum_planner import depth
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**kw:pytest.fail('No camera'))
    monkeypatch.setenv('OPENBLAS_NUM_THREADS','1');monkeypatch.setenv('OMP_NUM_THREADS','1')


def test_original_sources_only_have_explicit_opt_in_wiring_changes():
    before=json.loads((ROOT/'diagnostics/sparse_hardware_20261023/before_hashes.json').read_text())
    for name,sha in before.items():
        if name not in WIRING:assert digest(ROOT/name)==sha,name


@pytest.mark.parametrize('side',['right','left'])
@pytest.mark.parametrize('backend',[None,'sparse'])
def test_dispatch_never_changes_default_backend(monkeypatch,side,backend):
    from drum_planner.timing import integration,smooth_v2,sparse_integration
    for module,name in ((integration,'old_right'),(smooth_v2,'old_left'),(sparse_integration,'sparse')):
        monkeypatch.setattr(module,'retime',lambda *a,n=name:n)
    context={} if backend is None else dict(timing_backend=backend)
    assert dispatch.retime({}, {}, {'moving_side':side}, context, '.', print)==(backend or 'old_'+side)


@pytest.mark.parametrize('side',['right','left'])
@pytest.mark.parametrize('command',['--hardware','--hardware-sparse'])
def test_cli_selects_fork_explicitly(monkeypatch,tmp_path,side,command):
    from drum_planner import cli
    from drum_planner.baseline import workspace
    calls=[];monkeypatch.setattr(workspace,'show',lambda *a,**kw:calls.append((a,kw)))
    assert cli.main([command,'--'+side,'--simulate','not-opened-by-mock.json','--output',str(tmp_path/'ui')])==0
    options=calls[0][1]
    assert options['moving_side']==side
    assert options.get('timing_backend')==('sparse' if command=='--hardware-sparse' else None)


@pytest.mark.parametrize('backend',['current','adaptive','sparse'])
def test_computation_forwards_only_requested_fork(monkeypatch,tmp_path,backend):
    from drum_planner.baseline import computation
    calls=[]
    def run(command,worker):
        calls.append(command);atomic_json(tmp_path/'computation/result.json',{'test':True});return 0
    monkeypatch.setattr(computation,'_run',run)
    assert computation.run(tmp_path,ROOT/'cal.json',SimpleNamespace(error=None),moving_side='left',timing_backend=backend)=={'test':True}
    assert ('--timing-backend' in calls[0])==(backend!='current')
    assert calls[0][calls[0].index('--side')+1]=='left'


def source():
    return json.loads((ROOT/'diagnostics/planner_performance_20261021/inputs/right_single_arc.json').read_text())


def test_sparse_constraints_equal_original_joint_limits_and_have_local_support():
    import casadi as ca
    from scipy.interpolate import BSpline
    s=source();path=core.original(s);n=8;grid=np.linspace(0,1,17)
    basis=BSpline(np.r_[np.zeros(4),np.linspace(0,1,n-2)[1:-1],np.ones(4)],np.eye(n),3)
    base=s['timing']['time_scale']/core.RESERVE
    solver,structure=sparse.problem(path,basis,grid,base,1e-11,1000)
    x=np.linspace(.8,1.1,n)
    g=np.asarray(solver.get_function('nlp_g')(x,ca.DM())).ravel()
    H,H1,H2=[(basis.derivative(d)(grid)*base@x)[:,None] for d in range(3)]
    q1,q2,q3=[path(grid,nu=d) for d in (1,2,3)]
    vals=[q1/H,q2/H**2-q1*H1/H**3,q3/H**3-3*q2*H1/H**4-q1*H2/H**4+3*q1*H1**2/H**5]
    expected=np.concatenate([(v/core.LIMITS[d]).ravel(order='F') for d,v in enumerate(vals)])
    assert np.allclose(g,expected,rtol=1e-12,atol=1e-12)
    assert structure['jacobian_nnz']<=structure['constraints']*4
    assert structure['jacobian_nnz']<structure['dense_jacobian_entries']


def test_sparse_law_preserves_curve_and_continuous_limits():
    from drum_planner.timing.benchmark import validate
    s=source();cfg=integration.policy()
    law=sparse.optimize(s,levels=(8,),ftol=cfg['ftol'],maxiter=cfg['maxiter'])
    proof=validate(law,s,Path('/unused'),collision=False)
    assert proof['geometry']['max_error_rad']==0
    assert law['path']==core.encode(core.original(s))
    assert np.all(np.array(law['peaks'])<=core.LIMITS[:,None]*(1+1e-8))
    assert law['duration_s']<=core.global_law(s)['duration_s']
    assert law['details']['tuning'][0]['attempts'][0]['jacobian_nnz']>0


def test_current_timing_policies_have_not_been_shortened_or_loosened():
    before=json.loads((ROOT/'diagnostics/sparse_hardware_20261023/before_hashes.json').read_text())
    for name in ('selected.json','selected_v2.json','adaptive.py','core.py','integration.py','smooth_v2.py'):
        assert digest(ROOT/'drum_planner/timing'/name)==before['drum_planner/timing/'+name]


def test_sparse_rejects_unsupported_path_instead_of_changing_geometry():
    s=source();s['timing']['stop_at_corners']=True
    with pytest.raises(ValueError,match='continuous Workspace curve'):
        sparse.optimize(s,levels=(8,),ftol=1e-11,maxiter=1000)
