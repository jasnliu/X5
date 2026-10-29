"""No-device tests for the separate task-preserving --fast planner."""
import json
from pathlib import Path
import numpy as np
import pytest
from drum_planner.common import DEFAULT_CALIBRATION,ROOT,arm_joints
from drum_planner.model import generate,stick_length
from drum_planner.fast_mode import app,planner
from drum_planner.fast_mode.kinematics import Kinematics,target,error,task_proof
from drum_planner.fast_mode.postures import solve


@pytest.fixture(params=['right','left'])
def kin(request,tmp_path):
    generate(tmp_path,DEFAULT_CALIBRATION,moving_side=request.param)
    return Kinematics(tmp_path/'robot.urdf',request.param)


def test_fk_matches_independent_tree(kin,tmp_path):
    from safe_zone.geometry import Model
    import xml.etree.ElementTree as ET
    path=tmp_path/'robot.urdf';tree=Model(path);root=ET.parse(path).getroot()
    from drum_planner.fast_mode.kinematics import transform
    local=transform(root.find(f"link[@name='{kin.side}_drumstick']/collision/origin"))
    for q in [(kin.lower+kin.upper)/2,kin.lower*.3+kin.upper*.7]:
        frames=tree.transforms(dict(zip(arm_joints(kin.side),q)))
        T=frames[kin.side+'_drumstick']@local;f=kin.fk(q)
        np.testing.assert_allclose(f['tip'],T[:3,3]+T[:3,2]*stick_length(root,kin.side+'_drumstick')/2,atol=1e-12)
        np.testing.assert_allclose(f['R'],T[:3,:3],atol=1e-12)
        np.testing.assert_allclose(f['wrist'],frames[f'openarmx_{kin.side}_hand'][:3,3],atol=1e-12)


@pytest.mark.parametrize('kind',['full-pose','tip-axis'])
def test_analytic_task_jacobian(kin,kind):
    nominal=(kin.lower+kin.upper)/2;goal=target(kin,nominal);q=nominal+.03
    _,J=error(kin,q,goal,kind,True);step=1e-6
    finite=np.column_stack([(error(kin,q+np.eye(7)[i]*step,goal,kind)-
                            error(kin,q-np.eye(7)[i]*step,goal,kind))/(2*step) for i in range(7)])
    np.testing.assert_allclose(J,finite,atol=1e-7,rtol=1e-6)


def test_task_proof_rejects_changed_contact(kin):
    q=(kin.lower+kin.upper)/2;goal=target(kin,q)
    assert task_proof(kin,q,goal,'full-pose')['valid']
    shifted=q.copy();shifted[0]+=.05
    assert not task_proof(kin,shifted,goal,'tip-axis')['valid']


@pytest.mark.parametrize('kind',['full-pose','tip-axis'])
@pytest.mark.parametrize('count',[1,2])
def test_postures_preserve_tasks_limits_and_proximal_cap(kin,kind,count):
    center=(kin.lower+kin.upper)/2
    nominal=np.array([center+np.array([.08,-.03,.02,.04,-.03,.07,-.04])*i for i in range(1,count+1)])
    before=nominal.copy();rows,report=solve(kin,nominal,start=center,kind=kind,seconds=2.)
    np.testing.assert_array_equal(nominal,before)
    assert rows and any(r['source']=='captured_postures' for r in rows)
    cap=sum(abs(nominal[0,:4]-center[:4])) if count==1 else sum(abs(nominal[1,:4]-nominal[0,:4]))
    for row in rows:
        q=np.array(row['poses'])
        assert np.all(q>=kin.lower) and np.all(q<=kin.upper)
        assert all(p['valid'] for p in row['proofs'])
        assert row['proximal_travel_rad']<=cap+1e-5
    assert report['proximal_travel_not_increased']


def test_out_of_range_pose_not_clipped(kin):
    q=(kin.lower+kin.upper)/2;q[0]=kin.upper[0]+.01
    with pytest.raises(ValueError,match='exceeds joint limits'):solve(kin,[q],start=kin.lower)


def test_tradeoff_is_bounded():
    def row(t,p):return (dict(transfer_movement_s=t,actual_proximal_travel_rad=p),None,None)
    fastest=row(1.,3.);preferred=row(1.04,2.);too_slow=row(1.06,1.)
    ranked=planner.order_finalists([too_slow,fastest,preferred])
    assert ranked==[preferred,fastest,too_slow]


def test_timing_override_restored_on_failure(monkeypatch):
    from drum_planner.experiment import fast_timing
    from drum_planner.timing import core
    before=(fast_timing.MAX_ITERATIONS,fast_timing.SOLVE_WALL_SECONDS,core.LIMITS.copy())
    def fail(*args):raise RuntimeError('deliberate')
    monkeypatch.setattr(fast_timing,'optimize',fail)
    with pytest.raises(RuntimeError):planner.timed({},print)
    assert (fast_timing.MAX_ITERATIONS,fast_timing.SOLVE_WALL_SECONDS)==before[:2]
    np.testing.assert_array_equal(core.LIMITS,before[2])


def test_fast_dispatch_is_separate(monkeypatch):
    from drum_planner import cli
    seen=[];monkeypatch.setattr(app,'main',lambda argv:seen.append(argv) or 17)
    assert cli.main(['--fast','--left'])==17 and seen==[['--left']]


@pytest.mark.parametrize('args',[
    ['--headless'],['--offline'],['--state','missing'],['--headless','--destinations','R1','--seconds','1'],
    ['--budget','nan'],['--seconds','-1'],['--destinations','R1','R1'],
    ['--destinations','R1','R2','R1','R2'],['--destinations','not_a_destination']])
def test_bad_cli_does_not_acquire(monkeypatch,args):
    from drum_planner.experiment import app as experiment_app
    def forbidden(*a,**kw):raise AssertionError('Invalid invocation must not acquire anything')
    monkeypatch.setattr(experiment_app,'prepare',forbidden)
    assert app.main(args)==1


def test_wrong_arm_cli_rejected_before_acquisition(monkeypatch):
    from drum_planner.experiment import app as experiment_app
    def forbidden(*a,**kw):raise AssertionError('Wrong side must not acquire devices')
    monkeypatch.setattr(experiment_app,'prepare',forbidden)
    assert app.main(['--left','--destinations',str(ROOT/'destinations/right/destinationR1.json')])==1


def test_side_routing():
    assert app.resolve_destination('R1','right').parent.name=='right'
    assert app.resolve_destination('L1','left').parent.name=='left'
    with pytest.raises(ValueError):app.resolve_destination('R1','left')


def test_no_hardware_execution_flag():
    with pytest.raises(SystemExit):app.main(['--execute'])


def test_destination_snapshot_preserved_and_wrong_arm_rejected(tmp_path,monkeypatch):
    from drum_planner.experiment import destinations
    original=ROOT/'destinations/right/destinationR1.json';before=original.read_bytes()
    with pytest.raises(ValueError):destinations.freeze(original,'left',tmp_path)
    assert original.read_bytes()==before


def test_minimum_joint_curve_has_no_extra_travel():
    a=np.zeros(7);b=np.array([.1,-.2,.05,.07,-.3,.4,.2])
    source=planner.source_for([a,b],dict(start=a.tolist(),goal=b.tolist()))
    np.testing.assert_allclose(planner.travel(source)['joint_travel_rad'],abs(b-a),atol=1e-12,rtol=0)


def test_continuous_position_bounds_reject_curve():
    a=np.zeros(7);b=np.ones(7)*.3;source=planner.source_for([a,b],{})
    planner.continuous_joint_bounds(source,[-1]*7,[1]*7)
    with pytest.raises(ValueError,match='position limits'):
        planner.continuous_joint_bounds(source,[-1]*7,[.2]*7)


def test_axis_constraint_does_not_accept_reversed_stick(kin):
    q=(kin.lower+kin.upper)/2;goal=target(kin,q);goal['axis']=-goal['axis']
    assert not task_proof(kin,q,goal,'tip-axis')['valid']


def test_continuous_bounds_find_interior_overshoot():
    coefficients=np.zeros((7,3,1));coefficients[0,:,0]=[-1.2,1.2,0.]
    source=dict(timing=dict(coefficients=coefficients.tolist(),polynomial_breaks_unscaled_s=[[0.,1.]]*7))
    with pytest.raises(ValueError,match='joint 1'):
        planner.continuous_joint_bounds(source,[-.2]*7,[.2]*7)


def test_blocked_endpoint_does_not_reach_timing(tmp_path,monkeypatch):
    class Blocked:
        def check(self,q):return {'valid':[True,False],'details':{'reason':'collision'}}
    monkeypatch.setattr(planner,'timed',lambda *a:pytest.fail('Timing a blocked endpoint'))
    with pytest.raises(ValueError,match='Endpoint collision'):
        planner.route(Blocked(),{'start':[0.]*7,'goal':[.1]*7},tmp_path,lambda _:None)


def test_workspace_reference_does_not_get_new_clearance_fallback(tmp_path,monkeypatch):
    from drum_planner.experiment import fast
    from drum_planner.fast_mode import clearance
    class ClearEndpoints:
        def check(self,q):return {'valid':[True,True]}
    monkeypatch.setattr(fast,'proposal_stages',lambda *a:iter([]))
    monkeypatch.setattr(clearance,'candidates',lambda *a,**kw:pytest.fail('Reference changed algorithms'))
    with pytest.raises(ValueError,match='No checked continuous route'):
        planner.route(ClearEndpoints(),{'start':[0.]*7,'goal':[.1]*7},tmp_path,lambda _:None,reference=True)
