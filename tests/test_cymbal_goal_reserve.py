"""Endpoint-only reserve regressions. All inputs are saved/software; no devices."""
import copy,json
from pathlib import Path
import numpy as np
import pytest
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,digest,atomic_json
from drum_planner.model import generate
from cymbal_destination import solver,goal_limits,output


def test_fault_capture_candidates_respect_per_joint_goal_reserves(tmp_path):
    generate(tmp_path,DEFAULT_CALIBRATION)
    kin=solver.Kinematics(tmp_path/'robot.urdf','right')
    # Exact failed hardware capture, not a hypothetical reachable green dot.
    start=np.array([-.0036443121995883843,.011316548409247673,.892089265278095,
        -.019756008239872003,-.8401968866255153,.1344559395742735,.6009279011215387])
    old=np.array([-.11457418765082465,-.11948622196409846,.09273277708051036,
        .7050458435688017,.20664920481650426,-.5348297078972614,1.4])
    point=np.array([.5408726891409983,-.378703976028645,.48438769130806975])
    normal=np.array([-.3385965855601337,.45275237210930486,.824844010584072])
    inward=np.array([.6300351368902299,-.49046495368940635,.4850874864203884])-np.array(
        [.5666737489606805,-.413203706783374,.42153457770156344])
    inward-=normal*(inward@normal);inward/=np.linalg.norm(inward)
    assert solver.acceptable(solver.geometry(kin,old,point,normal,inward))
    before=start.copy();lower=kin.lower.copy();upper=kin.upper.copy()
    class Oracle:
        def check(self,poses):return dict(valid=[True]*len(poses))  # Geometry unit test only.
    rows,report=solver.candidates(kin,Oracle(),start,point,normal,inward,lambda _:None)
    assert rows,report
    for row in rows:
        assert np.all(row['q']-kin.lower>=np.radians(goal_limits.policy()['lower_deg'])-1e-10)
        assert np.all(kin.upper-row['q']>=np.radians(goal_limits.policy()['upper_deg'])-1e-10)
        assert solver.acceptable(row['geometry'])
    np.testing.assert_array_equal(start,before)
    np.testing.assert_array_equal(kin.lower,lower);np.testing.assert_array_equal(kin.upper,upper)
    # Actual starting J4 is allowed near the model boundary. Only goals change.
    assert start[3]-kin.lower[3]<np.radians(3.)


@pytest.mark.parametrize('joint',range(7))
@pytest.mark.parametrize('upper_side',[False,True])
def test_each_boundary_and_interior_reserve(joint,upper_side):
    lower=np.full(7,-1.4);upper=-lower;lower[1]=0.;lo,hi=goal_limits.bounds(lower,upper)
    q=(lo+hi)/2;q[joint]=(hi if upper_side else lo)[joint]
    expected=goal_limits.policy()['upper_deg' if upper_side else 'lower_deg'][joint]
    assert goal_limits.require(q,lower,upper)['minimum_distance_deg']==pytest.approx(expected)
    q[joint]+=.001 if upper_side else -.001
    before=q.copy()
    with pytest.raises(ValueError,match='reserve'):goal_limits.require(q,lower,upper)
    np.testing.assert_array_equal(q,before)
    q[joint]=(upper if upper_side else lower)[joint]
    assert not goal_limits.valid(q,lower,upper)


@pytest.mark.parametrize('q',[[0.]*6,[0.]*8,[float('nan')]*7,[float('inf')]*7])
def test_nonfinite_or_malformed_goal_rejected(q):
    with pytest.raises(ValueError,match='reserve'):goal_limits.require(q,[-1.]*7,[1.]*7)


def test_only_optimizer_seed_is_projected_not_measured_start(tmp_path):
    generate(tmp_path,DEFAULT_CALIBRATION);kin=solver.Kinematics(tmp_path/'robot.urdf','right')
    start=kin.lower.copy();before=start.copy();seeds=solver.seeds(kin,start)
    assert all(goal_limits.valid(q,kin.lower,kin.upper) for q in seeds)
    np.testing.assert_array_equal(start,before)
    assert not goal_limits.valid(start,kin.lower,kin.upper)


def test_boundary_pose_cannot_be_exported_even_when_angles_are_perfect(tmp_path):
    generate(tmp_path,DEFAULT_CALIBRATION);kin=solver.Kinematics(tmp_path/'robot.urdf','right')
    q=np.array([-.4,.4,.2,1.,.006,-.022,1.4]);f=kin.fk(q,jac=True)
    n=f['Jtip'][:,6];n/=np.linalg.norm(n);u=f['axis']-n*(f['axis']@n);u/=np.linalg.norm(u)
    geometry=solver.geometry(kin,q,f['tip'],n,u);assert solver.acceptable(geometry)
    before=q.copy()
    with pytest.raises(ValueError,match='reserve'):
        output.destination(q,geometry,f['tip'],n,DEFAULT_CALIBRATION,'unit-test-scene',joint_bounds=(kin.lower,kin.upper))
    np.testing.assert_array_equal(q,before)
    assert not (tmp_path/'destination.json').exists()


def test_proof_bound_to_goal_model_and_policy():
    q=np.zeros(7);q[1]=.1;lo=np.full(7,-1.4);hi=-lo;lo[1]=0.;proof=goal_limits.require(q,lo,hi)
    assert goal_limits.verify(q,proof,(lo,hi))==proof
    with pytest.raises(ValueError,match='changed'):goal_limits.verify(q+.1,proof,(lo,hi))
    with pytest.raises(ValueError,match='model bounds'):goal_limits.verify(q,proof,(lo+.01,hi))
    bad=copy.deepcopy(proof);bad['policy']['minimum_deg']=0.
    with pytest.raises(ValueError,match='obsolete'):goal_limits.verify(q,bad)
    bad=copy.deepcopy(proof);bad['minimum_distance_deg']=90.
    with pytest.raises(ValueError,match='changed'):goal_limits.verify(q,bad)


def test_legacy_policy_only_for_already_calculated_hardware_missions():
    from cymbal_destination.config import ANGLE_CONVENTION,AIM_CONVENTION,MAX_INWARD_YAW_DEG,VERSION
    old=dict(version='cymbal-prime-destination-v4',angle_convention=ANGLE_CONVENTION,
             aim_convention=AIM_CONVENTION,maximum_inward_yaw_deg=20.)
    output.current_convention(old,allow_legacy_joint_reserve=True)
    with pytest.raises(ValueError,match='obsolete'):output.current_convention(old)
    with pytest.raises(ValueError,match='obsolete'):
        output.current_convention(dict(old,version=VERSION),allow_legacy_joint_reserve=True)


def test_hardware_uses_real_calibrated_bounds_not_claimed_reserve(tmp_path):
    from drum_planner.baseline import targets
    from drum_planner.experiment import rgbd
    path=ROOT/'diagnostics/cymbal_goal_reserve_20261027/hardware_fault/hardware/plan.json'
    if not path.exists():pytest.skip('Native saved-input study fixture unavailable')
    plan=json.loads(path.read_text());world=rgbd.scene(plan['provenance']['scene'],DEFAULT_CALIBRATION)
    targets.validate(plan,world)
    old=plan['destination_bindings'][0];obj=json.loads(Path(old['goal_recording']).read_text())
    q=np.asarray(obj['samples'][0]['positions_rad']);proof=obj['goal_joint_reserve']
    # A valid-looking proof with widened claimed bounds cannot enter hardware.
    obj['goal_joint_reserve']=goal_limits.require(q,np.asarray(proof['model_lower_rad'])-.1,proof['model_upper_rad'],legacy=True)
    file=tmp_path/'tampered.json';atomic_json(file,obj)
    old.update(goal_recording=str(file));plan['dt1_generation']['resolved_sha256'][0]=digest(file)
    with pytest.raises(ValueError,match='model bounds'):targets.validate(plan,world)


def test_manual_destinations_do_not_acquire_new_goal_reserve_restrictions():
    from drum_planner.baseline.targets import validate
    # Legacy/manual targets and left arm remain on their original validation.
    validate(dict(moving_side='left',destination_bindings=[dict(destination_kind='saved')]),{})
    validate(dict(moving_side='right',destination_bindings=[dict(destination_kind='saved')]),{})


def test_synthetic_observed_j7_excursion_fits_reserve_not_a_hardware_guarantee():
    # Largest retained post-fault J7 sample was 1.4072799267566953 rad, with
    # a commanded 1.4 rad boundary goal. This arithmetic does not predict or
    # certify a new physical run; it isolates the original zero-reserve issue.
    excursion=1.4072799267566953-1.4
    lower=np.full(7,-1.4);upper=-lower;_,inner_upper=goal_limits.bounds(lower,upper)
    assert 1.4+excursion>upper[6]
    assert inner_upper[6]+excursion<upper[6]


def test_export_rechecks_reserve_not_just_the_file_hash(tmp_path):
    from cymbal_destination.config import VERSION,ANGLE_CONVENTION,AIM_CONVENTION,MAX_INWARD_YAW_DEG
    from drum_planner.joint_position_limits import policy
    q=np.zeros(7);q[1]=.1;lower=np.full(7,-1.4);lower[1]=0.
    proof=goal_limits.require(q,lower,[1.4]*7)
    obj=dict(version=VERSION,angle_convention=ANGLE_CONVENTION,aim_convention=AIM_CONVENTION,
        joint_position_policy=policy(),
        maximum_inward_yaw_deg=MAX_INWARD_YAW_DEG,goal_joint_reserve_policy=goal_limits.policy(),
        goal_joint_reserve=proof,capture_kind='destination',sample_count=1,duration_s=0,motion_authorized=False,
        samples=[dict(time_s=0.,positions_rad=[0.,0.,0.,0.,0.,0.,1.4])])
    source=tmp_path/'source.json';atomic_json(source,obj)
    with pytest.raises(ValueError,match='reserve'):output.export(source,tmp_path/'export.json',digest(source))
    assert not (tmp_path/'export.json').exists()


@pytest.mark.parametrize('bad_winner',[True,False])
def test_worker_does_not_save_boundary_goal_or_false_ready(tmp_path,monkeypatch,bad_winner):
    from cymbal_destination import worker
    generate(tmp_path/'reference_model',DEFAULT_CALIBRATION)
    kin=solver.Kinematics(tmp_path/'reference_model/robot.urdf','right')
    q=np.array([-.4,.4,.2,1.,.006,-.022,1.4]);f=kin.fk(q,jac=True)
    n=f['Jtip'][:,6];n/=np.linalg.norm(n);u=f['axis']-n*(f['axis']@n);u/=np.linalg.norm(u)
    geometry=solver.geometry(kin,q,f['tip'],n,u)
    target=dict(target=dict(world_m=(f['tip']-.0762*n).tolist()),
                flat_disc=dict(center_world_m=(f['tip']-.0762*n+.1*u).tolist()))
    state=dict(right_rad=q.tolist(),left_rad=[0.]*7,left_gripper_opening_m=0.)
    monkeypatch.setattr(worker.inputs,'load',lambda *a:(state,dict(source_scene_sha256='unit-test'),target,f['tip'],n))
    class FakeOracle:
        lower=kin.lower;upper=kin.upper
        def __init__(self,*a):pass
        def close(self):pass
    monkeypatch.setattr(worker,'Oracle',FakeOracle)
    winner=dict(q=q.tolist(),geometry=geometry,approach=dict(valid=True)) if bad_winner else None
    monkeypatch.setattr(worker.solver,'solve',lambda *a:(winner,dict(reason='No reserved goal found')))
    run=tmp_path/'run';run.mkdir()
    if bad_winner:
        with pytest.raises(ValueError,match='reserve'):worker.run(run)
    else:assert worker.run(run)['status']=='no_feasible_destination'
    result=json.loads((run/'result.json').read_text())
    assert result['status']!='ready' and not (run/'destination.json').exists()
