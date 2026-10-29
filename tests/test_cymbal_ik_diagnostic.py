"""Recorded failure regressions, no camera or motor connections."""
import numpy as np
import pytest
from drum_planner.common import DEFAULT_CALIBRATION
from drum_planner.model import generate
from cymbal_destination import solver,goal_limits,aim,output
from cymbal_destination import diagnostic as diagnostic_module
from cymbal_destination.config import VERSION,ANGLE_CONVENTION,AIM_CONVENTION
from cymbal_destination.diagnostic import explain


def fixture(tmp_path):
    start=np.array([.032798809796291906,-.0009590295262071891,.17473517967498253,
                    .003260700389105864,-.2482838630503256,-.08266834515907462,.49466742961776156])
    generate(tmp_path,DEFAULT_CALIBRATION,reference_right=start.tolist(),both_tools=True)
    kin=solver.Kinematics(tmp_path/'robot.urdf','right')
    p=np.array([.5456778764161799,-.3773627777464458,.48015632163673116])
    n=np.array([-.3342309786365628,.4649859972616179,.8198034369714877])
    u=np.array([.5102473816512889,-.6420582966017407,.5721964289322987])
    return kin,start,p,n,u


def test_failed_dt1_snapshot_reports_old_20_degree_conflict_without_accepting_it(tmp_path,monkeypatch):
    # Preserve the original failing policy as a regression, rather than editing
    # its expectation to make the new setting look successful.
    for module in (solver,aim,diagnostic_module):monkeypatch.setattr(module,'MAX_INWARD_YAW_DEG',20.)
    kin,start,p,n,u=fixture(tmp_path)
    lo,hi=goal_limits.bounds(kin.lower,kin.upper);before=(lo.copy(),hi.copy(),start.copy())
    result=explain(kin,start,p,n,u,solver.seeds(kin,start),joint_bounds=(lo,hi))
    diagnostic=result['nearest_inward_diagnostic'];proof=diagnostic['geometry']
    assert 21.1<proof['inward_yaw_deg']<21.3 and not solver.acceptable(proof)
    assert proof['tip_error_m']<1e-8 and proof['j7_plane_error_deg']<1e-6
    assert diagnostic['not_a_destination'] and not diagnostic['global_impossibility_proven']
    assert not diagnostic['collision_validated'] and 'q' not in diagnostic
    assert diagnostic['active_joint_bounds']==[
        dict(joint='J2',side='lower'),dict(joint='J6',side='lower'),dict(joint='J7',side='upper')]
    assert '+21.17' in result['summary'] and 'No target/limit changes' in result['summary']
    for value,original in zip((lo,hi,start),before):np.testing.assert_array_equal(value,original)


def test_approved_25_range_solves_same_tip_without_changing_reserves(tmp_path):
    kin,start,p,n,u=fixture(tmp_path);before=[x.copy() for x in (kin.lower,kin.upper,start,p,n,u)]
    class Oracle:
        def check(self,poses):return dict(valid=[True]*len(poses))  # IK test; real FCL is checked separately.
    rows,report=solver.candidates(kin,Oracle(),start,p,n,u,lambda _:None)
    assert rows and report['maximum_inward_yaw_deg']==25.
    for row in rows:
        assert solver.acceptable(row['geometry']) and goal_limits.valid(row['q'],kin.lower,kin.upper)
        assert row['geometry']['tip_error_m']<1e-5
        assert abs(row['geometry']['inward_yaw_deg'])<=25.+1e-6
        assert row['q'][1]>=np.radians(1)-1e-10
    for value,original in zip((kin.lower,kin.upper,start,p,n,u),before):np.testing.assert_array_equal(value,original)


def test_v7_recovery_is_readable_but_never_relabeled_v8():
    from drum_planner.joint_position_limits import policy
    old=dict(version='cymbal-prime-destination-v7',angle_convention=ANGLE_CONVENTION,
             aim_convention=AIM_CONVENTION,maximum_inward_yaw_deg=20.,
             goal_joint_reserve_policy=goal_limits.policy(),joint_position_policy=policy())
    output.current_convention(old,allow_legacy_joint_reserve=True)
    with pytest.raises(ValueError,match='obsolete'):output.current_convention(old)
    with pytest.raises(ValueError,match='obsolete'):
        output.current_convention(dict(old,maximum_inward_yaw_deg=25.),allow_legacy_joint_reserve=True)
    current=dict(old,version=VERSION,maximum_inward_yaw_deg=25.)
    output.current_convention(current)
    with pytest.raises(ValueError,match='obsolete'):
        output.current_convention(dict(current,joint_position_policy={}),allow_legacy_joint_reserve=True)
