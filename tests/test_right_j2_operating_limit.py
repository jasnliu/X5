"""Encoder-coordinate J2 bounds and stale-Start regressions; no devices."""
import copy
import json
import socket
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from drum_planner import model,joint_position_limits as bounds
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,calibration,digest,atomic_json,endpoint
from drum_planner.baseline import tasks
from drum_planner.fast_mode.kinematics import Kinematics
from drum_planner.experiment.workspace_recovery import position_bounds
from cymbal_destination import output,goal_limits
from cymbal_destination.config import VERSION,ANGLE_CONVENTION,AIM_CONVENTION,MAX_INWARD_YAW_DEG


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    def denied(*a,**kw):raise AssertionError('This test must not open devices or sockets')
    monkeypatch.setattr(socket,'socket',denied)


@pytest.mark.parametrize('side',['right','left'])
def test_only_selected_right_j2_lower_changes_after_coordinate_conversion(tmp_path,side):
    original=digest(ROOT/'model/openarmx.urdf');cal=calibration()
    manifest=model.generate(tmp_path,moving_side=side)
    raw=ET.parse(ROOT/'model/openarmx.urdf').getroot();actual=ET.parse(tmp_path/'robot.urdf').getroot()
    for arm in ('right','left'):
        for i in range(1,8):
            name=f'openarmx_{arm}_joint{i}';a=raw.find(f"joint[@name='{name}']/limit")
            b=actual.find(f"joint[@name='{name}']/limit")
            offset=cal['kinematic_correction']['offsets_rad'].get(name,0.)
            for key in ('lower','upper','velocity','effort'):
                expected=float(a.get(key))-(offset if key in ('lower','upper') else 0.)
                if side=='right' and name==bounds.RIGHT_J2 and key=='lower':expected=max(expected,0.)
                assert float(b.get(key))==pytest.approx(expected,abs=1e-14)
    assert digest(ROOT/'model/openarmx.urdf')==original
    if side=='right':
        bounds.require_current_model(manifest,tmp_path/'robot.urdf',side)
        assert manifest['joint_limit_mapping'][bounds.RIGHT_J2]['saved_coordinate_lower_rad']==0.
        assert manifest['joint_limit_mapping'][bounds.RIGHT_J2]['nominal_saved_coordinate_lower_rad']==pytest.approx(-.11948622196409846)
    else:assert 'joint_position_policy' not in manifest


def test_zero_does_not_receive_camera_offset_and_j2_is_not_locked(tmp_path):
    manifest=model.generate(tmp_path);kin=Kinematics(tmp_path/'robot.urdf','right')
    assert kin.lower[1]==0.
    assert np.degrees(kin.upper[1])==pytest.approx(187.9595941159737)
    assert manifest['joint_position_policy']['observed_approximate_stop_deg']==-1.
    assert not manifest['joint_position_policy']['independently_hardware_verified']
    assert np.degrees(goal_limits.bounds(kin.lower,kin.upper)[0][1])==pytest.approx(1.)
    for name in ('destinationR1','destinationR2'):
        q=endpoint(ROOT/f'destinations/right/{name}.json')
        assert np.all(q>=kin.lower) and np.all(q<=kin.upper)


@pytest.mark.parametrize('bad',['missing_policy','modified_policy','old_urdf','wrong_mapping','nan'])
def test_obsolete_or_tampered_model_cannot_prepare_new_motion(tmp_path,bad):
    manifest=model.generate(tmp_path);path=tmp_path/'robot.urdf'
    if bad=='missing_policy':manifest.pop('joint_position_policy')
    elif bad=='modified_policy':manifest['joint_position_policy']['lower_deg']=-1.
    elif bad=='wrong_mapping':manifest['joint_limit_mapping'][bounds.RIGHT_J2]['saved_coordinate_lower_rad']=-.1
    else:
        tree=ET.parse(path);tree.getroot().find(f"joint[@name='{bounds.RIGHT_J2}']/limit").set('lower','nan' if bad=='nan' else '-.1')
        tree.write(path)
    with pytest.raises(ValueError,match='J2 operating limit'):bounds.require_current_model(manifest,path,'right')
    bounds.require_current_model(manifest,path,'left')  # No new left-side gate.


def test_old_prepared_start_rejected_before_files_devices_or_motion(tmp_path):
    p=tmp_path/'prepared.json'
    atomic_json(p,dict(schema='x5-baseline-prepared-v2',motion_authorized=False,
                      request=dict(moving_side='right',urdf='not-opened'),object=dict(model={})))
    with pytest.raises(ValueError,match='Calculate again'):tasks.check_packet(p)


def test_old_preparation_rejected_but_recovery_loader_not_replaced(tmp_path,monkeypatch):
    old=dict(model={});req=dict(moving_side='right',urdf='not-opened')
    monkeypatch.setattr(tasks,'load_checked',lambda *a:(old,req,None))
    with pytest.raises(ValueError,match='Calculate again'):
        tasks.prepare(tmp_path/'out','old-plan','not-opened',DEFAULT_CALIBRATION)
    # The actual recovery entry still uses load_checked, NOT the new Start gate.
    # If somebody moves the policy gate into the common loader, this fails.
    import inspect
    assert 'require_current_model' not in inspect.getsource(tasks.check_return)


def test_full_curve_extrema_reject_dip_even_when_endpoints_legal(tmp_path):
    from scipy.interpolate import PPoly
    model.generate(tmp_path);kin=Kinematics(tmp_path/'robot.urdf','right')
    c=np.zeros((3,1,7));c[-1,0]=(kin.lower+kin.upper)/2
    # q2(s)=0.2*s^2-0.2*s+0.01: endpoints positive, interior -0.04 rad.
    c[:,0,1]=[.2,-.2,.01];p=PPoly(c,[0.,1.])
    assert p(0)[1]>0 and p(1)[1]>0
    assert not position_bounds(p,kin.lower,kin.upper)
    c[-1,0,1]=.06
    assert position_bounds(PPoly(c,[0.,1.]),kin.lower,kin.upper)


def test_captured_start_below_zero_is_never_silently_projected(tmp_path):
    from cymbal_destination import solver
    model.generate(tmp_path);kin=Kinematics(tmp_path/'robot.urdf','right')
    q=(kin.lower+kin.upper)/2;q[1]=np.radians(-.1);before=q.copy()
    req=dict(urdf=str(tmp_path/'robot.urdf'),start=q.tolist())
    with pytest.raises(ValueError,match='not clipped'):
        solver.solve(None,req,[.5,-.3,.5],[0.,0.,1.],[1.,0.,0.])
    np.testing.assert_array_equal(q,before);np.testing.assert_array_equal(req['start'],before)


def test_v5_remains_readable_only_for_existing_mission_recovery():
    old=dict(version='cymbal-prime-destination-v5',angle_convention=ANGLE_CONVENTION,
        aim_convention=AIM_CONVENTION,maximum_inward_yaw_deg=20.,
        goal_joint_reserve_policy=goal_limits.legacy_policy())
    output.current_convention(old,allow_legacy_joint_reserve=True)
    with pytest.raises(ValueError,match='obsolete'):output.current_convention(old)
    current=dict(old,version=VERSION,maximum_inward_yaw_deg=MAX_INWARD_YAW_DEG,
        joint_position_policy=bounds.policy(),goal_joint_reserve_policy=goal_limits.policy())
    output.current_convention(current)
    current['goal_joint_reserve']=dict(model_lower_rad=[-1.4]*7)
    with pytest.raises(ValueError,match='obsolete right J2'):output.current_convention(current)


def test_dt1_job_identity_binds_the_operating_position_policy(monkeypatch):
    from drum_planner.experiment.dt1_goal import identity
    context=dict(scene_sha256='scene',state_sha256='state',calibration=str(DEFAULT_CALIBRATION),moving_side='right')
    before=identity(context,['dt1']);assert before['joint_position_policy']==bounds.policy()
    monkeypatch.setattr(bounds,'RIGHT_J2_LOWER_DEG',.5)
    assert identity(context,['dt1'])!=before
