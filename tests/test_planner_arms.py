"""Arm routing, original-frame FK/zone parity and left-only software playback.

All device access is mocked/blocked. Native tests use the real MoveIt/FCL oracle.
"""
import copy
import json
from pathlib import Path
import socket
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from drum_planner import common, model, planning, right_zone
from drum_planner.experiment import algorithms, config, worker
from drum_planner.trajectory_view import PlanDisplay
from safe_zone.geometry import Model, Zone, LEFT_TCP, MEMBERSHIP_BUFFER_M


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    real = socket.socket
    def guard(family=socket.AF_INET, *args, **kwargs):
        if family == socket.AF_CAN:
            raise AssertionError('No real CAN in arm-mode tests')
        return real(family, *args, **kwargs)
    monkeypatch.setattr(socket, 'socket', guard)
    from drum_planner import depth
    monkeypatch.setattr(depth, 'DepthCamera', lambda *a, **k: pytest.fail('No camera in arm-mode tests'))


@pytest.fixture(scope='module')
def left_request(tmp_path_factory):
    folder=tmp_path_factory.mktemp('left-experiment-model')
    state=common.read_state(config.saved_inputs('left')[0])
    req,manifest=planning.request(folder,state,dict(synthetic=True,obstacle_padding_m=.02,boxes=[]),moving_side='left')
    return req,manifest,state


def test_saved_left_start_is_measured_once_and_scene_is_the_same():
    state_path,scene=config.saved_inputs('left')
    old,old_scene=config.saved_inputs()
    state=common.read_state(state_path)
    assert state['measured_side']=='left' and state['motor_commands']==0
    assert state['right_rad']==common.read_state(old)['right_rad']
    assert scene==old_scene
    report=json.loads((state_path.parent/'capture_report.json').read_text())
    assert report['side']=='left' and report['bus']=='can1'
    assert not report['other_arm_queried'] and report['socket_closed']
    assert report['query_packets']>=8 and report['error'] is None
    assert report['motor_commands']==report['enable_commands']==report['disable_commands']==report['relax_commands']==0


def test_left_endpoint_uses_only_the_final_sample(tmp_path):
    original=json.loads(common.DEFAULT_LEFT_RECORDING.read_text())
    expected=original['samples'][-1]['positions_rad']
    original['samples'][:-1]=[{'not_a_path':'ignored, not a seed'}]
    p=tmp_path/'record.json';common.atomic_json(p,original)
    assert np.array_equal(common.endpoint(p,side='left'),expected)
    assert np.array_equal(common.endpoint(side='left'),expected)
    with pytest.raises(ValueError,match='Incompatible right'):
        common.endpoint(p)
    with pytest.raises(ValueError,match='Incompatible left'):
        common.endpoint(common.DEFAULT_RECORDING,side='left')


@pytest.mark.parametrize('side',['wrong','LEFT',None])
def test_bad_arm_rejected(side):
    with pytest.raises(ValueError,match='side'):
        common.arm_joints(side)


@pytest.mark.parametrize('flags,side',[([], 'right'),(['--right'],'right'),(['--left'],'left')])
def test_cli_routes_to_selected_side_without_devices(monkeypatch,tmp_path,flags,side):
    from drum_planner import cli
    from drum_planner.experiment import app
    calls=[]
    monkeypatch.setattr(app,'prepare',lambda *args,**kw:calls.append(kw))
    monkeypatch.setattr(app,'headless',lambda *a,**kw:dict(progress=dict(status='planned')))
    assert cli.main(['experiment',*flags,'--headless','--output',str(tmp_path/'out')])==0
    assert calls[0]['side']==side


def test_cli_arm_flags_are_mutually_exclusive():
    from drum_planner.cli import main
    with pytest.raises(SystemExit) as error:
        main(['experiment','--right','--left','--list-methods'])
    assert error.value.code==2


def test_cli_legacy_start_capture_requests_complete_fresh_snapshot(monkeypatch,tmp_path):
    from drum_planner import cli
    from drum_planner.experiment import app, start_pose
    events=[]
    def capture(*args):pytest.fail('Legacy selected-only capture must not mix a new pose with an old scene')
    def prepare(folder,state,scene,cal,**kwargs):
        events.append(('prepare',kwargs['side'],state,scene,kwargs['fresh']))
    monkeypatch.setattr(start_pose,'capture',capture)
    monkeypatch.setattr(app,'prepare',prepare)
    monkeypatch.setattr(app,'headless',lambda *a:dict(progress=dict(status='planned')))
    assert cli.main(['experiment','--left','--capture-start','--headless','--output',str(tmp_path/'session')])==0
    assert events==[('prepare','left',None,None,True)]


def test_left_request_model_groups_tools_and_goal(left_request):
    req,manifest,state=left_request
    assert req['moving_side']=='left' and req['start']==state['left_rad']
    assert req['right']==state['right_rad']
    assert req['goal']==common.endpoint(side='left').tolist()
    assert 'right_zone' not in req and req['left_zone']['tcp']==LEFT_TCP
    root=ET.parse(req['srdf']).getroot()
    assert [j.get('name') for j in root.find("group[@name='left_arm']")]==common.LEFT_JOINTS
    assert [j.get('name') for j in root.find("group[@name='right_arm']")]==common.JOINTS
    urdf=ET.parse(req['urdf']).getroot()
    for side in ('left','right'):
        assert urdf.find(f"link[@name='{side}_drumstick']/collision/geometry/cylinder") is not None
        assert urdf.find(f"joint[@name='{side}_drumstick_fixed']/parent").get('link')==f'openarmx_{side}_hand'
    assert manifest['left_kinematic_offsets_applied'] is False
    # Right-only calibration offsets are NOT mirrored onto the left arm.
    original=ET.parse(common.ROOT/'model/openarmx.urdf').getroot()
    for name in common.LEFT_JOINTS:
        for tag in ('origin','axis','limit'):
            assert urdf.find(f"joint[@name='{name}']/{tag}").attrib==original.find(f"joint[@name='{name}']/{tag}").attrib


def test_right_default_model_bytes_and_default_goal_are_unchanged(tmp_path):
    old=config.saved_inputs()[0].parent.parent/'plan/result/model'
    # Source old manifests have the same calibrated geometry and tools.
    new=model.generate(tmp_path/'right')
    assert common.digest(tmp_path/'right/robot.urdf')==common.digest(old/'robot.urdf')
    assert common.digest(tmp_path/'right/robot.srdf')==common.digest(old/'robot.srdf')
    state=common.read_state(config.saved_inputs()[0])
    req,_=planning.request(tmp_path/'default',state,dict(synthetic=True,boxes=[],obstacle_padding_m=.02))
    assert req.get('moving_side','right')=='right' and req['start']==state['right_rad']
    assert req['goal']==common.endpoint().tolist() and req['right_zone']==right_zone.constraint()
    assert new['joint_limit_intervals_widened'] is False


def test_left_zone_cannot_load_right_envelope():
    with pytest.raises(ValueError,match='Incompatible'):
        right_zone.constraint(right_zone.ZONE,side='left')


def test_left_native_fk_and_zone_parity_and_collision(left_request,tmp_path):
    from drum_planner.experiment.runtime import Oracle
    req,_,_=left_request;common.atomic_json(tmp_path/'request.json',req)
    oracle=Oracle(tmp_path/'request.json',tmp_path)
    try:
        assert oracle.meta['moving_side']=='left'
        assert oracle.meta['joint_limit_check']['moving_left_limits_unchanged']
        assert oracle.meta['left_zone_constraint']==right_zone.metadata(req['left_zone'])
        rng=np.random.default_rng(17)
        q=np.vstack([req['start'],req['goal'],rng.uniform(oracle.lower,oracle.upper,(80,7))])
        rows=oracle.call('zone',q=q.tolist())['rows']
        nominal=Model(right_zone.REFERENCE_MODEL)
        zone=Zone.load(req['left_zone']['zone_file'],nominal.digest,LEFT_TCP)
        for angles,row in zip(q,rows,strict=True):
            point=nominal.transforms(dict(zip(common.LEFT_JOINTS,angles)))[LEFT_TCP][:3,3]
            assert np.allclose(row['point_xyz_m'],point,atol=2e-14,rtol=0)
            assert row['valid']==zone.contains(point,MEMBERSHIP_BUFFER_M)
        fk=oracle.call('fk',q=q[:2].tolist())
        generated=Model(req['urdf'])
        static=dict(zip(common.JOINTS,req['right']))
        for i,angles in enumerate(q[:2]):
            frames=generated.transforms(dict(static,**dict(zip(common.LEFT_JOINTS,angles))))
            T=frames['left_drumstick']
            assert np.allclose(fk['wrist_xyz_m'][i],frames['openarmx_left_hand'][:3,3])
            assert np.allclose(fk['tip_xyz_m'][i],T[:3,3]+T[:3,2]*model.LEFT_STICK_LENGTH/2)
        assert all(oracle.check(q[:2])['valid'])
    finally:oracle.close()


def test_native_collision_checks_the_left_stick_not_just_right(left_request,tmp_path):
    req,_,_=left_request;req=copy.deepcopy(req)
    m=Model(req['urdf']);frames=m.transforms(dict(zip(common.JOINTS+common.LEFT_JOINTS,req['right']+req['start'])))
    T=frames['left_drumstick'];point=T[:3,3]+T[:3,2]*model.LEFT_STICK_LENGTH/2
    req['scene']['boxes']=[dict(id='left_tip_obstacle',xyz=point.tolist(),size=[.03]*3)]
    req['check_states']=[req['start']]
    result=planning.backend(req,tmp_path,'left_collision')
    assert result['status']=='checked'
    row=result['checks'][0];assert row['valid'] is False and row['reason']=='collision'
    assert any('left_drumstick' in pair and 'left_tip_obstacle' in pair for pair in row['contacts'])


def test_moving_left_bounds_are_not_given_static_tolerance(left_request,tmp_path):
    req,_,_=left_request;req=copy.deepcopy(req)
    req['start'][3]=-.0001
    result=planning.backend(req,tmp_path,'left_bounds')
    assert result['status']=='blocked_start'
    row=next(v for v in result['details']['violations'] if v['joint']=='openarmx_left_joint4')
    assert row['measurement_tolerance_rad']==0 and not result['details']['angles_clamped']


def test_left_cycle_and_preview_leave_right_completely_fixed(left_request,tmp_path):
    req,manifest,state=left_request
    context=dict(calibration=str(common.DEFAULT_CALIBRATION),state=str(config.saved_inputs('left')[0]),scene='synthetic',moving_side='left')
    result=worker.base_result(context,req,manifest,state)
    points,_,_=planning.smooth_samples([req['start'],req['goal']])
    result.update(status='planned',outward=points,cycle=planning.cycle(points),hold_at_goal_s=2.)
    common.atomic_json(tmp_path/'backend_request.json',req);common.atomic_json(tmp_path/'plan.json',result)
    plan=PlanDisplay(tmp_path/'plan.json')
    start=plan.frames(0.)
    for t in np.linspace(0,plan.duration,25):
        frames=plan.frames(t)
        for name in frames:
            if 'right' in name and not name.startswith('openarmx_left_'):
                assert np.array_equal(frames[name],start[name]),name
    assert not np.allclose(plan.frames(points[-1]['time_s'])['left_drumstick'],start['left_drumstick'])
    assert np.array_equal(plan.positions(points[-1]['time_s']),common.endpoint(side='left'))
    assert np.array_equal(plan.positions(plan.duration),state['left_rad'])
    from drum_planner.workflow import simulate
    report=simulate(result)
    assert report['moving_side']=='left' and report['right_arm']=='static, no commands'
    assert result['experiment_only'] and not result['hardware_execution_available']


def test_arm_identity_separates_candidate_caches(left_request):
    req,_,_=left_request
    context=dict(scene_sha256='scene',state_sha256='state',calibration=str(common.DEFAULT_CALIBRATION))
    left=algorithms.pool_key(context,req,config.DEFAULT,60)
    # Hold every other input identical to prove side itself is part of the key.
    other=copy.deepcopy(req);other['moving_side']='right';other['right_zone']=other.pop('left_zone')
    assert left!=algorithms.pool_key(context,other,config.DEFAULT,60)


def test_one_shot_capture_queries_only_selected_bus_and_closes(tmp_path,monkeypatch):
    from drum_planner.experiment import start_pose
    from safe_zone import encoder
    from safe_zone.gripper_feedback import EncoderState
    opened=[];frames=[]
    class Socket:
        closed=False
        def send(self,frame):frames.append(frame);return len(frame)
        def close(self):self.closed=True
    class Observer:
        def __init__(self,side,bus):
            opened.append((side,bus));self.socket=Socket()
        def sample(self):
            for motor in range(1,9):
                self.socket.send(encoder.request_frame(motor))
                now=start_pose.time.monotonic()
                self.socket.feedback[motor]=dict(request_monotonic=now,received_monotonic=now,mode=0,fault_bits=0)
            joints={j:.1 for j in common.LEFT_JOINTS};joints['openarmx_left_finger_joint1']=0.
            return EncoderState(joints,motor8_feedback={'left':{'raw_rad':.2}})
        def close(self):self.socket.close();self.socket=None
    monkeypatch.setattr(encoder,'SingleArmObserver',Observer)
    monkeypatch.setattr(start_pose.time,'sleep',lambda _:None)
    p=start_pose.capture(tmp_path/'captured','left',config.saved_inputs()[0])
    assert opened==[('left','can1')]
    assert len(frames)==160 and set(frames)=={encoder.request_frame(m) for m in range(1,9)}
    state=common.read_state(p)
    assert state['left_rad']==[.1]*7
    assert state['right_rad']==common.read_state(config.saved_inputs()[0])['right_rad']
    report=json.loads((p.parent/'capture_report.json').read_text())
    assert report['socket_closed'] and report['motor_commands']==report['relax_commands']==0
