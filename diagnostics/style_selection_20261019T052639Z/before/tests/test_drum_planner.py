"""No live devices: pure math, synthetic depth, real MoveIt/FCL/OMPL backend."""
import copy
import json
import socket
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from drum_planner import common,model,depth,planning,workflow
from camera_calibration.kinematics import HandKinematics, correction_offsets
from safe_zone.geometry import Model


@pytest.fixture(autouse=True)
def no_can_or_camera(monkeypatch):
    real=socket.socket
    def guarded(family=socket.AF_INET,*args,**kwargs):
        if family==socket.AF_CAN:raise AssertionError('Tests must not open CAN')
        return real(family,*args,**kwargs)
    monkeypatch.setattr(socket,'socket',guarded)
    def denied(*args,**kwargs):raise AssertionError('Tests must not open the live camera')
    monkeypatch.setattr(depth,'DepthCamera',denied)


@pytest.fixture(scope='module')
def generated(tmp_path_factory):
    folder=tmp_path_factory.mktemp('drum_model')
    info=model.generate(folder)
    return folder,info


@pytest.fixture
def start():
    return common.read_state(common.ROOT/'drum_planner/config/example_start.json')


@pytest.fixture
def req(tmp_path,start,generated):
    folder,info=generated
    return dict(schema='x5-moveit-request-v1',execute=False,urdf=str(folder/'robot.urdf'),
                srdf=str(folder/'robot.srdf'),start=start['right_rad'],left=start['left_rad'],
                left_opening_m=start['left_gripper_opening_m'],goal=common.endpoint().tolist(),
                scene=dict(synthetic=True,obstacle_padding_m=.02,boxes=[]),
                collision_policy=planning.COLLISION_POLICY,
                planning_seconds=8.,seed=19,occupied_only_preview=False)


def test_corrected_urdf_exactly_matches_calibration_fk(generated):
    path,info=generated
    m=Model(path/'robot.urdf');old=HandKinematics(common.ROOT/'model/openarmx.urdf')
    delta=correction_offsets(common.calibration()['kinematic_correction'])
    rng=np.random.default_rng(14)
    for q in rng.uniform(-.8,.8,(30,7)):
        T=m.transforms(dict(zip(common.JOINTS,q)))['openarmx_right_hand']
        assert np.allclose(T,old.fk((q+delta)[None])[0],atol=1e-12)
    assert not info['tool_geometry_physically_verified']
    assert info['joint_coordinates'].startswith('encoder_to_joint')


def test_calibrated_position_limits_use_same_coordinates_as_geometry(generated):
    path,info=generated
    source=ET.parse(common.ROOT/'model/openarmx.urdf').getroot()
    output=ET.parse(path/'robot.urdf').getroot()
    offsets=common.calibration()['kinematic_correction']['offsets_rad']
    assert not info['joint_limit_intervals_widened']
    assert not info['physical_joint_stops_independently_verified']
    for name in common.JOINTS+common.LEFT_JOINTS:
        a=source.find(f"joint[@name='{name}']/limit")
        b=output.find(f"joint[@name='{name}']/limit")
        offset=offsets.get(name,0.)
        lo,hi=float(a.get('lower')),float(a.get('upper'))
        mapped_lo,mapped_hi=float(b.get('lower')),float(b.get('upper'))
        assert mapped_lo==pytest.approx(lo-offset,abs=1e-15)
        assert mapped_hi==pytest.approx(hi-offset,abs=1e-15)
        assert mapped_hi-mapped_lo==pytest.approx(hi-lo,abs=1e-15)
        assert b.get('velocity')==a.get('velocity') and b.get('effort')==a.get('effort')
        if name in offsets:
            row=info['joint_limit_mapping'][name]
            assert row['saved_coordinate_lower_rad']==mapped_lo
            assert row['saved_coordinate_upper_rad']==mapped_hi
        else:
            assert b.attrib==a.attrib  # uncalibrated left limits unchanged


def test_reported_negative_right_elbow_is_valid_calibrated_pose(req,tmp_path):
    # Exact measured state in the user's failure. Nothing is clipped to zero.
    req['start']=[.0036443121995883843,.013234607461662051,.4532373540856032,
                  -.019756008239872003,-.24406413313501255,.1785712977798113,.5587306019684135]
    req['left']=[-.009014877546348998,.005562371252002762,-.00019180590524214836,
                 -.00019180590524214836,-.8732922865644301,.033182421606774426,-.07844861524376334]
    req['check_states']=[req['start'],req['goal']]
    checked=planning.backend(req,tmp_path,'negative_elbow')
    assert checked['status']=='checked'
    assert checked['joint_limit_check']['valid'] and not checked['joint_limit_check']['angles_clamped']
    assert all(row['valid'] for row in checked['checks']),checked
    assert req['start'][3]==-.019756008239872003


@pytest.mark.parametrize('edge',['lower','upper'])
def test_translated_right_limits_still_reject_true_model_overtravel(req,tmp_path,edge):
    limit=ET.parse(req['urdf']).getroot().find("joint[@name='openarmx_right_joint4']/limit")
    req['start'][3]=float(limit.get(edge))+(-1e-6 if edge=='lower' else 1e-6)
    checked=planning.backend(req,tmp_path,'true_overtravel_'+edge)
    assert checked['status']=='blocked_start'
    assert checked['details']['reason']=='joint_limits'
    assert checked['details']['violations'][0]['joint']=='openarmx_right_joint4'
    assert checked['details']['violations'][0]['measurement_tolerance_rad']==0.


def test_custom_halves_form_same_bore_and_stick_has_full_length(generated):
    path,info=generated
    m=Model(path/'robot.urdf');frames=m.transforms({})
    hand=frames['openarmx_right_hand']
    for a,cx in zip(info['gripper_assemblies'],[-14.75,14.75]):
        bore=frames[a['link']]@np.array([cx*.001,-.02335,.130,1])
        expected=hand@np.array([0,0,model.MOUNT_Z+.130,1])
        assert np.allclose(bore,expected,atol=1e-10)
    stick=frames['right_drumstick'];tip=stick@np.array([0,0,info['stick_length_from_mount_m']/2,1])
    assert np.allclose(tip,hand@np.array([0,0,model.MOUNT_Z+.130+11*.0254,1]),atol=1e-12)
    assert info['stick_radius_m']==pytest.approx(.0074)
    root=ET.parse(path/'robot.urdf').getroot()
    assert root.find('ros2_control') is None
    for name in ('openarmx_right_finger_joint1','openarmx_right_finger_joint2'):
        j=root.find(f"joint[@name='{name}']");assert j.get('type')=='fixed' and j.find('limit') is None


def test_only_structural_collision_exemptions(generated):
    root=ET.parse(generated[0]/'robot.srdf').getroot()
    for pair in root.findall('disable_collisions'):
        assert pair.get('reason')=='Adjacent or rigid assembly'
        if 'right_drumstick' in (pair.get('link1'),pair.get('link2')):
            assert not any(pair.get(k)=='openarmx_right_link3' for k in ('link1','link2'))


def test_exact_record3_endpoint_not_its_path():
    d=json.loads(common.DEFAULT_RECORDING.read_text())
    assert np.array_equal(common.endpoint(),d['samples'][-1]['positions_rad'])


@pytest.mark.parametrize('bad',['left','gripper','nan','convention'])
def test_bad_saved_states_rejected(tmp_path,start,bad):
    if bad=='left':start['left_rad']=[]
    if bad=='gripper':start['right_gripper_closed_motor_joint_rad']=0.
    if bad=='nan':start['right_rad'][0]=float('nan')
    if bad=='convention':start['coordinate_convention']='raw_motor'
    p=tmp_path/'state.json';p.write_text(json.dumps(start))
    with pytest.raises(ValueError):common.read_state(p)


def camera_meta():
    cal=common.calibration();meta=copy.deepcopy(cal['camera'])
    meta.update(protocol=1,alignment='native_depth',depth_stream_enabled=True,color_stream_enabled=False)
    return meta


@pytest.mark.parametrize('bad',['width','serial','scale_model','extrinsic','firmware'])
def test_camera_profile_or_calibration_mismatch_rejected(bad):
    cal=common.calibration();meta=camera_meta();depth.check_metadata(meta,cal)
    if bad=='width':meta['depth']['width']=640
    if bad=='serial':meta['serial']='wrong'
    if bad=='scale_model':meta['depth']['D_opencv_order'][0]=.1
    if bad=='extrinsic':meta['T_color_depth_m'][0][3]+=.01
    if bad=='firmware':meta['firmware']='new'
    with pytest.raises(ValueError):depth.check_metadata(meta,cal)


def test_partial_depth_and_invalid_scale_rejected():
    data=np.full((800,1280),1000,dtype='<u2').tobytes()
    h=dict(n=len(data),value_scale_m=.001)
    assert depth.decode(h,data)[0,0]==1000
    with pytest.raises(ValueError):depth.decode(h,data[:-1])
    h['value_scale_m']=float('nan')
    with pytest.raises(ValueError):depth.decode(h,data)


def test_empty_depth_never_becomes_a_clear_scene(tmp_path):
    frames=[(dict(value_scale_m=.001),np.zeros((800,1280),dtype='<u2')) for _ in range(5)]
    with pytest.raises(ValueError,match='Too few valid'):
        depth.snapshot(tmp_path,camera_meta(),frames,common.DEFAULT_CALIBRATION)


@pytest.fixture(scope='module')
def saved_scene(tmp_path_factory):
    folder=tmp_path_factory.mktemp('synthetic_depth')
    frames=[]
    for i in range(5):
        a=np.full((800,1280),1000,dtype='<u2');a[100,100]=0
        # A one-pixel edge confirmed in three frames remains an obstacle.
        # Missing frames still prevent a free-space certificate.
        a[200,200]=500 if i<3 else 0
        a[201,201]=500 if i==0 else 0
        frames.append((dict(value_scale_m=.001,sequence=i,received_monotonic=100+i*.1),a))
    depth.snapshot(folder,camera_meta(),frames,common.DEFAULT_CALIBRATION)
    return folder


def test_depth_scene_retains_thin_return_and_unknown(saved_scene):
    d=np.fromfile(saved_scene/'depth_m.bin',dtype='<f4').reshape(800,1280)
    f=np.fromfile(saved_scene/'free_depth_m.bin',dtype='<f4').reshape(800,1280)
    assert d[100,100]==0 and f[100,100]==0
    assert d[200,200]==.5 and f[200,200]==0
    assert d[201,201]==0 and f[201,201]==0  # unconfirmed, not clear
    metadata=json.loads((saved_scene/'scene.json').read_text())
    assert metadata['depth_fusion']['minimum_support']==3
    assert metadata['depth_fusion']['raw_frames_preserved']
    raw=np.load(saved_scene/'raw_depth.npz')
    assert raw['depth'][0,201,201]==500
    assert d[300,300]==1 and f[300,300]==1
    sc=planning.scene(saved_scene/'scene.json')
    assert sc['synthetic'] is False and sc['obstacle_padding_m']==.02
    K=np.array(sc['camera']['depth']['K']);T=np.array(sc['T_base_depth'])
    p=np.array([(300-K[0,2])/K[0,0],(300-K[1,2])/K[1,1],1.])@T[:3,:3].T+T[:3,3]
    cloud=np.fromfile(saved_scene/'points_base.f32',dtype='<f4').reshape(-1,3)
    assert np.min(np.linalg.norm(cloud-p,axis=1))<=np.sqrt(3)*.005+1e-6


@pytest.mark.parametrize('values,expected,free',[
    ([0,1068,0,916,914],0.,0.),  # actual intermittent stick-adjacent failure
    ([0,0,0,915,913],0.,0.),
    ([500,0,500,0,500],.5,0.),  # isolated thin edge; no spatial neighbors needed
    ([0,500,510,520,0],.5,0.),  # inclusive 20 mm cluster span
    ([0,500,510,521,0],0.,0.),
    ([1000,1000,500,500,500],.5,0.),  # confirmed foreground wins over background
    ([1000,1000,1000,500,0],0.,0.),  # never substitute farther background
    ([999,1000,1001,1002,1003],.999,.999),  # nearest, not median/average
    ([0,65535,149,5001,0],0.,0.),
    ([1000,1000,1000,1000,0],1.,0.),
])
def test_temporal_confirmation_preserves_thin_edges_and_unknown(values,expected,free):
    frames=[(dict(value_scale_m=.001),np.array([[v]],dtype='<u2')) for v in values]
    with np.errstate(all='raise'):
        arrays,scales,minimum,free_depth=depth.depth_window(frames)
    assert minimum[0,0]==pytest.approx(expected)
    assert free_depth[0,0]==pytest.approx(free)
    assert arrays[:,0,0].tolist()==values  # raw evidence never rewritten


def test_temporal_confirmation_uses_each_frame_scale_and_no_spatial_erosion():
    frames=[]
    for i in range(5):
        a=np.full((5,5),1000 if i%2==0 else 500,dtype='<u2')
        a[2,2]=(500 if i%2==0 else 250) if i<3 else 0
        frames.append((dict(value_scale_m=.001 if i%2==0 else .002),a))
    _,_,d,f=depth.depth_window(frames)
    assert d[2,2]==.5 and f[2,2]==0.
    assert np.count_nonzero(d==1.)==24
    assert np.count_nonzero(f==1.)==24


def test_planned_view_uses_saved_surface_not_new_fusion(saved_scene,tmp_path,monkeypatch):
    import shutil
    from types import SimpleNamespace
    from drum_planner import planned_view
    folder=tmp_path/'legacy_scene';shutil.copytree(saved_scene,folder)
    # Model a historical union containing a one-frame return. Today's fusion
    # would reject it, but its historical playback must NOT change underfoot.
    d=np.fromfile(folder/'depth_m.bin',dtype='<f4').reshape(800,1280)
    d[201,201]=.5;d.tofile(folder/'depth_m.bin')
    world=json.loads((folder/'scene.json').read_text());world.pop('depth_fusion')
    world['files_sha256']['depth_m.bin']=common.digest(folder/'depth_m.bin')
    common.atomic_json(folder/'scene.json',world)
    state=tmp_path/'state.json';state.write_text('{}')
    plan=SimpleNamespace(obj=dict(provenance=dict(scene=str(folder/'scene.json'),
        state=str(state),state_sha256=common.digest(state))),
        req=dict(scene=dict(source_scene_sha256=common.digest(folder/'scene.json'))))
    def denied(*a):raise AssertionError('Playback must not re-fuse saved evidence')
    monkeypatch.setattr(depth,'depth_window',denied)
    monkeypatch.setattr(planned_view,'prepare_scene',lambda h,d,*a:(h,d))
    h,rendered=planned_view.load(plan,common.DEFAULT_CALIBRATION)
    assert rendered[201,201]*h['value_scale_m']==.5


def test_temporally_confirmed_thin_obstacle_still_blocks_actual_moveit(req,tmp_path):
    T=Model(req['urdf']).transforms(dict(zip(common.JOINTS,req['start'])))['right_drumstick']
    obstacle=T[:3,3]+T[:3,0]*(.0074+.032)
    camera=np.eye(4);camera[:3,3]=obstacle-[0,0,.5]
    frames=[(dict(value_scale_m=.001),np.array([[500 if i<3 else 0]],dtype='<u2')) for i in range(5)]
    _,_,d,_=depth.depth_window(frames)
    points=depth.deproject(d,dict(depth=dict(K=np.eye(3).tolist())),camera)
    cloud=tmp_path/'confirmed_thin.f32'
    np.vstack([points,[3,3,3]]).astype('<f4').tofile(cloud)
    req['scene']=dict(synthetic=False,points_file=str(cloud),obstacle_padding_m=.02)
    req['drumstick_self_filter_padding_m']=planning.DRUMSTICK_SELF_FILTER_PADDING_M
    req['check_states']=[req['start']]
    checked=planning.backend(req,tmp_path,'confirmed_thin')
    assert not checked['checks'][0]['valid']
    assert ['gemini_native_depth','right_drumstick'] in checked['checks'][0]['contacts']


def test_scene_hashes_and_manual_obstacles_rejected(tmp_path,saved_scene):
    p=tmp_path/'bad_scene.json';d=json.loads((saved_scene/'scene.json').read_text())
    d['synthetic']=True;p.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='Gemini'):planning.scene(p)
    d['synthetic']=False;d['calibration_sha256']='wrong';p.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='different calibration'):planning.scene(p)


def test_smooth_cycle_is_endpoint_exact_and_not_creeping(start):
    target=common.endpoint()
    points,check,info=planning.smooth_samples([start['right_rad'],target.tolist()])
    assert np.array_equal(points[0]['positions_rad'],start['right_rad'])
    assert np.array_equal(points[-1]['positions_rad'],target)
    assert 3<info['duration_s']<8
    assert max(info['peak_velocity_rad_s'])<=.6
    assert max(info['peak_acceleration_rad_s2'])<=1.2
    assert max(info['peak_jerk_rad_s3'])<=6.
    cyc=planning.cycle(points)
    assert np.array_equal(cyc[-1]['positions_rad'],start['right_rad'])
    assert cyc[-1]['time_s']==pytest.approx(2*points[-1]['time_s']+2)
    assert all(b['time_s']>a['time_s'] for a,b in zip(cyc,cyc[1:]))
    assert points[0]['velocities_rad_s']==[0.]*7 and points[-1]['accelerations_rad_s2']==[0.]*7
    report=workflow.simulate(dict(status='planned',motion_authorized=False,outward=points,cycle=cyc,start_state=start))
    assert report['motor_commands']==0 and report['events'][-1]=='RELAX_INTENT_ONLY'


@pytest.mark.parametrize('kwargs',[
    dict(stall=True,at_saved_start=True,stationary=True,supported=True),
    dict(feedback_ok=False,checked_return_available=True),
    dict(motion_available=False,checked_return_available=True),
    dict(at_saved_start=True,stationary=True,supported=False),
    dict(at_saved_start=True,stationary=False,supported=True),dict()])
def test_faults_never_blindly_relax(kwargs):
    assert workflow.recovery_action(**kwargs)=='STOP_AND_HOLD_INTENT_NO_RELAX_OPERATOR_REQUIRED'


def test_recovery_requires_checked_return_and_support():
    assert workflow.recovery_action(checked_return_available=True)=='RETURN_TO_SAVED_START_INTENT'
    assert workflow.recovery_action(at_saved_start=True,stationary=True,supported=True)=='RELAX_INTENT_ONLY'
    with pytest.raises(RuntimeError,match='SOFTWARE-ONLY'):common.hardware_unavailable()


def stick_obstacle(req,q,offset=.14):
    m=Model(req['urdf']);T=m.transforms(dict(zip(common.JOINTS,q)))['right_drumstick']
    return dict(id='synthetic_stick_obstacle',xyz=(T[:3,3]+T[:3,2]*offset).tolist(),size=[.025]*3)


def test_moveit_rejects_stick_only_collision_and_checks_between_waypoints(req,tmp_path):
    mid=(np.array(req['start'])+req['goal'])/2
    req['scene']['boxes']=[stick_obstacle(req,mid)]
    req['check_states']=[req['start'],mid.tolist(),req['goal']]
    result=planning.backend(req,tmp_path,'stick')
    assert result['checks'][0]['valid'] and result['checks'][2]['valid']
    middle=result['checks'][1]
    assert not middle['valid'] and any('right_drumstick' in pair for pair in middle['contacts'])
    req['check_states']=[req['start'],req['goal']];req['check_path']=True
    result=planning.backend(req,tmp_path,'edge')
    assert result['checks'][0]['valid'] and not result['checks'][1]['valid']
    # Demonstrate that omitting the drumstick would incorrectly accept this pose.
    root=ET.parse(req['urdf']).getroot();stick=root.find("link[@name='right_drumstick']")
    stick.remove(stick.find('collision'));mutant=tmp_path/'test_ONLY_without_stick.urdf'
    ET.ElementTree(root).write(mutant,encoding='unicode')
    req['urdf']=str(mutant);req['check_states']=[mid.tolist()];req['check_path']=False
    result=planning.backend(req,tmp_path,'without_stick')
    assert result['checks'][0]['valid']


def test_real_moveit_plans_smooth_detour_around_stick_obstacle(req,tmp_path,generated,start):
    mid=(np.array(req['start'])+req['goal'])/2
    req['scene']['boxes']=[stick_obstacle(req,mid)]
    result=planning.finish_plan(tmp_path,req,generated[1],start,common.DEFAULT_CALIBRATION,dict(source='test'))
    assert result['status']=='planned',result
    assert not result['motion_authorized'] and not result['hardware_execution_available']
    assert np.array_equal(result['outward'][-1]['positions_rad'],common.endpoint())
    assert np.array_equal(result['cycle'][-1]['positions_rad'],start['right_rad'])
    assert result['reverse_checked_on_same_static_scene']
    assert result['duration_outward_s']<20
    assert len(result['path_waypoints'])>=3
    assert workflow.simulate(result)['motor_commands']==0
    optimization=result['optimization']
    assert optimization['searched_candidates']==planning.DEFAULT_CANDIDATES
    variants=[v for c in optimization['candidates'] for v in c['variants']]
    assert sum(v['status']=='selected' for v in variants)==1
    for v in variants:
        if v.get('duration_outward_s',float('inf'))<result['duration_outward_s']:
            assert v['status']=='rejected_collision_check' and not v['passed']


def test_fast_candidate_rejection_checks_all_states_only_if_valid(req,tmp_path):
    req['scene']['boxes']=[stick_obstacle(req,req['goal'])]
    req['check_states']=[req['start'],req['goal'],req['start']]
    req['stop_on_invalid']=True
    result=planning.backend(req,tmp_path,'early_rejection')
    assert result['status']=='checked'
    assert len(result['checks'])==2
    assert result['checks'][0]['valid'] and not result['checks'][1]['valid']
    req['stop_on_invalid']=False
    assert len(planning.backend(req,tmp_path,'full_diagnostic')['checks'])==3
    req['stop_on_invalid']=True
    req['check_states']=[req['start']]*3
    req['check_path']=True
    checked=planning.backend(req,tmp_path,'valid_full')['checks']
    assert len(checked)==3 and all(c['valid'] for c in checked)


def test_colliding_goal_not_forced_or_clipped(req,tmp_path):
    req['scene']['boxes']=[stick_obstacle(req,req['goal'])]
    result=planning.backend(req,tmp_path)
    assert result['status']=='blocked_goal' and result['details']['reason']=='collision'


def test_joint_limit_violations_rejected_without_recovery(req,tmp_path):
    req['start'][0]=99
    result=planning.backend(req,tmp_path)
    assert result['status']=='blocked_start' and result['details']['reason']=='joint_limits'
    assert result['details']['violations'][0]['joint']=='openarmx_right_joint1'
    assert not result['details']['angles_clamped']


@pytest.mark.parametrize('offset_counts',[-3.,-1.5,-.5])
def test_static_left_encoder_boundary_allowance_keeps_measured_pose(req,tmp_path,offset_counts):
    req['left'][3]=offset_counts*25.14/65535
    measured=req['left'][3]
    req['check_states']=[req['start'],req['goal']]
    result=planning.backend(req,tmp_path)
    assert result['status']=='checked'
    bounds=result['joint_limit_check']
    assert bounds['valid'] and bounds['moving_right_limits_unchanged'] and not bounds['angles_clamped']
    accepted=bounds['accepted_static_encoder_overshoot']
    assert len(accepted)==1 and accepted[0]['joint']=='openarmx_left_joint4'
    assert accepted[0]['measured_rad']==measured and req['left'][3]==measured
    assert accepted[0]['measurement_tolerance_rad']==pytest.approx(3*25.14/65535)
    assert all(row['valid'] for row in result['checks'])


@pytest.mark.parametrize('which',['left_beyond','right_start','right_goal','left_finger'])
def test_encoder_allowance_never_widens_commanded_or_finger_limits(req,tmp_path,which):
    count=25.14/65535
    # Bounds are in q_saved, just like the request, not the uncorrected q_model.
    right_lower=float(ET.parse(req['urdf']).getroot().find("joint[@name='openarmx_right_joint4']/limit").get('lower'))
    if which=='left_beyond':req['left'][3]=-3.01*count
    if which=='right_start':req['start'][3]=right_lower-.5*count
    if which=='right_goal':req['goal'][3]=right_lower-.5*count
    if which=='left_finger':req['left_opening_m']=-.00001
    result=planning.backend(req,tmp_path)
    assert result['status']==('blocked_goal' if which=='right_goal' else 'blocked_start')
    detail=result['details'];assert detail['reason']=='joint_limits' and not detail['angles_clamped']
    assert detail['violations']


def test_tiny_static_left_overshoot_survives_real_planning_and_smoothing(req,tmp_path,generated,start):
    req['left'][3]=-.0005754177157246687
    start=copy.deepcopy(start);start['left_rad']=req['left'][:]
    result=planning.finish_plan(tmp_path,req,generated[1],start,common.DEFAULT_CALIBRATION,dict(source='offline regression'))
    assert result['status']=='planned'
    assert result['start_state']['left_rad'][3]==req['left'][3]
    assert all(a['passed'] for a in result['smoothing_attempts'])
    assert result['outward'][0]['positions_rad']==req['start']
    assert result['cycle'][-1]['positions_rad']==req['start']


def test_backend_exceptions_keep_schema_and_open_as_nonplayable_reports(req,tmp_path,generated,start):
    req['minimum_self_clearance_m']=.2  # deliberately invalid; model itself remains intact
    result=planning.finish_plan(tmp_path,req,generated[1],start,common.DEFAULT_CALIBRATION,dict(source='offline regression'))
    assert result['status']=='error' and result['schema']=='x5-moveit-result-v1'
    from drum_planner.trajectory_view import PlanDisplay
    display=PlanDisplay(tmp_path/'plan.json')
    assert not display.playable and display.duration==0
    assert 'Invalid self clearance' in display.warning and 'NO TRAJECTORY' in display.warning


def test_unknown_pixels_are_not_free_space(req,tmp_path):
    req['collision_policy']='observed_free_space'  # historical strict mode, not normal planning
    np.array([[3.,3.,3.]],dtype='<f4').tofile(tmp_path/'cloud.bin')
    np.zeros((800,1280),dtype='<f4').tofile(tmp_path/'free.bin')
    req['scene']=dict(synthetic=False,obstacle_padding_m=.02,points_file=str(tmp_path/'cloud.bin'),
                      free_depth_file=str(tmp_path/'free.bin'),camera=camera_meta(),
                      T_base_depth=common.calibration()['T_base_depth'])
    result=planning.backend(req,tmp_path)
    assert result['status']=='blocked_start' and result['details']['reason']=='unknown_or_occluded_space'
    report=result['visibility_coverage']
    assert set(report)=={'start','goal'}
    for pose in report.values():
        assert pose['diagnostic_only'] and not pose['collision_checked']
        assert pose['moving_links_checked']==11
        assert pose['links_with_unobserved_clearance']==11
        names={row['link'] for row in pose['links']}
        assert 'right_drumstick' in names and 'openarmx_right_link1' in names
        assert not any(name.startswith('openarmx_left_') for name in names)
        for row in pose['links']:
            assert sum(row['failure_counts'].values())==row['blocked_probe_count']
            assert 0<row['blocked_probe_count']<=row['probe_count']
    # Exact surface extrema, independent Python FK/STL implementation. This is
    # the nominal shape, not an inflated AABB or visibility sphere.
    root=ET.parse(req['urdf']).getroot()
    collision=root.find("link[@name='openarmx_right_link1']/collision")
    mesh=collision.find('geometry/mesh')
    vertices=model.stl_vertices(mesh.get('filename').removeprefix('file://')).reshape(-1,3)
    vertices=vertices*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
    origin=collision.find('origin');local=np.eye(4)
    if origin is not None:
        local[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        local[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
    T=np.linalg.inv(req['scene']['T_base_depth'])@Model(req['urdf']).transforms(
        dict(zip(common.JOINTS,req['start'])))['openarmx_right_link1']@local
    z=(vertices@T[:3,:3].T+T[:3,3])[:,2]
    row=next(row for row in report['start']['links'] if row['link']=='openarmx_right_link1')
    # Assimp/geometric_shapes and Python retain slightly different float precision.
    assert row['nominal_geometry_camera_z_min_m']==pytest.approx(z.min(),abs=1e-7)
    assert row['nominal_geometry_camera_z_max_m']==pytest.approx(z.max(),abs=1e-7)
    # Diagnostics must not alter the original verdict or create a trajectory.
    req['check_states']=[req['start']]
    checked=planning.backend(req,tmp_path,'after_coverage')
    assert not checked['checks'][0]['valid']
    assert checked['checks'][0]['visibility']==result['details']['visibility']
    assert 'outward' not in result and not result['motion_authorized']


@pytest.mark.parametrize('observed',[0.,2.,4.])
def test_strict_visibility_accepts_observed_clearance_not_holes_or_occlusions(req,tmp_path,observed):
    req['collision_policy']='observed_free_space'
    # ARTIFICIAL camera above the entire arm, NOT a change to real calibration.
    # Real MoveIt backend; no FOV exemptions or occupied-only fallback.
    np.array([[3.,3.,3.]],dtype='<f4').tofile(tmp_path/'cloud.bin')
    np.full((800,1280),observed,dtype='<f4').tofile(tmp_path/'free.bin')
    req['scene']=dict(synthetic=False,obstacle_padding_m=.02,points_file=str(tmp_path/'cloud.bin'),
                      free_depth_file=str(tmp_path/'free.bin'),camera=camera_meta(),
                      T_base_depth=[[1,0,0,0],[0,-1,0,0],[0,0,-1,3],[0,0,0,1]])
    req['check_states']=[req['start'],req['goal']]
    result=planning.backend(req,tmp_path)
    assert result['status']=='checked' and result['strict_unknown_space']
    assert all(row['valid']==(observed==4.) for row in result['checks'])
    if observed!=4.:
        expected='missing_or_invalid_depth' if observed==0 else 'observed_surface_occludes_required_clearance'
        assert all(row['visibility']['reason']==expected for row in result['checks'])


def test_coverage_message_explains_refusal_without_claiming_connection_failure():
    from drum_planner.diagnostics import blockage_summary
    obj=dict(status='blocked_start',details=dict(reason='unknown_or_occluded_space'),visibility_coverage=dict(
        start=dict(links_with_unobserved_clearance=2,moving_links_checked=11,links=[dict(
            failure_counts=dict(body_or_clearance_buffer_outside_depth_fov=1))])))
    message=blockage_summary(obj)
    assert 'START: 2/11' in message and 'outside depth FOV' in message
    assert 'not a camera connection failure' in message and 'NOT been marked clear' in message
    assert blockage_summary(dict(status='planned'))==''
    assert 'example error' in blockage_summary(dict(status='error',error='example error'))


def test_default_policy_allows_unseen_arm_but_retains_occupied_collision_checks(req,tmp_path):
    # The real camera transform excludes the shoulder/resting arm. Empty free-
    # depth pixels must no longer reject either endpoint under the NEW policy.
    np.array([[3.,3.,3.]],dtype='<f4').tofile(tmp_path/'cloud.bin')
    np.zeros((800,1280),dtype='<f4').tofile(tmp_path/'free.bin')
    req['scene']=dict(synthetic=False,obstacle_padding_m=.02,points_file=str(tmp_path/'cloud.bin'),
                      free_depth_file=str(tmp_path/'free.bin'),camera=camera_meta(),
                      T_base_depth=common.calibration()['T_base_depth'])
    req['check_states']=[req['start'],req['goal']]
    result=planning.backend(req,tmp_path,'observed')
    assert result['collision_policy']=='observed_obstacles'
    assert not result['strict_unknown_space'] and not result['unobserved_space_blocks_motion']
    assert all(row['valid'] for row in result['checks'])
    # A detected voxel at the goal stick is still an obstacle, even though its
    # pixel is unknown in the free-depth image. No depth/FOV-based voxel culling.
    obstacle=stick_obstacle(req,req['goal'])
    np.array([obstacle['xyz'],[3.,3.,3.]],dtype='<f4').tofile(tmp_path/'cloud.bin')
    result=planning.backend(req,tmp_path,'observed_collision')
    assert result['checks'][0]['valid']
    assert not result['checks'][1]['valid'] and result['checks'][1]['reason']=='collision'
    assert any('right_drumstick' in pair for pair in result['checks'][1]['contacts'])
    # Historical requests are NOT silently reinterpreted as the new policy.
    req.pop('collision_policy');req['check_states']=[req['start']]
    old=planning.backend(req,tmp_path,'legacy_strict')
    assert old['strict_unknown_space'] and old['collision_policy']=='observed_free_space'
    assert not old['checks'][0]['valid'] and old['checks'][0]['reason']=='unknown_or_occluded_space'


@pytest.mark.parametrize('link',['openarmx_right_link1','openarmx_right_link6','openarmx_right_hand'])
def test_observed_policy_still_checks_upper_arm_and_end_joints(req,tmp_path,link):
    root=ET.parse(req['urdf']).getroot();c=root.find(f"link[@name='{link}']/collision")
    mesh=c.find('geometry/mesh');v=model.stl_vertices(mesh.get('filename').removeprefix('file://'))[0,0]
    v=v*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
    local=np.eye(4);origin=c.find('origin')
    if origin is not None:
        local[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        local[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
    T=Model(req['urdf']).transforms(dict(zip(common.JOINTS,req['start'])))[link]@local
    req['scene']['boxes']=[dict(id='test_detected_obstacle',xyz=(T[:3,:3]@v+T[:3,3]).tolist(),size=[.025]*3)]
    result=planning.backend(req,tmp_path)
    assert result['status']=='blocked_start' and result['details']['reason']=='collision'
    assert any(link in pair and 'test_detected_obstacle' in pair for pair in result['details']['contacts'])


def test_unknown_policy_or_conflicting_legacy_flag_rejected(req,tmp_path):
    req['collision_policy']='typo'
    assert planning.backend(req,tmp_path,'bad')['status']=='error'
    req['collision_policy']='observed_free_space';req['occupied_only_preview']=True
    assert planning.backend(req,tmp_path,'conflict')['status']=='error'


def test_no_hardware_execute_option(req,tmp_path):
    req['execute']=True
    result=planning.backend(req,tmp_path)
    assert result['status']=='error' and not result['motion_authorized']
    from drum_planner.cli import main
    with pytest.raises(SystemExit) as e:main(['plan','--execute'])
    assert e.value.code==2



def test_thin_stick_self_filter_is_start_only_and_keeps_nearby_obstacles(req,tmp_path):
    frames=Model(req['urdf']).transforms(dict(zip(common.JOINTS,req['start'])))
    T=frames['right_drumstick']
    # Camera return 15 mm outside the nominal thin-stick surface, within the
    # explicit depth + voxel uncertainty mask; not inside the old 5 mm mask.
    own=(T@np.array([model.STICK_RADIUS+.015,0,0,1]))[:3]
    external=(T@np.array([model.STICK_RADIUS+.032,0,0,1]))[:3]
    cloud=tmp_path/'cloud.bin';np.array([own,[3.,3.,3.]],dtype='<f4').tofile(cloud)
    req['scene']=dict(synthetic=False,obstacle_padding_m=.02,points_file=str(cloud),
                      camera=camera_meta(),T_base_depth=common.calibration()['T_base_depth'])
    req['check_states']=[req['start'],req['goal']]
    old=planning.backend(req,tmp_path,'legacy_self_mask')
    assert not old['checks'][0]['valid'] and old['checks'][0]['reason']=='collision'
    assert old['robot_self_filter']['drumstick_padding_m']==.005
    req['drumstick_self_filter_padding_m']=planning.DRUMSTICK_SELF_FILTER_PADDING_M
    checked=planning.backend(req,tmp_path,'bounded_stick_mask')
    assert all(p['valid'] for p in checked['checks'])
    mask=checked['robot_self_filter']
    assert mask['removed_voxels_per_link']['right_drumstick']==1
    assert mask['initial_pose_only'] and not mask['swept_path_masking']
    assert not mask['collision_geometry_changed'] and mask['body_padding_m']==.005
    # A close external voxel outside that fixed mask STILL rejects the start.
    np.array([own,external,[3.,3.,3.]],dtype='<f4').tofile(cloud)
    blocked=planning.backend(req,tmp_path,'near_stick_obstacle')
    assert not blocked['checks'][0]['valid'] and blocked['checks'][0]['reason']=='collision'
    # It must never clear the future route/goal merely because the tool goes there.
    obstacle=stick_obstacle(req,req['goal'])
    np.array([own,obstacle['xyz'],[3.,3.,3.]],dtype='<f4').tofile(cloud)
    blocked=planning.backend(req,tmp_path,'future_stick_obstacle')
    assert blocked['checks'][0]['valid'] and not blocked['checks'][1]['valid']


@pytest.mark.parametrize('padding',[.004,.029001,.1,-1])
def test_self_filter_padding_cannot_grow_to_hide_a_collision(req,tmp_path,padding):
    req['drumstick_self_filter_padding_m']=padding
    result=planning.backend(req,tmp_path,'invalid_self_mask')
    assert result['status']=='error' and 'self-filter' in result['error']


def test_depth_shadow_filter_keeps_foreground_and_other_camera_rays(req,tmp_path):
    # Exact failed start, for which the camera sees the bottom of the stick.
    req['start']=[.004027924010070905,.013234607461662051,.4536209658960857,
                  -.019756008239872003,-.24406413313501255,.17818768596932877,.558346990157931]
    T=Model(req['urdf']).transforms(dict(zip(common.JOINTS,req['start'])))['right_drumstick']
    camera=np.array(common.calibration()['T_base_depth'])[:3,3]
    center=T[:3,3]
    shadow=camera+1.2*(center-camera)
    foreground=camera+.8*(center-camera)
    # A different ray, not hidden by the nominal stick, must be kept.
    side=shadow+.15*T[:3,0]
    cloud=tmp_path/'shadow_cloud.bin'
    np.array([shadow,foreground,side,[3.,3.,3.]],dtype='<f4').tofile(cloud)
    req['scene']=dict(synthetic=False,obstacle_padding_m=.02,points_file=str(cloud),
                      camera=camera_meta(),T_base_depth=common.calibration()['T_base_depth'])
    req['check_states']=[req['start']]
    req['filter_drumstick_depth_shadow']=True
    req['drumstick_self_filter_padding_m']=.005  # ray behavior is separate from padding
    checked=planning.backend(req,tmp_path,'nominal_ray_shadow')
    assert checked['status']=='checked',checked
    mask=checked['robot_self_filter']
    assert mask['nominal_stick_depth_shadow_filter']
    assert mask['shadow_voxels']==1
    assert checked['self_filtered_voxels']==1  # foreground and neighboring rays retained
    req['filter_drumstick_depth_shadow']=False
    legacy=planning.backend(req,tmp_path,'without_ray_shadow')
    assert legacy['self_filtered_voxels']==0
    # Bad/missing camera geometry cannot silently enable a filtering shortcut.
    req['filter_drumstick_depth_shadow']=True;req['scene'].pop('T_base_depth')
    assert planning.backend(req,tmp_path,'missing_shadow_camera')['status']=='error'
