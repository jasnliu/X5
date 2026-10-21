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
        # A close thin return seen only once must remain an obstacle, but it
        # cannot create certified free space through the other missing frames.
        a[200,200]=500 if i==0 else 0
        frames.append((dict(value_scale_m=.001,sequence=i,received_monotonic=100+i*.1),a))
    depth.snapshot(folder,camera_meta(),frames,common.DEFAULT_CALIBRATION)
    return folder


def test_depth_scene_retains_thin_return_and_unknown(saved_scene):
    d=np.fromfile(saved_scene/'depth_m.bin',dtype='<f4').reshape(800,1280)
    f=np.fromfile(saved_scene/'free_depth_m.bin',dtype='<f4').reshape(800,1280)
    assert d[100,100]==0 and f[100,100]==0
    assert d[200,200]==.5 and f[200,200]==0
    assert d[300,300]==1 and f[300,300]==1
    sc=planning.scene(saved_scene/'scene.json')
    assert sc['synthetic'] is False and sc['obstacle_padding_m']==.02
    K=np.array(sc['camera']['depth']['K']);T=np.array(sc['T_base_depth'])
    p=np.array([(300-K[0,2])/K[0,0],(300-K[1,2])/K[1,1],1.])@T[:3,:3].T+T[:3,3]
    cloud=np.fromfile(saved_scene/'points_base.f32',dtype='<f4').reshape(-1,3)
    assert np.min(np.linalg.norm(cloud-p,axis=1))<=np.sqrt(3)*.005+1e-6


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
    if which=='left_beyond':req['left'][3]=-3.01*count
    if which=='right_start':req['start'][3]=-.5*count
    if which=='right_goal':req['goal'][3]=-.5*count
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
