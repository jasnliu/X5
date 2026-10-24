"""Occupied volume clearance accounting; real native checks, no devices."""
import copy,json,socket
from pathlib import Path
import numpy as np
import pytest
from drum_planner import common,planning,model,depth
from drum_planner.baseline import tasks
from drum_planner.experiment import algorithms,config,runtime
from safe_zone.geometry import Model
D=common.ROOT/'diagnostics/start_collision_20261021'

@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    original=socket.socket
    def socket_guard(family=socket.AF_INET,*a,**kw):
        if family==socket.AF_CAN:pytest.fail('No physical CAN in voxel tests')
        return original(family,*a,**kw)
    monkeypatch.setattr(socket,'socket',socket_guard)
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**k:pytest.fail('No physical camera'))

@pytest.fixture(scope='module')
def geometry(tmp_path_factory):
    folder=tmp_path_factory.mktemp('voxel_geometry');model.generate(folder);return folder

@pytest.fixture
def req(tmp_path,geometry):
    case=json.loads((D/'incident_case.json').read_text())
    path=tmp_path/'points.bin';np.array(case['colliding_voxel_centers']+[[3,3,3]],dtype='<f4').tofile(path)
    r={k:case[k] for k in ('start','left','goal','left_opening_m','drumstick_self_filter_padding_m','filter_drumstick_depth_shadow')}
    r.update(schema='x5-moveit-request-v1',execute=False,urdf=str(geometry/'robot.urdf'),srdf=str(geometry/'robot.srdf'),
             scene=dict(synthetic=False,obstacle_padding_m=.02,points_file=str(path),T_base_depth=case['camera_transform']),
             collision_policy='observed_obstacles',check_states=[r['start'],r['goal']])
    return r


def test_exact_failed_voxels_clear_only_duplicate_padding_removed(req,tmp_path):
    old=planning.backend(req,tmp_path/'old');assert [x['valid'] for x in old['checks']]==[False,True]
    req['world_padding_policy']=planning.WORLD_PADDING_POLICY
    result=planning.backend(req,tmp_path/'new');assert all(x['valid'] for x in result['checks'])
    assert result['world_padding']==planning.world_padding_metadata(req)
    assert result['world_padding']['robot_padding_m']==.025
    assert result['robot_self_filter']==old['robot_self_filter']
    assert result['self_filtered_voxels']==old['self_filtered_voxels']==0
    assert any('right_drumstick' in c for c in old['checks'][0]['contacts'])


def test_failed_voxels_have_at_least_twenty_five_mm_true_geometric_clearance(req):
    pts=np.fromfile(req['scene']['points_file'],dtype='<f4').reshape(-1,3)
    T=Model(req['urdf']).transforms(dict(zip(common.JOINTS,req['start'])))['right_drumstick']
    local=(pts-T[:3,3])@T[:3,:3]
    dr=np.linalg.norm(local[:,:2],axis=1)-model.STICK_RADIUS;dz=abs(local[:,2])-.4094/2
    sd=np.hypot(np.maximum(dr,0),np.maximum(dz,0))+np.minimum(np.maximum(dr,dz),0)
    # Any point in a 10mm cube lies within sqrt(3)*5mm of its center.
    assert sd.min()-np.sqrt(3)*.005>.0296
    assert sd.min()-np.sqrt(3)*.005>.02+.005


def test_existing_legacy_requests_and_explicit_legacy_remain_identical(req,tmp_path):
    a=planning.backend(req,tmp_path/'implicit');req['world_padding_policy']='legacy_voxel_radius_v1'
    b=planning.backend(req,tmp_path/'explicit')
    assert a['checks']==b['checks'] and a['world_padding']==b['world_padding']
    assert b['world_padding']['additional_voxel_radius_m']==.009


@pytest.mark.parametrize('bad',['typo','',False,123])
def test_invalid_policy_fails_in_python_and_native(req,tmp_path,bad):
    req['world_padding_policy']=bad
    with pytest.raises(ValueError,match='padding policy'):planning.world_padding_metadata(req)
    result=planning.backend(req,tmp_path)
    assert result['status']=='error'


def test_synthetic_obstacle_within_requested_buffer_still_blocks(req,tmp_path):
    T=Model(req['urdf']).transforms(dict(zip(common.JOINTS,req['goal'])))['right_drumstick']
    req['world_padding_policy']=planning.WORLD_PADDING_POLICY
    # This entire box is near the stick surface (<20mm), not merely its center.
    point=(T@np.array([model.STICK_RADIUS+.018,0,0,1]))[:3]
    req['scene']=dict(synthetic=True,obstacle_padding_m=.02,boxes=[dict(id='physical_obstacle',xyz=point.tolist(),size=[.002]*3)])
    result=planning.backend(req,tmp_path)
    assert result['checks'][0]['valid'] and not result['checks'][1]['valid']
    assert any('right_drumstick' in pair and 'physical_obstacle' in pair for pair in result['checks'][1]['contacts'])


def test_retained_goal_voxel_inside_stick_is_not_filtered_or_exempted(req,tmp_path):
    T=Model(req['urdf']).transforms(dict(zip(common.JOINTS,req['goal'])))['right_drumstick']
    p=Path(req['scene']['points_file']);pts=np.fromfile(p,dtype='<f4').reshape(-1,3)
    np.vstack((pts,T[:3,3])).astype('<f4').tofile(p)
    req['world_padding_policy']=planning.WORLD_PADDING_POLICY
    result=planning.backend(req,tmp_path)
    assert result['checks'][0]['valid'] and not result['checks'][1]['valid']
    assert result['checks'][1]['reason']=='collision'


def test_oracle_and_final_checker_use_exact_same_volume_policy(req,tmp_path):
    req['world_padding_policy']=planning.WORLD_PADDING_POLICY
    path=tmp_path/'request.json';common.atomic_json(path,req)
    oracle=runtime.Oracle(path,tmp_path)
    try:
        assert oracle.meta['world_padding']==planning.world_padding_metadata(req)
        assert oracle.check([req['start'],req['goal']])['valid']==[True,True]
    finally:oracle.close()
    assert all(x['valid'] for x in planning.backend(req,tmp_path/'final')['checks'])


def test_cache_cannot_mix_legacy_and_corrected_clearance(req,tmp_path):
    context=dict(scene_sha256='scene',state_sha256='state',calibration=str(common.DEFAULT_CALIBRATION))
    legacy=algorithms.pool_key(context,req,config.settings(),120)
    req['world_padding_policy']=planning.WORLD_PADDING_POLICY
    assert algorithms.pool_key(context,req,config.settings(),120)!=legacy


@pytest.mark.parametrize('side',['left','right'])
def test_both_new_arm_requests_explicitly_choose_same_policy(tmp_path,side):
    state=common.read_state(common.ROOT/'drum_planner/config/example_start.json')
    r,_=planning.request(tmp_path,state,dict(synthetic=True,boxes=[],obstacle_padding_m=.02),moving_side=side)
    assert r['world_padding_policy']==planning.WORLD_PADDING_POLICY
    assert planning.world_padding_metadata(r)['robot_padding_m']==.025


def test_prepared_proof_records_new_policy_and_retains_legacy_shape(req,tmp_path):
    legacy=tasks.margin_contract(req);assert 'world_padding' not in legacy
    req['world_padding_policy']=planning.WORLD_PADDING_POLICY
    proof=tasks.hard_check(req,[req['start']],tmp_path)
    assert proof['world_padding']==planning.world_padding_metadata(req)
    assert proof['obstacle_padding_m']==legacy['obstacle_padding_m']==.02


def test_corrected_endpoints_do_not_bypass_collision_on_connecting_edge(req,tmp_path):
    req['world_padding_policy']=planning.WORLD_PADDING_POLICY
    # Both endpoints are clear, but a naive straight joint interpolation is
    # not. The real planner must search for a detour, not waive this contact.
    with pytest.raises(tasks.PathBlocked):tasks.hard_check(req,[req['start'],req['goal']],tmp_path)


def test_wrong_native_padding_proof_rejected(req,tmp_path,monkeypatch):
    req['world_padding_policy']=planning.WORLD_PADDING_POLICY
    monkeypatch.setattr(planning,'backend',lambda *a,**kw:dict(status='checked',collision_policy='observed_obstacles',
        strict_unknown_space=False,checks=[dict(valid=True)],world_padding={'policy':'wrong'}))
    with pytest.raises(ValueError,match='world-padding proof'):tasks.hard_check(req,[req['start']],tmp_path)
