

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
