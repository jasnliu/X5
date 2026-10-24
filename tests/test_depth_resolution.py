"""5 mm occupancy from full native depth, unchanged margins and archived replay."""
from test_checkpoint_performance import OPTIMIZED
import copy,json,socket
from pathlib import Path
import numpy as np
import pytest
from drum_planner import common,depth,planning,model
from drum_planner.experiment import worker,config,algorithms,runtime
D=common.ROOT/'diagnostics/voxel_5mm_20261022'

@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    real=socket.socket
    def guard(family=socket.AF_INET,*a,**k):
        if family==socket.AF_CAN:pytest.fail('No physical CAN')
        return real(family,*a,**k)
    monkeypatch.setattr(socket,'socket',guard)
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**k:pytest.fail('No physical camera'))

@pytest.fixture
def source(tmp_path,monkeypatch):
    meta=common.calibration(common.DEFAULT_CALIBRATION)['camera']
    # Use validated SDK metadata from the immutable saved input, no camera open.
    _,path=config.saved_inputs('right');meta=json.loads(Path(path).read_text())['camera']
    a=np.zeros((800,1280),dtype='<u2');a[100:180,400:480]=1000
    frames=[(dict(sequence=i,value_scale_m=.001,received_monotonic=float(i)),a.copy()) for i in range(5)]
    folder=tmp_path/'original';folder.mkdir()
    with monkeypatch.context() as m:
        m.setattr(depth,'VOXEL_SIZE_M',.01)
        depth.snapshot(folder,meta,frames,common.DEFAULT_CALIBRATION)
    return folder/'scene.json',meta,frames


def test_fresh_snapshot_uses_five_mm_and_every_confirmed_return(source,tmp_path):
    path,meta,frames=source;out=tmp_path/'fresh';out.mkdir()
    obj=depth.snapshot(out,meta,frames,common.DEFAULT_CALIBRATION)
    assert obj['voxel_size_m']==.005 and obj['obstacle_padding_m']==.02
    assert obj['camera']['depth']['width']==1280 and obj['camera']['depth']['height']==800
    _,_,minimum,_=depth.depth_window(frames)
    expect=depth.occupied_voxels(depth.deproject(minimum,meta,obj['T_base_depth']),.005).astype('<f4')
    assert np.array_equal(np.fromfile(out/'points_base.f32',dtype='<f4').reshape(-1,3),expect)
    for name in ('raw_depth.npz','depth_m.bin','free_depth_m.bin'):
        assert common.digest(out/name)==common.digest(path.parent/name)
    assert planning.scene(out/'scene.json')['voxel_size_m']==.005
    assert depth.refine_snapshot(out/'scene.json',tmp_path/'unused')==out/'scene.json'
    assert not (tmp_path/'unused').exists()


def test_refine_reprojects_pixels_not_old_centers_and_preserves_source(source,tmp_path):
    path,meta,_=source;before={p.name:common.digest(p) for p in path.parent.iterdir()}
    out=depth.refine_snapshot(path,tmp_path/'fine');obj=planning.scene(out)
    minimum=np.fromfile(path.parent/'depth_m.bin',dtype='<f4').reshape(800,1280)
    expect=depth.occupied_voxels(depth.deproject(minimum,meta,obj['T_base_depth']),.005).astype('<f4')
    assert np.array_equal(np.fromfile(obj['points_file'],dtype='<f4').reshape(-1,3),expect)
    assert len(expect)>json.loads(path.read_text())['point_count']
    assert before=={p.name:common.digest(p) for p in path.parent.iterdir()}
    for name in ('raw_depth.npz','depth_m.bin','free_depth_m.bin','depth_preview.png'):
        assert common.digest(out.parent/name)==before[name]
    assert planning.scene(path)['voxel_size_m']==.01
    assert obj['refinement']['source_scene_sha256']==before['scene.json']
    assert depth.refine_snapshot(path,tmp_path/'fine')==out  # idempotent immutable derivation
    other=depth.refine_snapshot(path,tmp_path/'another')
    assert common.digest(other)==common.digest(out)  # deterministic cache identity
    from drum_planner.planned_view import load_scene
    _,old_mesh=load_scene(path,common.DEFAULT_CALIBRATION)
    _,new_mesh=load_scene(out,common.DEFAULT_CALIBRATION)
    assert np.array_equal(old_mesh.vertices,new_mesh.vertices)
    assert np.array_equal(old_mesh.faces,new_mesh.faces)


def test_refinement_rejects_corrupt_hashed_depth_and_different_destination(source,tmp_path):
    path,_,_=source;out=depth.refine_snapshot(path,tmp_path/'fine')
    obj=json.loads(out.read_text());obj['refinement']['source_scene_sha256']='changed';common.atomic_json(out,obj)
    with pytest.raises(ValueError,match='different inputs'):depth.refine_snapshot(path,tmp_path/'fine')
    (path.parent/'depth_m.bin').write_bytes(b'bad')
    with pytest.raises(ValueError,match='hash mismatch'):depth.refine_snapshot(path,tmp_path/'bad')
    assert not (tmp_path/'bad').exists()


def test_new_experiment_initialization_refines_saved_map_but_not_archived_plan(source,tmp_path):
    path,_,_=source;state,_=config.saved_inputs('right')
    folder=tmp_path/'experiment';worker.initialize(folder,state,path,common.DEFAULT_CALIBRATION)
    ctx=json.loads((folder/'context.json').read_text());req=json.loads((folder/'initial/backend_request.json').read_text())
    assert ctx['scene']!=str(path) and req['scene']['voxel_size_m']==.005
    assert ctx['scene_sha256']==common.digest(ctx['scene'])
    assert ctx['state_sha256']==common.digest(state)
    assert planning.scene(path)['voxel_size_m']==.01


@pytest.mark.parametrize('size',[.005,.01])
def test_same_clearance_for_both_resolutions(size):
    r=dict(world_padding_policy='occupied_volume_v1',scene=dict(obstacle_padding_m=.02,voxel_size_m=size))
    assert planning.world_padding_metadata(r)==dict(policy='occupied_volume_v1',requested_clearance_m=.02,
        edge_sampling_allowance_m=.005,additional_voxel_radius_m=0.,robot_padding_m=.025,
        occupied_octree_cells_are_volumes=True,octree_resolution_m=size)
    assert planning.DRUMSTICK_SELF_FILTER_PADDING_M==.029


@pytest.fixture(scope='module')
def geometry(tmp_path_factory):
    folder=tmp_path_factory.mktemp('resolution_geometry');model.generate(folder);return folder

@pytest.fixture
def req(tmp_path,geometry):
    case=json.loads((D/'incident_case.json').read_text())
    r={k:case[k] for k in ('start','left','goal','left_opening_m','drumstick_self_filter_padding_m','filter_drumstick_depth_shadow')}
    r.update(schema='x5-moveit-request-v1',execute=False,urdf=str(geometry/'robot.urdf'),srdf=str(geometry/'robot.srdf'),
        scene=dict(synthetic=False,obstacle_padding_m=.02,T_base_depth=case['camera_transform']),
        world_padding_policy='occupied_volume_v1',collision_policy='observed_obstacles',check_states=[r['start'],r['goal']])
    r['scene']['points_file']=str(tmp_path/'points.bin')
    return r,case


def test_original_colliding_cell_refined_from_returns_passes_native_and_oracle(req,tmp_path):
    r,case=req;p=Path(r['scene']['points_file']);points=np.array(case['confirmed_points']+[[3.,3.,3.]])
    r['scene']['voxel_size_m']=.01;depth.occupied_voxels(points,.01).astype('<f4').tofile(p)
    old=planning.backend(r,tmp_path/'old');assert [c['valid'] for c in old['checks']]==[False,True]
    r['scene']['voxel_size_m']=.005;depth.occupied_voxels(points,.005).astype('<f4').tofile(p)
    new=planning.backend(r,tmp_path/'new');assert all(c['valid'] for c in new['checks'])
    assert new['world_padding']==planning.world_padding_metadata(r)
    assert new['self_filtered_voxels']==old['self_filtered_voxels']==0
    path=tmp_path/'oracle.json';common.atomic_json(path,r);oracle=runtime.Oracle(path,tmp_path)
    try:
        assert oracle.check(r['check_states'])['valid']==[True,True]
        assert oracle.meta['world_padding']==new['world_padding']
    finally:oracle.close()
    # Genuine obstacle at the goal must still be rejected at 5 mm.
    from safe_zone.geometry import Model
    T=Model(r['urdf']).transforms(dict(zip(common.JOINTS,r['goal'])))['right_drumstick']
    points=np.vstack((points,T[:3,3]));depth.occupied_voxels(points,.005).astype('<f4').tofile(p)
    blocked=planning.backend(r,tmp_path/'blocked');assert not blocked['checks'][1]['valid']
    assert blocked['checks'][1]['reason']=='collision'


@pytest.mark.parametrize('bad',[0.,.001,.02,-.005,'0.005',None,False])
def test_unsupported_resolution_rejected_by_both_backends(req,tmp_path,bad):
    r,_=req;r['scene']['voxel_size_m']=bad;np.array([[3,3,3]],dtype='<f4').tofile(r['scene']['points_file'])
    with pytest.raises(ValueError,match='voxel'):planning.world_padding_metadata(r)
    result=planning.backend(r,tmp_path)
    assert result['status']=='error'


def test_resolutions_cannot_share_candidate_cache_even_with_same_scene_hash(req):
    r,_=req;ctx=dict(scene_sha256='same',state_sha256='same',calibration=str(common.DEFAULT_CALIBRATION))
    old=algorithms.pool_key(ctx,r,config.settings(),120)
    r['scene']['voxel_size_m']=.005
    assert algorithms.pool_key(ctx,r,config.settings(),120)!=old


def test_generator_timing_models_and_original_inputs_unchanged():
    before=json.loads((D/'before_hashes.json').read_text())
    allowed={'drum_planner/'+p for p in ('depth.py','planning.py','cpp/moveit_plan.cpp',
        'experiment/worker.py','experiment/algorithms.py','baseline/work.py','live_view.py')}
    for name,sha in before.items():
        if name not in allowed|OPTIMIZED and not name.startswith('planner_runs/'):
            assert common.digest(common.ROOT/name)==sha,name
    # Even in changed modules, generation and fusion logic stay byte-identical.
    import ast
    for name,functions in {'depth.py':['depth_window','deproject'], 'baseline/work.py':['calculate','camera_scene']}.items():
        old=(D/'before'/name).read_text();new=(common.ROOT/'drum_planner'/name).read_text()
        def bodies(text):return {n.name:ast.get_source_segment(text,n) for n in ast.parse(text).body if isinstance(n,ast.FunctionDef)}
        a,b=bodies(old),bodies(new)
        if name=='baseline/work.py':
            # Only the opt-in sparse-backend plumbing is new. With its default
            # removed, require the ENTIRE original acquisition body unchanged.
            b['calculate']=b['calculate'].replace(",timing_backend='adaptive'",'').replace(
                "        if timing_backend!='adaptive':options['timing_backend']=timing_backend\n",'')
        for f in functions:assert a[f]==b[f],(name,f)
