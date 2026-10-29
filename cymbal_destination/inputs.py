"""Hash-bound, honest single-pair scene adapter; no five-frame claims."""
import json
from pathlib import Path
import numpy as np
from drum_planner.common import atomic_json,calibration,digest,read_state
from drum_planner.depth import deproject,occupied_voxels,VOXEL_SIZE_M

PAYLOADS=('pair.npz','target.json','depth_m.bin','points_base.f32','display.npz','rgb.png')


def validate_pair(raw,color,header,meta,cal):
    """Recheck DT1's native paired contract in the isolated X5 environment."""
    expected=cal['camera']
    if (meta.get('protocol')!=1 or meta.get('alignment')!='native_depth' or
        meta.get('frame_sync') is not True or any(meta[k]!=expected[k] for k in ('serial','firmware','depth_work_mode'))):
        raise ValueError('Invalid paired camera identity')
    for name,size in (('depth',(1280,800)),('color',(1280,720))):
        if (meta[name]['width'],meta[name]['height'])!=size:raise ValueError('Cropped camera profile')
        for k in ('K','D_opencv_order','distortion_model','intrinsic_width','intrinsic_height'):
            if not np.allclose(meta[name][k],expected[name][k],rtol=0,atol=1e-6):raise ValueError('Camera geometry mismatch')
    if meta['depth']['format_enum']!=8 or meta['depth']['fps']!=10 or meta.get('encoding') not in ('mjpeg','yuyv'):
        raise ValueError('Unexpected native camera stream')
    if meta['color']['format_enum']!={'mjpeg':5,'yuyv':0}[meta['encoding']]:raise ValueError('RGB encoding mismatch')
    if not np.allclose(meta['T_color_depth_m'],expected['T_color_depth_m'],rtol=0,atol=1e-7):
        raise ValueError('Factory RGBD transform changed')
    if raw.shape!=(800,1280) or raw.dtype!=np.uint16 or color.shape!=(720,1280,3) or color.dtype!=np.uint8:
        raise ValueError('Invalid native paired images')
    timestamps=np.array([header['depth_timestamp_us'],header['color_timestamp_us']],float)
    if not np.isfinite(timestamps).all() or np.any(timestamps<0) or abs(timestamps[0]-timestamps[1])>40000:
        raise ValueError('Unpaired RGB/depth timestamps')
    scale=float(header['scale_m'])
    if not np.isfinite(scale) or not 1e-5<=scale<=.01:raise ValueError('Invalid depth scale')
    return scale


def target(path,cal_path):
    obj=json.loads(Path(path).read_text())
    if obj.get('schema')!='x5-cymbal-target-v1' or obj.get('calibration_sha256')!=digest(cal_path):
        raise ValueError('Target/calibration identity mismatch')
    if obj.get('valid') is not True:raise ValueError('DT1 has no valid green target: '+str(obj.get('target',{}).get('status')))
    red=np.asarray(obj['target']['world_m'],float);green=np.asarray(obj['above_target']['world_m'],float)
    normal=np.asarray(obj['flat_disc']['normal_world'],float)
    if any(a.shape!=(3,) or not np.isfinite(a).all() for a in (red,green,normal)):
        raise ValueError('Nonfinite/invalid target geometry')
    if abs(np.linalg.norm(normal)-1)>1e-8 or normal[2]<0:
        raise ValueError('Disc normal must be unit and upward')
    if not np.allclose(green,red+.0762*normal,rtol=0,atol=1e-10):
        raise ValueError('Green target must be the unmodified DT1 disc-normal offset')
    if obj['above_target'].get('direction')!='estimated_disc_upward_normal':
        raise ValueError('No vertical-offset or stale-normal fallback')
    disc=obj['flat_disc'];center=np.asarray(disc['center_world_m'],float)
    if center.shape!=(3,) or not np.isfinite(center).all() or not np.isfinite(disc['radius_m']) or disc['radius_m']<=0:
        raise ValueError('Invalid fitted disc')
    return obj,green,normal


def build_scene(folder,cal_path):
    folder=Path(folder);camera=folder/'camera';cal=calibration(cal_path)
    with np.load(camera/'pair.npz',allow_pickle=False) as a:
        raw=a['depth'];color=a['color'];header=json.loads(str(a['header']));meta=json.loads(str(a['meta']))
        if json.loads(str(a['calibration']))!=cal:raise ValueError('Pair calibration differs')
    # RGBD metadata was checked by DT1; independently check geometry here without
    # falsifying stream flags to pass X5's depth-only acquisition validator.
    scale=validate_pair(raw,color,header,meta,cal)
    depth=raw.astype(np.float32)*scale
    depth[(raw==0)|(raw==65535)|(depth<.15)|(depth>5.)]=0
    if np.count_nonzero(depth)<1024:raise ValueError('Insufficient observed scene depth')
    points=occupied_voxels(deproject(depth,meta,cal['T_base_depth']),VOXEL_SIZE_M)
    depth.astype('<f4').tofile(camera/'depth_m.bin')
    points.astype('<f4').tofile(camera/'points_base.f32')
    obj=dict(schema='x5-cymbal-single-pair-scene-v1',synthetic=False,camera=meta,
        calibration_sha256=digest(cal_path),T_base_depth=cal['T_base_depth'],
        points_file='points_base.f32',point_count=len(points),free_depth_file='depth_m.bin',
        voxel_size_m=VOXEL_SIZE_M,obstacle_padding_m=.02,frames=[header],
        depth_fusion=dict(method='none_single_paired_frame',frame_count=1,temporal_confirmation=False,
            spatial_filter=False,holes_filled=False),
        default_collision_policy='observed_obstacles',unknown_space_policy='unobserved is not certified clear',
        files_sha256={n:digest(camera/n) for n in PAYLOADS},
        measured_state_sha256=digest(folder/'measured_start.json'),motion_authorized=False)
    atomic_json(camera/'scene.json',obj)
    return obj


def load(folder,cal_path):
    folder=Path(folder).resolve();p=folder/'camera/scene.json';world=json.loads(p.read_text())
    if world.get('schema')!='x5-cymbal-single-pair-scene-v1' or world.get('synthetic') is not False:
        raise ValueError('Expected actual single-pair scene')
    if world['calibration_sha256']!=digest(cal_path):raise ValueError('Scene/calibration mismatch')
    if set(world['files_sha256'])!=set(PAYLOADS):raise ValueError('Incomplete snapshot manifest')
    if world['measured_state_sha256']!=digest(folder/'measured_start.json'):raise ValueError('Captured state changed')
    for name,sha in world['files_sha256'].items():
        file=(p.parent/name).resolve()
        if not file.is_relative_to(p.parent) or digest(file)!=sha:raise ValueError('Snapshot payload changed: '+name)
    world['source_scene_sha256']=digest(p)
    for key,name in (('points_file','points_base.f32'),('free_depth_file','depth_m.bin')):
        if world[key]!=name:raise ValueError('Scene geometry must use its verified local payload')
        world[key]=str((p.parent/name).resolve())
    state=read_state(folder/'measured_start.json')
    if state.get('calibration_sha256')!=digest(cal_path):raise ValueError('State/calibration mismatch')
    obj,green,normal=target(p.parent/'target.json',cal_path)
    return state,world,obj,green,normal
