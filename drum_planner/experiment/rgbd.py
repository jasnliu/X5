"""Paired acquisition/scene reader for experiments and normal right hardware.

Honest RGB-enabled metadata uses a separate schema, not forged depth-only flags.
Depth fusion, occupied voxels, margins and visibility policy remain unchanged.
"""
import json,subprocess,time
from pathlib import Path
import numpy as np
from ..common import ROOT,atomic_json,calibration,digest,read_state

SCHEMA='x5-experiment-paired-depth-scene-v1'
CAMERA_PYTHON=ROOT.parent/'dt1/.venv/bin/python'
PAYLOADS={'raw_depth.npz','depth_m.bin','free_depth_m.bin','points_base.f32','pair.npz'}


def paired(path):
    return json.loads(Path(path).read_text()).get('schema')==SCHEMA


def read_pair(path,cal_path):
    from cymbal_destination.inputs import validate_pair
    cal=calibration(cal_path)
    with np.load(path,allow_pickle=False) as data:
        raw=data['depth'];color=data['color'];header=json.loads(str(data['header']));meta=json.loads(str(data['meta']))
        if json.loads(str(data['calibration']))!=cal:raise ValueError('Paired snapshot calibration differs')
    validate_pair(raw,color,header,meta,cal)
    return raw,header,meta


def headers(values):
    """Explicit bridge field aliases, preserving all original timestamps."""
    result=[dict(h,value_scale_m=h['scale_m'],received_monotonic=h['received']) for h in values]
    stamps=np.array([h['received_monotonic'] for h in result],float)
    sequences=np.array([h['sequence'] for h in result],float)
    if (len(result)!=5 or not np.isfinite(stamps).all() or not np.isfinite(sequences).all()
            or np.any(np.diff(stamps)<=0) or np.max(np.diff(stamps))>.25 or np.any(np.diff(sequences)<=0)):
        raise ValueError('Missing or discontinuous paired planning frames')
    for key in ('depth_timestamp_us','color_timestamp_us'):
        stamps=np.array([h[key] for h in result],float)
        if not np.isfinite(stamps).all() or np.any(stamps<=0) or np.any(np.diff(stamps)<=0):
            raise ValueError('Repeated or invalid paired sensor timestamps')
    if any(abs(h['depth_timestamp_us']-h['color_timestamp_us'])>40000 for h in result):
        raise ValueError('Unpaired color/depth window')
    if any(not np.isfinite(h['scale_m']) or not 1e-5<=h['scale_m']<=.01 for h in result):
        raise ValueError('Invalid paired depth scale')
    return result


def capture(camera_folder,cal_path,worker=None,*,stationary_joints=None):
    """Optional query-only observer is owned by acquisition.py, not this module."""
    from ..depth import snapshot
    folder=Path(camera_folder);folder.mkdir(parents=True,exist_ok=False)
    cal=calibration(cal_path)
    if not CAMERA_PYTHON.is_file():raise ValueError('Missing DT1 camera environment: '+str(CAMERA_PYTHON))
    began=time.monotonic()
    if worker:
        while worker.count<15 or time.monotonic()-began<.8:
            if worker.error:raise RuntimeError(worker.error)
            if time.monotonic()-began>4:raise RuntimeError('No complete disabled-motor encoder stream')
            time.sleep(.01)
    with (folder/'paired_worker.log').open('wb') as log:
        # Keep the existing planner busy-device diagnostics and familiar log
        # entrypoint; this is an alias of the paired bridge's actual stderr.
        (folder/'depth_stderr.log').symlink_to('camera_stderr.log')
        child=subprocess.Popen([str(CAMERA_PYTHON),'-B',str(Path(__file__).with_name('rgbd_camera.py')),
            '--folder',str(folder),'--calibration',str(cal_path)],stdout=log,stderr=subprocess.STDOUT)
        try:
            began=time.monotonic()
            while child.poll() is None:
                if worker and worker.error:raise RuntimeError(worker.error)
                if time.monotonic()-began>30:raise TimeoutError('Paired camera child timed out')
                time.sleep(.01)
            if child.returncode:
                error=folder/'error.json'
                detail=json.loads(error.read_text())['error'] if error.exists() else 'see '+str(folder/'paired_worker.log')
                from ..camera_diagnostics import failure_message
                raise RuntimeError('Paired camera capture failed: '+failure_message(detail,folder,cal['camera']['serial']))
        finally:
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)
    meta=json.loads((folder/'paired_capture.json').read_text())
    if meta.get('camera_released') is not True:raise ValueError('Paired camera not released')
    hs=headers(meta['headers']);raw,h,camera=read_pair(folder/'pair.npz',cal_path)
    with np.load(folder/'paired_depths.npz',allow_pickle=False) as data:arrays=data['depth']
    if arrays.shape!=(5,800,1280) or arrays.dtype!=np.uint16 or not np.array_equal(raw,arrays[-1]):
        raise ValueError('Paired target must be the final depth of the planning window')
    if h!=meta['headers'][-1] or camera!=meta['camera'] or meta['calibration_sha256']!=digest(cal_path):
        raise ValueError('Paired capture identity mismatch')
    if worker:
        from ..read_only import select_stationary
        from ..hardware_check import measured_state
        while time.monotonic()<hs[-1]['received_monotonic']+.35:
            if worker.error:raise RuntimeError(worker.error)
            time.sleep(.01)
        evidence=[];samples=worker.samples()
        for frame in hs:
            selected,proof=select_stationary(samples,frame['received_monotonic'],stationary_joints=stationary_joints)
            evidence.append(proof)
        state=measured_state(selected,evidence[-1],cal_path)
        state['synchronization'].update(all_five_frames=evidence[:-1],planning_frame_count=5)
        atomic_json(folder.parent/'measured_start.json',state);read_state(folder.parent/'measured_start.json')
    obj=snapshot(folder,camera,list(zip(hs,arrays)),cal_path)
    obj.update(schema=SCHEMA,dt1_pair_file='pair.npz',dt1_frame_sequence=h['sequence'],
        camera_released=True,scene_immutable=True,
        pose_synchronization='encoder_bracketed' if worker else 'saved_pose_no_CAN',
        acquisition='five paired native-depth frames; final frozen RGBD pair is the dt1 source')
    obj['files_sha256']['pair.npz']=digest(folder/'pair.npz')
    if worker:obj['measured_state_sha256']=digest(folder.parent/'measured_start.json')
    atomic_json(folder/'scene.json',obj)
    return obj


def scene(path,cal_path):
    """Dispatch old depth-only scenes unchanged; validate paired scenes locally."""
    from .. import planning
    path=Path(path).resolve();obj=json.loads(path.read_text())
    if obj.get('schema')!=SCHEMA:return planning.scene(path,cal_path)
    if obj.get('synthetic') is not False or obj['calibration_sha256']!=digest(cal_path):
        raise ValueError('Invalid experiment paired scene/calibration')
    if set(obj['files_sha256'])!=PAYLOADS:raise ValueError('Incomplete paired scene manifest')
    for name,sha in obj['files_sha256'].items():
        file=(path.parent/name).resolve()
        if not file.is_relative_to(path.parent) or digest(file)!=sha:raise ValueError('Paired scene payload changed: '+name)
    raw,h,meta=read_pair(path.parent/'pair.npz',cal_path)
    hs=headers(obj['frames'])
    if (meta!=obj['camera'] or obj.get('dt1_pair_file')!='pair.npz'
            or obj.get('dt1_frame_sequence')!=h['sequence'] or hs[-1]!=headers_last(h)):
        raise ValueError('DT1 pair does not match the planning scene')
    with np.load(path.parent/'raw_depth.npz',allow_pickle=False) as data:
        arrays=data['depth'];scales=data['scales_m']
    if (arrays.shape!=(5,800,1280) or arrays.dtype!=np.uint16 or not np.array_equal(arrays[-1],raw)
            or not np.array_equal(scales,[f['value_scale_m'] for f in hs])):
        raise ValueError('DT1 depth is not in the five-frame scene')
    fusion=obj['depth_fusion']
    if (fusion['method']!='nearest_temporally_confirmed_v1' or fusion['frame_count']!=5
            or fusion['minimum_support']!=3 or fusion['maximum_cluster_span_m']!=.02
            or fusion['spatial_filter'] is not False or obj['voxel_size_m']!=.005 or obj['obstacle_padding_m']<.02):
        raise ValueError('Paired scene changed the approved depth fusion or margins')
    if not np.allclose(obj['T_base_depth'],calibration(cal_path)['T_base_depth'],rtol=0,atol=1e-10):
        raise ValueError('Paired scene transform mismatch')
    for field,name,size in (('points_file','points_base.f32',obj['point_count']*12),
                            ('free_depth_file','free_depth_m.bin',1280*800*4)):
        if obj[field]!=name or (path.parent/name).stat().st_size!=size:raise ValueError('Invalid paired scene geometry')
        obj[field]=str(path.parent/name)
    if (path.parent/'depth_m.bin').stat().st_size!=1280*800*4:raise ValueError('Invalid paired scene depth')
    obj['source_scene_sha256']=digest(path)
    return obj


def headers_last(h):
    return dict(h,value_scale_m=h['scale_m'],received_monotonic=h['received'])


def load_view(plan,cal_path):
    from ..planned_view import load
    path=Path(plan.obj['provenance']['scene']).resolve()
    if not paired(path):return load(plan,cal_path)
    from ..snapshot_preview import prepare_scene
    obj=scene(path,cal_path)
    if (obj['source_scene_sha256']!=plan.req['scene']['source_scene_sha256']
            or digest(plan.obj['provenance']['state'])!=plan.obj['provenance']['state_sha256']):
        raise ValueError('Display inputs differ from the generated plan')
    return load_scene(path,cal_path)


def load_scene(path,cal_path):
    from ..planned_view import load_scene as legacy
    from ..snapshot_preview import prepare_scene
    path=Path(path).resolve()
    if not paired(path):return legacy(path,cal_path)
    obj=scene(path,cal_path)
    depth=np.fromfile(path.parent/'depth_m.bin',dtype='<f4').reshape(800,1280)
    if not np.isfinite(depth).all() or np.any((depth<0)|(depth>5)):raise ValueError('Invalid display depth')
    return prepare_scene(dict(obj['frames'][-1],value_scale_m=.0001),np.floor(depth*10000).astype('<u2'),
                         obj['camera'],obj['T_base_depth'])
