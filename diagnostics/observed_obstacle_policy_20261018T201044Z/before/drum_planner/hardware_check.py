"""Read-only hardware preflight: measured pose + optional frozen depth scene.

This deliberately has NO execution/enable/relax path. Both CAN sockets close
before mesh generation or GUI display. Every output remains non-executable.
"""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from .common import (ROOT,JOINTS,LEFT_JOINTS,DEFAULT_CALIBRATION,atomic_json,
                     calibration,digest,read_state)
from .depth import DepthCamera
from .model import generate
from .read_only import Telemetry,select_stationary
from safe_zone.geometry import Model


BLOCKERS=[
    'Read-only stage: motor execution is unavailable in this command; separately armed live execution requires commissioning',
    'Visual/physical verification of depth alignment and provisional custom gripper/stick mounting is still required',
    'Left gripper opening is an encoder-derived model estimate; left stick geometry remains unverified',
    'Resting table contact, support and camera-blind regions are not verified safe',
    'A collision-checked route from the measured start against a real planning scene is still required',
    'Feedback tracking, stall handling and a checked return/support policy are required before motion',
]


def measured_state(sample,evidence,cal_path):
    return dict(schema='x5-planner-start-state-v1',coordinate_convention='encoder_to_joint',
        right_rad=[sample['joints'][name] for name in JOINTS],
        left_rad=[sample['joints'][name] for name in LEFT_JOINTS],
        left_gripper_opening_m=sample['joints']['openarmx_left_finger_joint1'],
        left_gripper_opening_source='legacy encoder-derived display estimate; not a measured jaw width',
        right_gripper_closed_motor_joint_rad=float(np.radians(7)),
        right_gripper_closed_field_meaning='future configured target ONLY; no gripper command was sent',
        motor8_feedback=sample['motor8_feedback'],motor_telemetry=sample['feedback'],
        preview_only=True,measured=True,source='Read-only disabled-motor CAN encoder snapshot of both arms',
        captured_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        measurement_start_monotonic=sample['start'],measurement_end_monotonic=sample['end'],
        model_sha256=digest(ROOT/'model/openarmx.urdf'),encoder_source_sha256=digest(ROOT/'safe_zone/encoder.py'),
        calibration_sha256=digest(cal_path),synchronization=evidence,
        motor_commands=0,gripper_commands=0,motion_authorized=False,hardware_execution_available=False,
        supported_rest_verified=False,gripper_closed_physically_verified=False)


def limit_report(state,urdf):
    root=ET.parse(urdf).getroot();violations=[]
    for name,value in zip(JOINTS+LEFT_JOINTS,state['right_rad']+state['left_rad']):
        bounds=root.find(f"joint[@name='{name}']/limit")
        lo=float(bounds.get('lower'));hi=float(bounds.get('upper'))
        if not lo<=value<=hi:violations.append(dict(joint=name,measured_rad=value,lower_rad=lo,upper_rad=hi))
    return dict(within_urdf_limits=not violations,violations=violations,angles_clamped=False)


def landmarks(state,urdf,manifest):
    values=dict(zip(JOINTS+LEFT_JOINTS,state['right_rad']+state['left_rad']))
    values['openarmx_left_finger_joint1']=state['left_gripper_opening_m']
    frames=Model(urdf).transforms(values)
    points={f'{side}_joint{i}':frames[f'openarmx_{side}_link{i}'][:3,3]
            for side in ('right','left') for i in range(8)}
    for side in ('right','left'):points[side+'_hand']=frames[f'openarmx_{side}_hand'][:3,3]
    T=frames['right_drumstick'];half=manifest['stick_length_from_mount_m']/2
    points['right_stick_mount']=T[:3,3]-half*T[:3,2];points['right_stick_tip']=T[:3,3]+half*T[:3,2]
    return points


def coverage_report(points,meta,transform,raw,scale):
    T=np.asarray(transform);K=np.array(meta['depth']['K']);height,width=raw.shape;out={}
    for name,point in points.items():
        p=(np.asarray(point)-T[:3,3])@T[:3,:3]
        uv=None if p[2]<=0 else (K@p)[:2]/p[2]
        inside=uv is not None and 0<=uv[0]<width and 0<=uv[1]<height
        observed=None
        if inside:
            x,y=np.floor(uv).astype(int);value=int(raw[y,x]);z=value*scale
            if value not in (0,65535) and .15<=z<=5:observed=z
        out[name]=dict(base_xyz_m=np.asarray(point).tolist(),camera_xyz_m=p.tolist(),
                       pixel_uv=None if uv is None else uv.tolist(),inside_depth_image=bool(inside),
                       valid_depth_at_projected_pixel=observed is not None,observed_surface_depth_m=observed)
    return dict(landmarks=out,alignment_verified=False,clearance_verified=False,
                note='Projected FK landmarks only, not detected arm features. In-image does NOT mean visible/unoccluded or clear. No alignment error is inferred from interior joint centers.')


def run(folder,*,query_seconds=3.,seconds=0.,headless=False,no_camera=False,
        cal_path=DEFAULT_CALIBRATION):
    if not np.isfinite(query_seconds) or not 1.<=query_seconds<=600:
        raise ValueError('--query-seconds must be 1–600 seconds')
    if not np.isfinite(seconds) or seconds<0:raise ValueError('--seconds must be finite and nonnegative')
    if no_camera and not headless:raise ValueError('--no-camera requires --headless; alignment needs depth')
    if headless and seconds:raise ValueError('Use --query-seconds for headless acquisition duration')
    cal=calibration(cal_path)
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False)
    manifest=generate(folder/'model',cal_path)
    worker=None;camera=None;error=None;success=False;scene=None;state=None;coverage=None;limits=None
    display=None;camera_closed=False;encoder_closed=False
    print('READ-ONLY CHECK: querying both disabled arms. No torque, motion, gripper or relax commands.',flush=True)
    try:
        worker=Telemetry(folder)
        began=time.monotonic()
        # Verify disabled-motor feedback before starting the camera. Motors that
        # are enabled cause a stop; we never try to disable them ourselves.
        while worker.count<15:
            if worker.error:raise RuntimeError(worker.error)
            if time.monotonic()-began>4:raise RuntimeError('No complete disabled-motor encoder stream')
            time.sleep(.01)
        if not no_camera:
            scene=folder/'camera';scene.mkdir()
            camera=DepthCamera(scene,cal,single_frame=True)
            start_camera=time.monotonic()
            while camera.latest is None:
                if worker.error:raise RuntimeError(worker.error)
                if camera.error:raise RuntimeError(camera.error)
                if time.monotonic()-start_camera>15:raise RuntimeError('Initial depth camera startup timeout')
                time.sleep(.005)
            header,raw=camera.latest
            if not -.01<=time.monotonic()-header['received_monotonic']<=1.:
                raise RuntimeError('Initial camera frame is not fresh')
            camera.close()
            if camera.error:raise RuntimeError(camera.error)
            camera_closed=not camera.thread.is_alive() and camera.process.poll()==0
            if not camera_closed or camera.count!=1:raise RuntimeError('Single-frame camera did not shut down cleanly')
        # Retain fresh samples on both sides of the frame, even for a very fast
        # camera/short requested run. Do not estimate a pose from stale feedback.
        target_end=began+query_seconds
        if camera:target_end=max(target_end,header['received_monotonic']+.35)
        while time.monotonic()<target_end:
            if worker.error:raise RuntimeError(worker.error)
            time.sleep(.01)
        worker.close();encoder_closed=True
        if worker.error:raise RuntimeError(worker.error)
        samples=worker.samples()
        selected,evidence=select_stationary(samples,None if camera is None else header['received_monotonic'])
        if not -.01<=time.monotonic()-samples[-1]['end']<=.5:raise RuntimeError('Final encoder batch is stale')
        state=measured_state(selected,evidence,cal_path)
        atomic_json(folder/'measured_start.json',state)
        # Validate against the same existing state schema used by MoveIt input;
        # do not add offsets or normalize/clamp measured joint angles here.
        read_state(folder/'measured_start.json')
        limits=limit_report(state,folder/'model/robot.urdf')
        if camera:
            from .snapshot_preview import prepare_scene,save_scene
            _,mesh=prepare_scene(header,raw,camera.meta,cal['T_base_depth'])
            save_scene(scene,header,raw,camera.meta,cal,cal_path,mesh)
            camera_scene=json.loads((scene/'scene.json').read_text())
            camera_scene.update(can_access=True,can_operation='separate read-only encoder acquisition; see parent audit',
                                read_only_joint_measurement=True)
            atomic_json(scene/'scene.json',camera_scene)
            coverage=coverage_report(landmarks(state,folder/'model/robot.urdf',manifest),
                                     camera.meta,cal['T_base_depth'],raw,header['value_scale_m'])
            atomic_json(folder/'coverage.json',coverage)
        files=['measured_start.json','encoder_batches.jsonl','can_query_audit.jsonl','can_rx_audit.jsonl',
               'model/robot.urdf','model/robot.srdf','model/model_manifest.json']
        if camera:files+=['coverage.json','camera/scene.json','camera/raw_depth.npz','camera/surface_mesh.npz']
        atomic_json(folder/'snapshot_manifest.json',dict(schema='x5-readonly-hardware-snapshot-v1',
                    calibration_sha256=digest(cal_path),camera_present=camera is not None,
                    files_sha256={name:digest(folder/name) for name in files},
                    motion_authorized=False,hardware_execution_available=False))
        success=True
        print('Measured start saved: '+str(folder/'measured_start.json'),flush=True)
        print('CAN sockets closed; '+('camera released.' if camera else 'no camera requested.')+
              ' Motion remains unavailable.',flush=True)
        # Viewer does not even have an observer/camera reference. Replay uses
        # validated saved files only, exactly like the offline hardware-view CLI.
        if not headless:
            from .alignment_view import show
            display=show(folder,seconds=seconds,cal_path=cal_path)
    except BaseException as exc:
        error=str(exc) or type(exc).__name__;raise
    finally:
        cleanup_errors=[]
        for name,resource in [('camera',camera),('encoder',worker)]:
            if resource:
                try:resource.close()
                except BaseException as exc:cleanup_errors.append(name+': '+str(exc))
        encoder_closed=bool(worker and not worker.thread.is_alive())
        camera_closed=bool(camera and not camera.thread.is_alive() and
                           camera.process and camera.process.poll() is not None)
        cleanup_only_error=bool(cleanup_errors and error is None)
        if cleanup_only_error:error='Cleanup failed: '+'; '.join(cleanup_errors)
        atomic_json(folder/'preflight_report.json',dict(schema='x5-readonly-preflight-report-v1',
            acquisition_passed=success,passed=success and error is None,error=error,cleanup_errors=cleanup_errors,
            execution_ready=False,motion_authorized=False,hardware_execution_available=False,
            can_access=worker is not None,can_operation='state queries only on can0 and can1',
            mode_changes=0,motor_commands=0,gripper_commands=0,relax_commands=0,
            encoder_sockets_closed=encoder_closed,telemetry=None if worker is None else worker.report(),
            camera_requested=not no_camera,camera_frames=0 if camera is None else camera.count,
            camera_released=camera_closed,camera_exit_code=None if not camera or not camera.process else camera.process.poll(),
            joint_limits=limits,coverage=coverage,visualization=display,
            physical_alignment_verified=False,tool_mount_verified=False,supported_rest_verified=False,
            execution_blockers=BLOCKERS))
        if cleanup_only_error:raise RuntimeError(error)
    return folder/'preflight_report.json'


def load_snapshot(folder,cal_path=DEFAULT_CALIBRATION):
    """Offline integrity checks before drawing a saved measured pose."""
    folder=Path(folder).resolve();cal=calibration(cal_path)
    manifest=json.loads((folder/'snapshot_manifest.json').read_text())
    if (manifest.get('schema')!='x5-readonly-hardware-snapshot-v1' or
            manifest.get('motion_authorized') is not False or manifest.get('hardware_execution_available') is not False):
        raise ValueError('Expected a non-executable measured snapshot')
    if manifest['calibration_sha256']!=digest(cal_path):raise ValueError('Snapshot calibration changed')
    required={'measured_start.json','model/robot.urdf','model/robot.srdf','model/model_manifest.json'}
    if manifest.get('camera_present'):required|={'camera/scene.json','camera/raw_depth.npz','camera/surface_mesh.npz','coverage.json'}
    if not required<=manifest['files_sha256'].keys():raise ValueError('Incomplete measured snapshot manifest')
    for name,sha in manifest['files_sha256'].items():
        p=(folder/name).resolve()
        if not p.is_relative_to(folder) or not p.is_file() or digest(p)!=sha:raise ValueError('Measured snapshot file path/hash mismatch: '+name)
    state=read_state(folder/'measured_start.json')
    if (state.get('measured') is not True or state.get('preview_only') is not True or
            state.get('model_sha256')!=digest(ROOT/'model/openarmx.urdf') or
            state.get('encoder_source_sha256')!=digest(ROOT/'safe_zone/encoder.py') or
            state.get('calibration_sha256')!=digest(cal_path) or
            state.get('motion_authorized') is not False or state.get('hardware_execution_available') is not False):
        raise ValueError('Incompatible measured-state provenance')
    model=json.loads((folder/'model/model_manifest.json').read_text())
    if model['calibrated_urdf_sha256']!=digest(folder/'model/robot.urdf') or model['calibration_sha256']!=digest(cal_path):
        raise ValueError('Measured-state model mismatch')
    return folder,cal,state,model,manifest
