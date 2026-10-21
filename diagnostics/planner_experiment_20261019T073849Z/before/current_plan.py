"""Current measured pose + synchronized native depth -> MoveIt, never actuation.

Device ownership is retained through planning. Five frames are needed for the
conservative collision scene, unlike the one-frame display-only preview. Both
devices close before showing the frozen result. No long soak is required.
"""
import json
from pathlib import Path
import time

import numpy as np

from .common import DEFAULT_CALIBRATION, atomic_json, calibration, digest, read_state
from .depth import DepthCamera, snapshot
from .hardware_check import measured_state
from .read_only import Telemetry, select_stationary
from . import planning


def capture_current(folder, worker, cal_path=DEFAULT_CALIBRATION):
    """Acquire a short stationary, bracketed window without changing motor mode."""
    cal=calibration(cal_path);folder=Path(folder);camera=None
    began=time.monotonic()
    try:
        while worker.count<15 or time.monotonic()-began<.8:
            if worker.error:raise RuntimeError(worker.error)
            if time.monotonic()-began>4:raise RuntimeError('No complete disabled-motor encoder stream')
            time.sleep(.01)
        camera_folder=folder/'camera';camera_folder.mkdir()
        camera=DepthCamera(camera_folder,cal)
        start=time.monotonic()
        while len(camera.frames)<5:
            if worker.error:raise RuntimeError(worker.error)
            if camera.error:raise RuntimeError(camera.error)
            if time.monotonic()-start>15:raise RuntimeError('Five-frame depth startup timeout')
            time.sleep(.005)
        frames=list(camera.frames);meta=camera.meta
        stamps=np.array([h['received_monotonic'] for h,_ in frames])
        sequences=np.array([h['sequence'] for h,_ in frames])
        if (len(frames)!=5 or not np.isfinite(stamps).all() or np.any(np.diff(stamps)<=0)
                or np.max(np.diff(stamps))>.25 or np.any(np.diff(sequences)<=0)
                or not 0<=time.monotonic()-stamps[-1]<.5):
            raise RuntimeError('Missing, stale or discontinuous planning depth frames')
        camera.close()
        if camera.error:raise RuntimeError(camera.error)
        if camera.thread.is_alive() or (camera.process and camera.process.poll() is None):
            raise RuntimeError('Depth camera failed to release')
        while time.monotonic()<stamps[-1]+.35:
            if worker.error:raise RuntimeError(worker.error)
            time.sleep(.01)
        samples=worker.samples()
        evidence=[];selected=None
        for stamp in stamps:
            selected,proof=select_stationary(samples,float(stamp));evidence.append(proof)
        state=measured_state(selected,evidence[-1],cal_path)
        state['synchronization']['all_five_frames']=evidence[:-1]
        state['synchronization']['planning_frame_count']=5
        atomic_json(folder/'measured_start.json',state)
        read_state(folder/'measured_start.json')
        scene=snapshot(camera_folder,meta,frames,cal_path)
        scene.update(measured_state_sha256=digest(folder/'measured_start.json'),
                     acquisition='stationary encoder-bracketed five-frame planning capture')
        atomic_json(camera_folder/'scene.json',scene)
        return state,scene
    finally:
        if camera:camera.close()


def verify_unchanged(worker,state):
    if worker.error:raise RuntimeError(worker.error)
    samples=worker.samples()
    select_stationary(samples)
    if not -.01<=time.monotonic()-samples[-1]['end']<=.3:
        raise RuntimeError('Encoder feedback stale after planning')
    from .common import JOINTS,LEFT_JOINTS
    actual=np.array([samples[-1]['joints'][j] for j in JOINTS+LEFT_JOINTS])
    if np.max(abs(actual-np.array(state['right_rad']+state['left_rad'])))>np.radians(.35):
        raise RuntimeError('Arm changed pose while planning; discard route and recapture')
    return dict(checked_monotonic=time.monotonic(),maximum_difference_deg=float(np.degrees(
        np.max(abs(actual-np.array(state['right_rad']+state['left_rad']))))),
        note='Valid at acquisition/check time only, not ongoing feedback or motion authorization')


def run(folder,*,headless=False,seconds=0.,cal_path=DEFAULT_CALIBRATION,
        candidates=planning.DEFAULT_CANDIDATES):
    candidates=planning.candidate_count(candidates)
    if not np.isfinite(seconds) or seconds<0 or (headless and seconds):
        raise ValueError('Nonnegative --seconds is for the preview window only')
    calibration(cal_path)
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False)
    worker=None;result=None;error=None;proof=None;view=None
    try:
        print('CURRENT-POSE PLAN: both arms must be relaxed and stationary. State queries ONLY.',flush=True)
        worker=Telemetry(folder)
        state,scene=capture_current(folder,worker,cal_path)
        verify_unchanged(worker,state)
        print('Planning from measured right joints to record3 final pose against DETECTED OBSTACLES. '
              'Full arm and stick checked; unseen space does not block planning.',flush=True)
        result=planning.plan(folder/'plan',folder/'measured_start.json',folder/'camera/scene.json',cal_path,candidates=candidates)
        proof=verify_unchanged(worker,state)
        worker.close()
        if worker.error:raise RuntimeError(worker.error)
        result['actual_current_robot_state_verified']=True
        result['current_state_check']=proof
        result['provenance']['current_pose_acquisition']=str(folder)
        result['physical_execution_blockers']=[b for b in result['physical_execution_blockers']
            if not b.startswith('Actual resting')]
        atomic_json(folder/'plan/plan.json',result)
        print(f"{result['status']}: {folder/'plan/plan.json'}",flush=True)
        if result['status']!='planned':
            print(json.dumps(result.get('details',result.get('error')),indent=2),flush=True)
            from .diagnostics import blockage_summary
            print(blockage_summary(result),flush=True)
            if result.get('visibility_coverage'):
                print('Per-link coverage evidence: '+str(folder/'plan/plan.json')+' [visibility_coverage]',flush=True)
        if not headless:
            from .planned_view import show
            view=show(folder/'plan/plan.json',seconds=seconds,cal_path=cal_path)
        return folder/'plan/plan.json'
    except BaseException as exc:
        error=str(exc) or type(exc).__name__
        if result is not None and proof is None:
            result.update(status='invalidated_acquisition',acquisition_error=error,
                          actual_current_robot_state_verified=False)
            atomic_json(folder/'plan/plan.json',result)
        raise
    finally:
        if worker:worker.close()
        atomic_json(folder/'current_plan_report.json',dict(schema='x5-current-pose-plan-report-v1',
            passed=error is None and result is not None and result['status']=='planned',error=error,
            planning_status=None if result is None else result['status'],
            motor_commands=0,mode_changes=0,gripper_commands=0,relax_commands=0,
            current_state_check=proof,telemetry=None if worker is None else worker.report(),
            scene_source='five synchronized native-depth frames; no manual obstacles',
            visualization=view,motion_authorized=False))
