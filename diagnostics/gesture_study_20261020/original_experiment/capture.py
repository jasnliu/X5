"""Initial full-FOV five-frame scene, then camera OFF. No encoder access."""
import time
from pathlib import Path
import numpy as np
from ..common import calibration,atomic_json
from ..depth import DepthCamera,snapshot


def capture(folder,cal_path):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    camera=DepthCamera(folder,calibration(cal_path));began=time.monotonic()
    print('CAMERA: capturing the initial full-FOV five-frame scene (no CAN).',flush=True)
    try:
        while len(camera.frames)<5:
            if camera.error:raise RuntimeError(camera.error)
            if time.monotonic()-began>15:raise TimeoutError('Camera initial scene timeout')
            time.sleep(.005)
        frames=list(camera.frames);meta=camera.meta
        stamps=np.array([h['received_monotonic'] for h,_ in frames]);seq=[h['sequence'] for h,_ in frames]
        if np.any(np.diff(stamps)<=0) or np.max(np.diff(stamps))>.25 or np.any(np.diff(seq)<=0) or time.monotonic()-stamps[-1]>.5:
            raise ValueError('Camera frames are stale or discontinuous')
    finally:camera.close()
    if camera.error or camera.thread.is_alive() or (camera.process and camera.process.poll() is None):
        raise RuntimeError(camera.error or 'Camera failed to release')
    result=snapshot(folder,meta,frames,cal_path)
    result.update(acquisition='Software experiment: five-frame frozen scene; saved arm poses, NOT synchronized live encoders',
                  can_access=False,camera_released=True,scene_immutable=True)
    atomic_json(folder/'scene.json',result)
    print(f'CAMERA OFF: {result["point_count"]:,} occupied voxels saved to {folder}',flush=True)
    return folder/'scene.json'
