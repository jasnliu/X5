"""Camera-only capture and conservative native-depth scene snapshots."""
from collections import deque
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import cv2
import numpy as np

from camera_calibration.io import SDK
from camera_calibration.core import checked_transform, digest
from .common import ROOT, atomic_json, calibration, DEFAULT_CALIBRATION

BUILD = ROOT / 'drum_planner/build'


def build_depth():
    source = ROOT / 'drum_planner/cpp/depth_capture.cpp'
    BUILD.mkdir(parents=True, exist_ok=True)
    binary = BUILD / 'depth_capture'; stamp = BUILD / 'depth_signature.txt'
    signature = digest(source) + str(SDK.resolve())
    if binary.is_file() and stamp.is_file() and stamp.read_text() == signature:
        return binary
    temp = binary.with_suffix('.building')
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', str(source),
                    '-I'+str(SDK/'include'), '-L'+str(SDK/'lib'), '-Wl,-rpath,'+str(SDK/'lib'),
                    '-lOrbbecSDK', '-pthread', '-o', str(temp)], check=True, timeout=120)
    os.replace(temp, binary); stamp.write_text(signature)
    return binary


def check_metadata(meta, cal):
    if (meta.get('protocol') != 1 or meta.get('serial') != cal['camera']['serial']
            or meta.get('alignment') != 'native_depth' or not meta.get('depth_stream_enabled')
            or meta.get('color_stream_enabled')):
        raise ValueError('Wrong camera identity, native-depth format or stream protocol')
    for key in ('firmware', 'depth_work_mode'):
        if meta[key] != cal['camera'][key]: raise ValueError('Camera ' + key + ' changed since calibration')
    d = meta['depth']
    if (d['width'], d['height'], d['fps'], d['format_enum']) != (1280, 800, 10, 8):
        raise ValueError('Require full-FOV 1280x800 Y16 depth at 10 FPS, no cropped fallback')
    for stream in ('color', 'depth'):
        for key in ('K', 'D_opencv_order', 'intrinsic_width', 'intrinsic_height', 'distortion_model'):
            if not np.allclose(meta[stream][key], cal['camera'][stream][key], atol=1e-6, rtol=0):
                raise ValueError('Active camera parameters differ from calibration: ' + stream + '/' + key)
    if not np.allclose(checked_transform(meta['T_color_depth_m']), cal['camera']['T_color_depth_m'], atol=1e-7):
        raise ValueError('Factory depth-to-color transform changed')
    if not np.allclose(d['D_opencv_order'], 0):
        raise ValueError('This native-depth deprojection requires verified zero distortion')


def decode(header, data):
    if header.get('n') != 1280*800*2 or len(data) != 1280*800*2:
        raise ValueError('Incomplete Y16 depth frame')
    scale = float(header['value_scale_m'])
    if not np.isfinite(scale) or not 1e-5 <= scale <= .01:
        raise ValueError('Invalid SDK depth scale')
    return np.frombuffer(data, dtype='<u2').reshape(800, 1280).copy()


class DepthCamera:
    def __init__(self, folder, cal):
        self.folder = Path(folder); self.cal = cal
        self.meta = None; self.error = None; self.frames = deque(maxlen=5)
        self.latest = None; self.count = 0; self.maximum_gap_s = 0.; self.start = time.monotonic()
        self.stop = threading.Event(); self.process = None
        self.thread = threading.Thread(target=self._run, daemon=True); self.thread.start()

    def _run(self):
        try:
            binary = build_depth()
            with (self.folder/'depth_stderr.log').open('wb') as log:
                self.process = subprocess.Popen([str(binary), self.cal['camera']['serial']], cwd=self.folder,
                                                stdout=subprocess.PIPE, stderr=log)
                stream = self.process.stdout
                def line():
                    text = stream.readline(65537)
                    if not text or len(text)>65536 or not text.endswith(b'\n'):
                        raise ValueError('Depth stream ended or header incomplete; see depth_stderr.log')
                    return json.loads(text)
                self.meta = line(); check_metadata(self.meta, self.cal)
                previous = None
                while not self.stop.is_set():
                    header = line()
                    if header.get('n') != 1280*800*2: raise ValueError('Unexpected depth frame size')
                    frame = decode(header, stream.read(header['n']))
                    now = time.monotonic()
                    if not -.01 <= now-header['received_monotonic'] <= .5:
                        raise ValueError('Stale depth frame/host clock mismatch')
                    if previous:
                        if (header['sequence'] <= previous['sequence']
                                or header['device_timestamp_us'] <= previous['device_timestamp_us']):
                            raise ValueError('Repeated/backwards depth frame')
                        self.maximum_gap_s = max(self.maximum_gap_s, header['received_monotonic']-previous['received_monotonic'])
                    previous=header; self.latest=(header, frame); self.frames.append((header, frame)); self.count += 1
        except Exception as e:
            if not self.stop.is_set(): self.error = str(e)
        finally:
            self._terminate()

    def _terminate(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try: self.process.wait(timeout=3)
            except subprocess.TimeoutExpired: self.process.kill(); self.process.wait(timeout=3)

    def close(self):
        self.stop.set(); self._terminate(); self.thread.join(timeout=5)


def snapshot(folder, meta, frames, cal_path):
    if len(frames) < 5: raise ValueError('Need five complete depth frames')
    folder = Path(folder)
    arrays = np.stack([d for _, d in frames])
    scales = np.array([h['value_scale_m'] for h, _ in frames])
    depths = arrays.astype(np.float32) * scales[:, None, None]
    # Nearest valid return across the window, never median-filter away a thin
    # stick/cymbal edge. Invalid/too-near pixels stay unknown, not clear space.
    good = (arrays > 0) & (arrays < 65535) & (depths >= .15) & (depths <= 5.)
    minimum = np.min(np.where(good, depths, np.inf), axis=0)
    minimum[~np.isfinite(minimum)] = 0.
    if np.count_nonzero(minimum)<1024:
        raise ValueError('Too few valid depth returns; refusing an empty or nearly empty scene')
    # Free-space visibility is stricter: every retained frame must have a return.
    free_depth = np.where(np.all(good, axis=0), minimum, 0.).astype('<f4')
    minimum.astype('<f4').tofile(folder/'depth_m.bin')
    free_depth.tofile(folder/'free_depth_m.bin')
    np.savez_compressed(folder/'raw_depth.npz', depth=arrays, scales_m=scales)
    cal = calibration(cal_path)
    K=np.array(meta['depth']['K']); y,x=np.nonzero(minimum)
    z=minimum[y,x]
    points=np.c_[(x-K[0,2])*z/K[0,0], (y-K[1,2])*z/K[1,1], z]
    T=np.array(cal['T_base_depth']); points=points@T[:3,:3].T+T[:3,3]
    # Retain all returns; no manually drawn drum/cymbal boxes or fitted planes.
    # Voxel centers represent all raw points with half-diagonal uncertainty.
    voxel=.01
    cells=np.unique(np.floor(points/voxel).astype(np.int32), axis=0)
    cloud=(cells.astype(float)+.5)*voxel
    cloud.astype('<f4').tofile(folder/'points_base.f32')
    obj=dict(schema='x5-native-depth-scene-v1', synthetic=False,
             camera=meta, calibration_sha256=digest(cal_path), T_base_depth=T.tolist(),
             voxel_size_m=voxel, obstacle_padding_m=.02, point_count=len(cloud),
             points_file='points_base.f32', free_depth_file='free_depth_m.bin',
             frames=[h for h,_ in frames], captured_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
             observed_valid_fraction=float(np.mean(minimum>0)), fully_observed_fraction=float(np.mean(free_depth>0)),
             unknown_space_policy='blocked by default; occupied-only option is explicitly preview-only',
             files_sha256={n:digest(folder/n) for n in ['raw_depth.npz','depth_m.bin','free_depth_m.bin','points_base.f32']},
             physical_depth_alignment_verified=False, motion_authorized=False)
    atomic_json(folder/'scene.json',obj)
    colored=cv2.applyColorMap(np.uint8(np.clip(minimum/3.,0,1)*255),cv2.COLORMAP_TURBO)
    colored[minimum==0]=0
    cv2.imwrite(str(folder/'depth_preview.png'),colored)
    return obj


def capture(folder, seconds=0, headless=False, cal_path=DEFAULT_CALIBRATION):
    folder=Path(folder).resolve(); folder.mkdir(parents=True,exist_ok=False)
    if headless and seconds <= 0: raise ValueError('Headless capture needs a positive duration')
    cal=calibration(cal_path); camera=DepthCamera(folder,cal)
    started=time.monotonic(); first=None; last_sequence=-1; error=None
    try:
        while True:
            now=time.monotonic()
            if camera.error: raise RuntimeError(camera.error)
            current=camera.latest
            if current:
                h,d=current
                if now-h['received_monotonic']>1.: raise RuntimeError('Depth delivery stale for more than 1 second; stop planning')
                if first is None: first=now
                if h['sequence']!=last_sequence and not headless:
                    image=cv2.applyColorMap(np.uint8(np.clip(d*h['value_scale_m']/3.,0,1)*255),cv2.COLORMAP_TURBO)
                    image[d==0]=0
                    cv2.putText(image,f'SOFTWARE ONLY / NO CAN | full 1280x800 depth | frames {camera.count}',(12,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2)
                    cv2.putText(image,'Black = unknown, NOT free. S: save snapshot   Esc/Q: save and close',(12,52),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2)
                    cv2.imshow('X5 depth planner - arm OFF',image); last_sequence=h['sequence']
                if first is not None and seconds>0 and now-first>=seconds: break
            elif now-started>15: raise RuntimeError('Depth camera startup timeout')
            key=-1 if headless else cv2.waitKey(1)&255
            if key in (27,ord('q')): break
            if key==ord('s') and len(camera.frames)==5:
                snapshot(folder,camera.meta,list(camera.frames),cal_path)
            time.sleep(.005)
        if len(camera.frames)<5: raise RuntimeError('Not enough depth frames to create a scene')
        snapshot(folder,camera.meta,list(camera.frames),cal_path)
        if camera.error: raise RuntimeError(camera.error)
        if time.monotonic()-camera.latest[0]['received_monotonic']>1.:
            raise RuntimeError('Depth delivery became stale while saving the scene')
    except BaseException as e:
        error=str(e); raise
    finally:
        elapsed=0 if first is None else time.monotonic()-first
        camera.close()
        if not headless: cv2.destroyAllWindows()
        atomic_json(folder/'capture_report.json',dict(passed=error is None, error=error,
                    duration_after_first_frame_s=elapsed, frames=camera.count,
                    maximum_received_gap_s=camera.maximum_gap_s, native_depth_resolution=[1280,800],
                    camera_only=True, can_access=False, motor_commands=0,
                    warning='SDK warnings may record upstream dropped USB frames; inspect depth_sdk_logs.'))
    return folder/'scene.json'
