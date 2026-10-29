"""Frozen RGBD/pose acquisition. Encoder queries only; no motor-mode changes."""
import argparse,json,subprocess,time
from pathlib import Path
from drum_planner.common import DEFAULT_CALIBRATION,JOINTS,atomic_json,calibration,digest,read_state
from .config import ROOT,CAMERA_PYTHON
from . import inputs


def run(folder,cal_path=DEFAULT_CALIBRATION,*,replay=None,state_path=None):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False);camera=folder/'camera';camera.mkdir()
    calibration(cal_path)
    if replay and not state_path:raise ValueError('Replay requires an explicit saved --state; no implicit pose fallback')
    worker=None;child=None;report=dict(passed=False,motor_commands=0,mode_changes=0,gripper_commands=0,
        camera_replay=bool(replay),pose_source='explicit saved pose' if state_path else 'fresh query-only encoders',
        source_state=None if state_path is None else str(Path(state_path).resolve()),
        source_pair=None if replay is None else str(Path(replay).resolve()))
    try:
        if state_path:
            state=read_state(state_path)
            if state.get('calibration_sha256')!=digest(cal_path):raise ValueError('Saved pose uses a different calibration')
        else:
            from drum_planner.read_only import Telemetry
            worker=Telemetry(folder);began=time.monotonic()
            print('Read-only BOTH-arm encoders; motors must already be disabled. No motor commands.',flush=True)
            while worker.count<15 or time.monotonic()-began<.8:
                if worker.error:raise RuntimeError(worker.error)
                if time.monotonic()-began>5:raise RuntimeError('No disabled-motor encoder snapshot')
                time.sleep(.01)
        command=[str(CAMERA_PYTHON),'-B',str(ROOT/'cymbal_destination/camera_worker.py'),
                 '--folder',str(camera),'--calibration',str(cal_path)]
        if replay:command+=['--replay',str(Path(replay).resolve())]
        child=subprocess.Popen(command);began=time.monotonic()
        while child.poll() is None:
            if worker and worker.error:raise RuntimeError(worker.error)
            if time.monotonic()-began>70:raise TimeoutError('Frozen target acquisition exceeded 70 seconds')
            time.sleep(.05)
        if child.returncode:
            error=camera/'error.json'
            raise RuntimeError(json.loads(error.read_text())['error'] if error.exists() else 'DT1 snapshot child failed')
        capture=json.loads((camera/'capture.json').read_text())
        if not capture['camera_released']:raise RuntimeError('Camera still open')
        if worker:
            from drum_planner.read_only import select_stationary
            from drum_planner.hardware_check import measured_state
            stamp=capture['header']['received']
            while time.monotonic()<stamp+.4:
                if worker.error:raise RuntimeError(worker.error)
                time.sleep(.01)
            samples=[s for s in worker.samples() if stamp-1.<=s['start'] and s['end']<=stamp+.5]
            selected,evidence=select_stationary(samples,stamp,stationary_joints=JOINTS)
            state=measured_state(selected,evidence,cal_path)
            worker.close()
            if worker.error:raise RuntimeError(worker.error)
        atomic_json(folder/'measured_start.json',state)
        # The copied state retains its true origin; no old state is relabeled fresh.
        report['synchronized_pose']=worker is not None
        inputs.build_scene(folder,cal_path)
        report.update(passed=True,camera_released=True,encoder_released=True,
                      state_sha256=digest(folder/'measured_start.json'),scene_sha256=digest(camera/'scene.json'))
        print('SNAPSHOT FROZEN: camera and encoder connections CLOSED. Destination computation/preview only.',flush=True)
    except BaseException as exc:
        report['error']=str(exc) or type(exc).__name__;raise
    finally:
        if child and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=4)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        if worker:worker.close()
        report['telemetry']=None if worker is None else worker.report()
        atomic_json(folder/'acquisition_report.json',report)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',required=True)
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION));p.add_argument('--replay');p.add_argument('--state')
    a=p.parse_args();run(a.folder,a.calibration,replay=a.replay,state_path=a.state)
