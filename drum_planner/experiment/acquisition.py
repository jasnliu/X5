"""One fresh, query-only startup snapshot; never a planning or motor owner.

Runs in its own bounded process: only this startup job may open CAN. Planning
and the preview retain their no-CAN guard after the camera/encoders disconnect.
"""
import argparse
import datetime as dt
from pathlib import Path
import time

from ..common import DEFAULT_CALIBRATION, arm_joints, arm_side, atomic_json, calibration, digest
from ..current_plan import capture_current
from ..read_only import Telemetry


def capture(folder, side='right', cal_path=DEFAULT_CALIBRATION):
    side=arm_side(side)
    calibration(cal_path)  # Reject bad calibration before opening either device.
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False)
    began=time.monotonic();worker=None
    report=dict(schema='x5-experiment-startup-acquisition-v1',passed=False,error=None,
        started_utc=dt.datetime.now(dt.timezone.utc).isoformat(),moving_side=side,
        measured_sides=['right','left'],stationary_joints=arm_joints(side),
        scene_source='one new planning scene fused from five encoder-bracketed native depth frames',
        cached_inputs_used=False,motion_authorized=False,motor_commands=0,
        gripper_commands=0,mode_changes=0,relax_commands=0,camera_released=False)
    try:
        print(f'FRESH STARTUP: camera + BOTH arm poses; {side} arm stationary. '
              'Encoder state queries ONLY; motors must already be disabled. No motor mode changes.',flush=True)
        try:
            worker=Telemetry(folder)
            # Measure both arms, but passive-arm sway is not a stillness veto.
            capture_current(folder,worker,cal_path,stationary_joints=arm_joints(side))
            report['camera_released']=True
        finally:
            if worker:worker.close()
        if worker.error:raise RuntimeError(worker.error)
        state=folder/'measured_start.json';scene=folder/'camera/scene.json'
        report.update(passed=True,state=str(state),state_sha256=digest(state),
                      scene=str(scene),scene_sha256=digest(scene))
        print('FRESH STARTUP COMPLETE: camera and encoder connections CLOSED. '
              'This snapshot stays frozen; simulation only, no live tracking.',flush=True)
        return report
    except BaseException as exc:
        report['error']=str(exc) or type(exc).__name__
        raise
    finally:
        report.update(elapsed_s=time.monotonic()-began,telemetry=None if worker is None else worker.report())
        atomic_json(folder/'acquisition_report.json',report)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',required=True)
    parser.add_argument('--side',choices=['right','left'],required=True)
    parser.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    args=parser.parse_args()
    try:capture(args.folder,args.side,args.calibration)
    except Exception as exc:
        print('Fresh startup acquisition failed (NO saved-input fallback): '+str(exc),flush=True)
        return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
