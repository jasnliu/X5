"""Startup only: existing query-only acquisition, BOTH future moving arms still."""
from pathlib import Path
from drum_planner.common import DEFAULT_CALIBRATION, JOINTS, LEFT_JOINTS, atomic_json
from drum_planner.experiment.rgbd import capture as paired_capture
from drum_planner.read_only import Telemetry


def capture(folder,calibration=DEFAULT_CALIBRATION):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    worker=Telemetry(folder)
    try:
        print('GENERATED BEAT STARTUP: one frozen paired scene + BOTH resting arms. Encoder queries only; no arm enables or centering.',flush=True)
        paired_capture(folder/'camera',calibration,worker,stationary_joints=JOINTS+LEFT_JOINTS)
    finally:
        worker.close();atomic_json(folder/'query_report.json',worker.report())
    if worker.error:raise RuntimeError(worker.error)
    print('Camera and acquisition CAN connections CLOSED. Planning has no CAN authority.',flush=True)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--folder',required=True);p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    a=p.parse_args()
    try:capture(a.folder,a.calibration)
    except Exception as exc:
        atomic_json(Path(a.folder)/'error.json',dict(error=str(exc),motor_commands=0));raise
