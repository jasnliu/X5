"""Disposable camera/calculation/start-check workers; queries ONLY, no actuators.

One observer owns CAN for the entire calculation, not merely the capture.
Every right-arm batch is compared with the Calculate pose. Passive left sway
is allowed. The path is prepared against this ONE captured scene at Calculate;
Start does not recapture it or perform a different scene check.
"""
import argparse,json,time
from pathlib import Path

from ..common import DEFAULT_CALIBRATION,JOINTS,atomic_json,calibration,digest
from ..current_plan import capture_current,verify_unchanged
from ..read_only import ReadOnlyObserver,Telemetry
from ..depth import DepthCamera,snapshot
from .feedback import normalized_pose,movement_error,RollingAudit
from . import tasks


class MonitoredObserver(ReadOnlyObserver):
    def __init__(self,folder,anchor,output):
        self.anchor=anchor;self.output=output
        super().__init__(folder,log_factory=RollingAudit)
    def sample(self):
        try:
            batch=super().sample();pose=normalized_pose(batch['joints'],batch['start'])
            error=movement_error(self.anchor,pose)
            atomic_json(self.output,dict(pose=pose,error=error))
            if error:raise ValueError(error)
            return batch
        except Exception as exc:
            # Preserve the last pose if possible, never fabricate new feedback.
            previous=json.loads(self.output.read_text()) if self.output.exists() else {}
            atomic_json(self.output,dict(previous,error=str(exc)));raise


def camera_scene(folder,cal_path):
    camera=None;cal=calibration(cal_path)
    try:
        camera=DepthCamera(folder,cal);began=time.monotonic()
        while len(camera.frames)<5:
            if camera.error:raise RuntimeError(camera.error)
            if time.monotonic()-began>15:raise RuntimeError('Camera snapshot timed out; click Calculate to retry')
            time.sleep(.01)
        frames=list(camera.frames);meta=camera.meta;camera.close()
        if camera.error:raise RuntimeError(camera.error)
        snapshot(folder,meta,frames,cal_path)
        print('Camera snapshot captured; camera released. NO trajectory calculation.',flush=True)
    finally:
        if camera:camera.close()


def calculate(folder,cal_path,anchor,*,plan=None):
    folder=Path(folder);capture=folder/'capture';capture.mkdir()
    worker=Telemetry(capture,observer_factory=lambda f:MonitoredObserver(f,anchor,folder/'feedback.json'))
    try:
        print('Leave the RIGHT arm still from Calculate until Start; passive left sway is allowed. State queries only.',flush=True)
        state,scene=capture_current(capture,worker,cal_path,stationary_joints=JOINTS)
        proof=verify_unchanged(worker,state,stationary_joints=JOINTS)
        atomic_json(capture/'acquisition.json',dict(state=str(capture/'measured_start.json'),scene=str(capture/'camera/scene.json'),
            state_sha256=digest(capture/'measured_start.json'),scene_sha256=digest(capture/'camera/scene.json'),
            proof=proof,boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),motor_commands=0))
        atomic_json(folder/'progress.json',dict(stage='checking' if plan else 'planning',scene=str(capture/'camera/scene.json')))
        if plan is None:plan=tasks.make_plan(folder/'plan',capture,cal_path)
        # Dense exact-curve proof belongs to Calculate, using the very same
        # scene as the planner/preview. No second camera acquisition on Start.
        tasks.prepare(folder/'checked',plan,capture,cal_path,allow_replan=False)
        result=dict(plan=str(plan),prepared=str(folder/'checked/prepared.json'))
        verify_unchanged(worker,state,stationary_joints=JOINTS)
        if worker.error:raise RuntimeError(worker.error)
        result.update(scene=str(capture/'camera/scene.json'),motor_commands=0,anchor=anchor)
        atomic_json(folder/'result.json',result)
        print('Calculation and exact-path check complete on the captured scene. Preview, then click Start.',flush=True)
    finally:
        worker.close();atomic_json(folder/'query_report.json',worker.report())
    # The last in-flight encoder batch may fail while close() joins the reader.
    # Never publish a successful calculation after that latched failure.
    if worker.error:
        (folder/'result.json').unlink(missing_ok=True)
        raise RuntimeError(worker.error)


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['scene','calculate','check'])
    p.add_argument('--folder',required=True);p.add_argument('--input');p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    a=p.parse_args();folder=Path(a.folder);folder.mkdir(parents=True,exist_ok=True)
    try:
        if a.action=='scene':
            from ..experiment.config import no_can
            no_can();camera_scene(folder,a.calibration)
        else:
            inputs=json.loads(Path(a.input).read_text())
            calculate(folder,a.calibration,inputs['anchor'],plan=inputs.get('plan') if a.action=='check' else None)
    except Exception as exc:
        atomic_json(folder/'error.json',dict(error=str(exc),motor_commands=0));raise


if __name__=='__main__':main()
