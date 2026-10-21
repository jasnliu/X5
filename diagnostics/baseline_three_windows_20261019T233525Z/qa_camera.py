"""Actual initial camera in production three-window UI. Never actuator access."""
import time,json,sys
from pathlib import Path
from drum_planner.baseline.workspace import Workspace
from drum_planner.baseline import service,transport
from drum_planner.common import atomic_json
from drum_planner.bus_ownership import owners
D=Path(sys.argv[1]).resolve();folder=D/'actual_camera';folder.mkdir()
def forbidden(*a,**k):raise AssertionError('Physical controller forbidden during this test')
service.launch=forbidden;transport.BaselineMotors=forbidden
before=owners();v=Workspace(folder)
try:
    began=time.monotonic()
    while v.scene_path is None and time.monotonic()-began<25:
        assert v.poll();time.sleep(.01)
    assert v.scene_path is not None,v.control.status.text()
    assert v.plan is None and v.preview.plan is None and v.owner_info is None
    assert v.preview.scene[1] is v.live.scene[1]
    assert len(v.preview.scene[1].faces)>1000
    assert all(b.isEnabled() for b in v.control.buttons.values())
    # Prerequisite handling cannot enable anything, and cannot lock buttons.
    v.control.buttons['start'].click();assert v.owner_info is None
    for _ in range(100):v.poll();time.sleep(.01)
    atomic_json(D/'camera_report.json',dict(passed=True,camera_scene=str(v.scene_path),startup_plan=None,
        shared_scene=True,physical_motor_commands=0,live_encoder_feedback_available=v.pose is not None,
        external_can_owners_before=before,buttons={k:b.isEnabled() for k,b in v.control.buttons.items()},report=v.report()))
    v.preview.viewport.save_window(v.preview,folder/'preview.png');v.live.viewport.save_window(v.live,folder/'live.png')
    v.control.grab().save(str(folder/'controls.png'))
    print('ACTUAL CAMERA: three windows, frozen shared scene, no automatic plan, all buttons available; no actuator access',flush=True)
finally:v.close()
