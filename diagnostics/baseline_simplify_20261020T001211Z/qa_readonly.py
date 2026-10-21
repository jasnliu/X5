"""Actual camera/encoders/Calculate/preview, with motor launch forbidden."""
import json,sys,time
from pathlib import Path
from drum_planner.baseline import service,transport,tasks
from drum_planner.baseline.workspace import Workspace
from drum_planner.bus_ownership import owners
from drum_planner.common import atomic_json,JOINTS
D=Path(sys.argv[1]).resolve();folder=D/'actual_readonly';folder.mkdir()
def forbidden(*a,**k):raise AssertionError('Motor owner forbidden in read-only QA')
service.launch=forbidden;transport.BaselineMotors=forbidden
assert not owners(),owners()
v=Workspace(folder)
try:
    began=time.monotonic()
    while time.monotonic()-began<25 and (v.scene_path is None or v.pose is None):
        assert v.poll();time.sleep(.01)
    assert v.pose is not None,v.reader_error
    assert v.scene_path is not None,v.control.status.text()
    assert v.plan is None and v.owner_info is None
    v.control.buttons['calculate'].click()
    began=time.monotonic()
    while v.job is not None and time.monotonic()-began<320:
        assert v.poll();time.sleep(.01)
    assert v.plan_valid and v.prepared,v.control.status.text()
    packet,curve=tasks.check_packet(v.prepared)
    assert packet['fresh_scene']==str(v.scene_path)
    assert v.preview.scene[1] is v.live.scene[1]
    assert len([w for w in v.app.topLevelWidgets() if w.isVisible()])==3
    for _ in range(10):v.poll();time.sleep(.01)
    plan_scene=v.scene_path;plan_path=v.plan.path
    v.preview.play.click()
    for _ in range(50):v.poll();time.sleep(.01)
    assert v.preview.clock.playing
    v.preview.pause.click();assert not v.preview.clock.playing
    v.preview.rate.setValue(.75);v.preview.timeline.setValue(1000);v.poll()
    assert v.preview.clock.sample()>0 and v.prepared
    # Test real Start callback dispatch without creating ANY motor owner.
    # The current actual encoder stream remains read-only throughout.
    dispatched=[]
    v.launch_owner=lambda:dispatched.append(dict(prepared=str(v.prepared),rate=v.start_rate))
    v.control.buttons['start'].click()
    assert dispatched and v.job is None and v.owner_info is None
    assert v.scene_path==plan_scene and v.plan.path==plan_path
    v.want_start=False
    for b in v.control.buttons.values():assert b.isEnabled()
    v.preview.viewport.save_window(v.preview,folder/'preview.png')
    v.live.viewport.save_window(v.live,folder/'live.png')
    v.control.grab().save(str(folder/'controls.png'))
    report=dict(passed=True,actual_camera=True,actual_disabled_encoders=True,
        actual_calculate=True,actual_dense_collision_check=packet['proof'],
        plan=str(v.plan.path),prepared=str(v.prepared),start_callback_dry_run=dispatched,
        camera_scene_unchanged_on_start=True,physical_enable_or_motion_commands=0,
        right_stationary_policy=JOINTS,report=v.report())
    atomic_json(D/'actual_readonly_report.json',report)
    print('ACTUAL READ-ONLY QA PASSED: camera, encoders, Calculate, dense proof, three windows, preview, dry-run Start dispatch. NO MOTOR OWNER.',flush=True)
finally:v.close()
