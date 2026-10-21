"""No CAN: replan UI, live measured mock feedback, fault, recovery, end-release."""
import json,sys,time,os,subprocess
from pathlib import Path
import numpy as np
from PyQt5 import QtCore,QtTest
from drum_planner.baseline.app import View
from drum_planner.common import DEFAULT_CALIBRATION,atomic_json,JOINTS
from drum_planner.experiment.config import no_can
no_can();d=Path(sys.argv[1]).resolve();folder=d/'mock';folder.mkdir()
packet=json.loads((d/'replay/prepared.json').read_text())
v=View(packet['replanned']['previous_plan'],folder,DEFAULT_CALIBRATION,simulated=True)
info=None;pose_checks=0

def pump(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:v.poll();time.sleep(.01)
def wait(test,seconds=50):
    global pose_checks
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        v.poll();time.sleep(.01)
        if v.display_pose and v.live_display:
            v.draw_canvas()
            assert np.allclose(v.robot.last_positions,[v.display_pose['joints'][j] for j in JOINTS])
            pose_checks+=1
        if test():return
    raise AssertionError((v.hardware_status,v.last_error))
def click_reverse():
    assert v.reverse_button.isEnabled();QtTest.QTest.mouseClick(v.reverse_button,QtCore.Qt.LeftButton)
try:
    before=[layer.geometry_uploads for *_,layer in v.robot.mesh_entries]
    v.adopt_replanned(packet)
    assert str(v.plan.path)==packet['plan'] and v.robot.plan is v.plan and not v.active
    assert not v.live_display and not v.pending_start
    v.seek(v.plan.obj['outward'][-1]['time_s']);assert np.allclose(v.robot.last_positions,v.plan.req['goal'])
    v.seek(0);v.play_button.click();pump(.3);assert v.clock.playing
    v.live_box.click();assert v.live_display and not v.clock.playing
    # One native window for this PID; controls remain contained/nonoverlapping.
    own=[x for x in subprocess.check_output(['wmctrl','-lpG'],text=True).splitlines() if x.split()[2]==str(os.getpid())]
    assert len(own)==1,own
    for size in [(1440,900),(1120,820)]:
        v.window.resize(*size);v.app.processEvents()
        buttons=[b for b in v.window.findChildren(v.QtWidgets.QPushButton) if b.isVisible()]
        rects=[QtCore.QRect(b.mapTo(v.window,QtCore.QPoint()),b.size()) for b in buttons]
        for i,r in enumerate(rects):
            assert v.window.rect().contains(r)
            assert not any(r.intersects(other) for other in rects[i+1:])
    v.window.resize(1440,900);v.app.processEvents()
    v.viewport.save_window(v.window,folder/'replacement.png')
    v.execute_button.click();wait(lambda:v.hardware_status.get('phase')=='OUTWARD');info=v.client_info
    assert info['simulated'] and v.live_display
    wait(lambda:v.hardware_status.get('tau',0)>.5)
    v.send('inject_fault',kind='can');wait(lambda:v.hardware_status.get('phase')=='FAULT_HOLD')
    pump(.7);v.draw_canvas();assert 'STALE' in v.viewport.title
    click_reverse();wait(lambda:'CAN failure' in v.reverse_error)
    assert v.reverse_button.isEnabled() and not v.hardware_status['disabled']
    v.viewport.save_window(v.window,folder/'fault_live_stale.png')
    v.send('inject_fault',kind='clear');pump(.7)
    assert v.hardware_status['phase']=='FAULT_HOLD'
    click_reverse();wait(lambda:'Fresh hold established' in v.reverse_error);pump(.8)
    click_reverse();wait(lambda:v.hardware_status.get('phase')=='RECOVERY')
    wait(lambda:v.hardware_status.get('phase')=='RELAXED')
    assert v.hardware_status['disabled'] and v.hardware_status['tau']==0
    assert before==[layer.geometry_uploads for *_,layer in v.robot.mesh_entries]
    assert not v.viewport.errors
    events=[json.loads(x) for x in (Path(info['folder'])/'events.jsonl').read_text().splitlines()]
    assert [x['phase'] for x in events][-2:]==['VERIFIED_START_POSITION','RELAXED']
    atomic_json(folder/'report.json',dict(passed=True,motor_commands=0,can_access=False,pose_checks=pose_checks,
        checks=['replacement adopted without auto start','replacement preview and goal','live vs preview toggle',
            'one native window','no overlapping controls','active owner measured joints drive model','CAN loss marks stale',
            'reverse always enabled','fault does not relax','no queued recovery','reverse then verified rest release',
            'geometry uploaded only once'],events=events,render=v.report()))
    v.close_button.click();assert v.poll()==ord('q')
    print('MOCK LIVE FEEDBACK / REPLAN / FAULT-REVERSE QA PASSED',flush=True)
finally:
    if v.active and v.client:
        assert v.simulated;v.send('inject_fault',kind='clear');pump(.3);v.send('relax');pump(1.3)
    v.close()
