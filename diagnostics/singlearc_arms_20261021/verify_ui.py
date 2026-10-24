"""Native three-window UI + independent simulated owner. Python/kernel deny CAN."""
import json,time,sys,os,subprocess,copy
from pathlib import Path
import numpy as np
from drum_planner.experiment.config import no_can
no_can()
from drum_planner.baseline.workspace import Workspace
from drum_planner.common import atomic_json,JOINTS,arm_joints
D=Path(sys.argv[1]).resolve();D.mkdir(parents=True,exist_ok=True);f=D/'mock';f.mkdir()
PLAN=Path(sys.argv[2]).resolve();SIDE=sys.argv[3];JOINTS=arm_joints(SIDE)
# Even a mistaken caller cannot launch a physical owner during this test.
from drum_planner.baseline import service
original_launch=service.launch
def simulated_launch(*args,**kwargs):
    assert kwargs.get('simulate_plan') is not None
    return original_launch(*args,**kwargs)
service.launch=simulated_launch
# Exercise the actual no-plan startup constructor too; do not poll it,
# since hardware-mode polling starts real read-only acquisition.
startup=Workspace(D/'real_startup',autoscene=False,moving_side=SIDE)
assert startup.preview.robot.plan.stick_link==SIDE+'_drumstick'
assert startup.preview.robot.plan.joints==JOINTS
assert len([w for w in startup.windows if w.isVisible()])==3
startup.app.processEvents()
startup.control.grab().save(str(D/'startup_controls.png'))
startup.close()
from PyQt5 import QtCore
QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete)
startup.app.processEvents();time.sleep(.5);startup.app.processEvents()
v=Workspace(f,simulate_plan=PLAN,autoscene=False,moving_side=SIDE)
checks=['real no-plan startup DisplayModel path (no polling/devices)'];sessions=[]
def pump(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:assert v.poll();time.sleep(.008)
def wait(test,seconds=60):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        assert v.poll();time.sleep(.008)
        assert all(b.isEnabled() for b in v.control.buttons.values())
        if test():return
    raise AssertionError((v.owner_status,v.control.status.text(),v.last_error))
def click(name):v.control.buttons[name].click()
def start_cycle():
    click('calculate');assert v.plan_valid
    click('start');wait(lambda:v.owner_status.get('phase')=='OUTWARD')
    assert v.owner_info['simulated'];sessions.append(v.owner_info['folder'])
def ended():
    wait(lambda:v.owner_info is None)
    final=Path(sessions[-1])/'final_status.json'
    wait(lambda:final.exists(),seconds=5)
    s=json.loads(final.read_text())
    assert s['phase']=='RELAXED' and s['disabled'],s
    assert not v.plan_valid and all(b.isEnabled() for b in v.control.buttons.values())
try:
    pump(.4)
    assert v.plan is None and v.preview.plan is None and v.job is None
    windows=[line for line in subprocess.check_output(['wmctrl','-lpG'],text=True).splitlines() if line.split()[2]==str(os.getpid())]
    assert len(windows)==3,windows
    for w in v.windows:
        buttons=[b for b in w.findChildren(v.control.buttons['start'].__class__) if b.isVisible()]
        rects=[v.control.rect().__class__(b.mapTo(w,v.control.pos().__class__(0,0)),b.size()) for b in buttons]
        for i,r in enumerate(rects):
            assert w.rect().contains(r), (w.windowTitle(),buttons[i].text(),r)
            assert not any(r.intersects(x) for x in rects[i+1:])
    click('start');click('hold');click('reverse');click('cancel')
    assert v.owner_info is None
    checks+=['exactly three native windows','no plan until Calculate','no overlapping controls','all five actions always clickable','prerequisite errors do not block retry']
    click('calculate');v.preview.play.click();pump(.2);v.preview.pause.click()
    v.preview.rate.setValue(.65);v.preview.timeline.setValue(1500)
    fixed=v.preview.robot.last_positions.copy();old=copy.deepcopy(v.sim_joints)
    passive=arm_joints('left' if SIDE=='right' else 'right')[0]
    v.sim_joints[passive]+=.08;pump(.15)
    assert v.plan_valid and np.array_equal(fixed,v.preview.robot.last_positions)
    v.sim_joints=copy.deepcopy(old);pump(.1)
    checks+=['passive arm sway does not invalidate selected-arm plan']
    v.sim_joints[JOINTS[0]]+=.08;pump(.15)
    assert not v.plan_valid and np.array_equal(fixed,v.preview.robot.last_positions)
    assert v.live.robot.last_positions[0]!=old[JOINTS[0]]
    v.sim_joints=old;pump(.1);click('calculate');assert v.plan_valid
    v.preview.rate.setValue(1.)
    checks+=['preview play pause scrub speed','live movement affects only connected window','movement invalidates plan','recalculate in same process']
    v.preview.viewport.save_window(v.preview,f/'preview.png');v.live.viewport.save_window(v.live,f/'live.png');v.control.grab().save(str(f/'controls.png'))
    start_cycle();info=v.owner_info;click('calculate');click('start')
    assert v.owner_info is info
    v.preview.rate.setValue(.5);pump(.2);assert v.owner_status['rate']==1.
    click('hold');wait(lambda:v.owner_status.get('phase')=='HELD')
    # Reconnect through the SAME saved-session path used by --resume; a new
    # Workspace infers its arm from metadata and never creates a second owner.
    info=v.owner_info;v.close()
    QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete)
    v=Workspace(Path(info['folder']),resume=info,autoscene=False)
    assert v.side==SIDE and v.simulated
    wait(lambda:v.owner_status.get('phase') in ('HELD','FAULT_HOLD'))
    checks+=['saved owner reconnect restores selected arm without enable']
    click('reverse');ended()
    checks+=['busy buttons explain without second owner','preview speed never changes active arm speed','one click Reverse holds checks returns and relaxes','completed owner removed without restart']
    v.preview.rate.setValue(1.);start_cycle();ended()
    checks+=['new cycle after return in same program','full outward goal-hold reverse-rest-release cycle']
    start_cycle();pump(.2);v.client.send('inject_fault',kind='can')
    wait(lambda:v.owner_status.get('phase')=='FAULT_HOLD');pump(.6)
    assert 'STALE' in v.live.caption.text()
    click('reverse');pump(.3);assert v.reverse_intent is None
    v.client.send('inject_fault',kind='clear');pump(.7)
    assert v.owner_status['phase']=='FAULT_HOLD' and not v.owner_status['disabled']
    click('reverse');ended()
    checks+=['CAN failure marks live pose stale','fault leaves every button enabled','no queued return after failed request','one click recovery after restored feedback','no automatic midair relaxation']
    assert all(not w.viewport.errors for w in (v.preview,v.live))
    atomic_json(D/'mock_report.json',dict(passed=True,checks=checks,sessions=sessions,physical_motor_commands=0,can_access=False,moving_side=SIDE,report={k:val for k,val in v.report().items() if k!='owner'}))
    print('THREE-WINDOW MOCK WORKFLOW PASSED',flush=True)
finally:
    if v.client and v.active:
        assert v.simulated;v.client.send('inject_fault',kind='clear');pump(.3);v.client.send('relax');pump(1.)
    v.close()
