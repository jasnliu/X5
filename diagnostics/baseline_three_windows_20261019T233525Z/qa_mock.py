"""Native three-window UI + independent simulated owner. Python/kernel deny CAN."""
import json,time,sys,os,subprocess,copy
from pathlib import Path
import numpy as np
from drum_planner.experiment.config import no_can
no_can()
from drum_planner.baseline.workspace import Workspace
from drum_planner.common import atomic_json,JOINTS
D=Path(sys.argv[1]).resolve();f=D/'mock';f.mkdir()
v=Workspace(f,simulate_plan=Path('planner_runs/20261019T231921Z-baseline-f78aca/plan/result/plan.json'),autoscene=False)
checks=[];sessions=[]
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
    s=json.loads((Path(sessions[-1])/'status.json').read_text())
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
    atomic_json(D/'mock_report.json',dict(passed=True,checks=checks,sessions=sessions,physical_motor_commands=0,can_access=False,report=v.report()))
    print('THREE-WINDOW MOCK WORKFLOW PASSED',flush=True)
finally:
    if v.client and v.active:
        assert v.simulated;v.client.send('inject_fault',kind='clear');pump(.3);v.client.send('relax');pump(1.)
    v.close()
