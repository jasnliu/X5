"""Native Qt/OpenGL and independent mock controller. Kernel/Python deny CAN."""
import json,sys,time
from pathlib import Path
from PyQt5 import QtCore,QtTest
from drum_planner.baseline.app import View
from drum_planner.common import DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can
no_can();d=Path(sys.argv[1]);d.mkdir(parents=True,exist_ok=True)
v=View(Path('planner_runs/20261019T215312Z-baseline-ba479b/plan/result/plan.json'),d,DEFAULT_CALIBRATION,simulated=True)
report={};info=None
def pump(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:v.poll();time.sleep(.01)
def wait(test,seconds=50):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  v.poll();time.sleep(.01)
  if test():return
 raise AssertionError((v.hardware_status,v.last_error))
def click():
 assert v.reverse_button.isEnabled()
 QtTest.QTest.mouseClick(v.reverse_button,QtCore.Qt.LeftButton)
try:
 assert v.reverse_button.isEnabled()
 v.execute_button.click();wait(lambda:v.hardware_status.get('phase')=='OUTWARD')
 info=v.client_info;assert info['simulated'];wait(lambda:v.hardware_status.get('tau',0)>.5)
 v.send('inject_fault',kind='can');wait(lambda:v.hardware_status.get('phase')=='FAULT_HOLD' and not v.hardware_status.get('hold_confirmed'))
 click();wait(lambda:'CAN failure' in v.reverse_error);pump(.4)
 assert 'Reverse not started' in v.hardware_label.text() and v.reverse_button.isEnabled()
 assert v.hardware_status['phase']=='FAULT_HOLD' and not v.hardware_status['disabled']
 v.viewport.save_window(v.window,d/'fault_button_enabled.png')
 v.send('inject_fault',kind='clear');pump(.7)
 assert v.hardware_status['phase']=='FAULT_HOLD' # never queues motion after a failed click
 click();wait(lambda:'Fresh hold established' in v.reverse_error);pump(.8)
 click();wait(lambda:v.hardware_status.get('phase')=='RECOVERY');wait(lambda:v.hardware_status.get('phase')=='RELAXED')
 assert v.hardware_status['disabled'] and v.hardware_status['tau']==0 and v.reverse_button.isEnabled()
 assert not v.reverse_error and not v.viewport.errors
 events=[json.loads(s) for s in (Path(info['folder'])/'events.jsonl').read_text().splitlines()]
 assert not any(e['phase']=='RELAXING' for e in events)
 assert [e['phase'] for e in events][-2:]==['VERIFIED_START_POSITION','RELAXED']
 atomic_json(d/'result.json',dict(passed=True,can_access=False,physical_motor_commands=0,
  checks=['button enabled without controller','button enabled during unconfirmed fault hold',
  'actual mouse click reaches owner despite fault','rejection stays visible','no queued motion after communication restored',
  'retry establishes measured hold','retry reverses traveled path','relax only after verified rest'],
  events=events,final_status=v.hardware_status))
 v.close_button.click();assert v.poll()==ord('q')
 print('FAULT / REVERSE BUTTON UI TEST PASSED',flush=True)
finally:
 if v.active and v.client:
  assert v.simulated;v.send('inject_fault',kind='clear');pump(.3);v.send('relax');pump(1.3)
 v.close()
