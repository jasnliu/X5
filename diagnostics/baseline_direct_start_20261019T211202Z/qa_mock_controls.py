import json,os,time,sys
from pathlib import Path
from PyQt5 import QtWidgets
from drum_planner.baseline.app import View
from drum_planner.common import DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can
no_can()
d=Path(sys.argv[1]);plan=Path('/home/jason/Proyectos3/X5/diagnostics/baseline_live_retry_20261019T205948Z/actual_startup/plan/result/plan.json')
folder=d/'mock_ui';folder.mkdir()
v=View(plan,folder,DEFAULT_CALIBRATION,simulated=True)
def wait_for(predicate,timeout=45):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  v.poll();time.sleep(.01)
  if predicate():return
 raise AssertionError((v.hardware_status,v.last_error))
try:
 v.rate_box.setValue(.75);assert v.requested_rate==.75
 v.toggle_play();wait_for(lambda:v.clock.position>.25);v.toggle_play();v.seek(.1)
 v.execute_button.click();wait_for(lambda:v.hardware_status.get('phase')=='OUTWARD')
 wait_for(lambda:v.hardware_status.get('tau',0)>.7)
 v.emergency_hold();wait_for(lambda:v.hardware_status.get('phase')=='HELD' and v.hardware_status.get('hold_confirmed'))
 tau=v.hardware_status['tau'];time.sleep(.1)
 for _ in range(75):v.poll();time.sleep(.01)
 assert v.hardware_status['tau']==tau and not v.hardware_status['disabled']
 v.request_close();assert not v.close_requested
 for _ in range(70):v.poll();time.sleep(.01)
 v.send('reverse',scene_confirmed=True);wait_for(lambda:v.hardware_status.get('phase')=='RECOVERY')
 v.emergency_hold();wait_for(lambda:v.hardware_status.get('phase')=='HELD')
 for _ in range(70):v.poll();time.sleep(.01)
 # Simulate a UI disappearance; independent service must stay held.
 info=v.client_info;v.client.close();v.client=None
 time.sleep(.15);v.reconnect();wait_for(lambda:v.hardware_status.get('phase')=='FAULT_HOLD')
 for _ in range(70):v.poll();time.sleep(.01)
 v.send('inject_fault',kind='can');wait_for(lambda:bool(v.hardware_status.get('feedback_error')))
 assert not v.hardware_status.get('can_return')
 v.send('inject_fault',kind='clear')
 for _ in range(100):v.poll();time.sleep(.01)
 v.send('reverse',scene_confirmed=True);wait_for(lambda:v.hardware_status.get('phase')=='RECOVERY')
 wait_for(lambda:v.hardware_status.get('phase')=='RELAXED',timeout=60)
 assert v.hardware_status['disabled'] and not v.viewport.errors
 v.viewport.save_window(v.window,d/'baseline_mock_ui.png')
 atomic_json(d/'mock_ui_report.json',dict(passed=True,checks=['preview','speed','start','hold','blocked_close','checked_prefix_reverse','hold_during_reverse','GUI_disconnect','reconnect','CAN_fault_blocks_reverse','clear_fault_then_recover','supported_start_release'],status=v.hardware_status,render=v.report(),motor_commands=0,can_access=False))
 print('MOCK UI QA PASSED',v.hardware_status,flush=True)
finally:
 if v.active and v.client:
  v.emergency_hold()
  for _ in range(20):v.poll();time.sleep(.01)
  v.send('supported_exit',confirmation='ARM PHYSICALLY SUPPORTED')
  for _ in range(20):
   try:v.poll()
   except Exception:pass
   time.sleep(.01)
 v.close()
