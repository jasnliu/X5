"""Actual GPU UI + independent MOCK motors. Kernel/Python deny CAN."""
import json,time,sys
from pathlib import Path
from PyQt5 import QtWidgets
from drum_planner.baseline.app import View
from drum_planner.common import DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can
no_can()
d=Path(sys.argv[1]);plan=Path.cwd()/'diagnostics/baseline_live_retry_20261019T205948Z/actual_startup/plan/result/plan.json'
def forbidden(*a,**kw):raise AssertionError('Button showed a forbidden modal confirmation')
QtWidgets.QInputDialog.getText=forbidden;QtWidgets.QMessageBox.question=forbidden
reports=[]
def case(name):
 folder=d/name;folder.mkdir();v=View(plan,folder,DEFAULT_CALIBRATION,simulated=True)
 info=None
 def pump(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:v.poll();time.sleep(.01)
 def wait(pred,seconds=60):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   v.poll();time.sleep(.01)
   if pred():return
  raise AssertionError((name,v.hardware_status,v.last_error))
 try:
  v.execute_button.click();wait(lambda:v.hardware_status.get('phase')=='OUTWARD')
  info=v.client_info;assert info['simulated']
  wait(lambda:v.hardware_status.get('tau',0)>.6)
  if name=='relax_while_moving':
   assert v.relax_button.isEnabled()
   v.relax_button.click();wait(lambda:v.hardware_status.get('phase')=='RELAXED')
   assert v.hardware_status['tau']>0 and v.hardware_status['disabled']
  elif name=='one_click_reverse':
   v.hold_button.click();wait(lambda:v.hardware_status.get('hold_confirmed'));pump(.75)
   assert v.reverse_button.isEnabled();v.reverse_button.click()
   wait(lambda:v.hardware_status.get('phase')=='RECOVERY')
   wait(lambda:v.hardware_status.get('phase')=='RELAXED')
   assert v.hardware_status['tau']==0 and v.hardware_status['disabled']
  elif name=='manual_relax_after_can_fault':
   v.send('inject_fault',kind='can');wait(lambda:v.hardware_status.get('phase')=='FAULT_HOLD')
   v.relax_button.click();wait(lambda:v.hardware_status.get('phase')=='RELAX_UNCONFIRMED')
   assert not v.hardware_status['disabled'] and not v.hardware_status['hold_confirmed']
   v.send('inject_fault',kind='clear');pump(.5)
   assert v.hardware_status['phase']=='RELAX_UNCONFIRMED' and not v.hardware_status['disabled']
   v.relax_button.click();wait(lambda:v.hardware_status.get('phase')=='RELAXED')
  elif name=='one_click_controller_exit':
   v.hold_button.click();wait(lambda:v.hardware_status.get('phase')=='HELD');pump(.2)
   assert v.support_button.isEnabled();v.support_button.click()
   wait(lambda:v.hardware_status.get('phase')=='CONTROLLER_EXIT' and v.client is None)
   assert not v.active and not v.hardware_status['disabled'] and not v.last_error
   v.close_button.click();assert v.close_requested
  elif name=='close_window_and_reconnect':
   v.window.close();assert v.close_requested
   assert v.poll()==ord('q');v.close();time.sleep(.3)
   # Close really closed, not a hidden confirmation/refusal. Owner survives.
   assert not v.window.isVisible()
   resumed=d/'reconnected';resumed.mkdir();v=View(plan,resumed,DEFAULT_CALIBRATION,simulated=True,resume=info)
   wait(lambda:v.hardware_status.get('phase')=='FAULT_HOLD' and v.hardware_status.get('hold_confirmed'))
   assert not v.hardware_status['disabled']
   v.relax_button.click();wait(lambda:v.hardware_status.get('phase')=='RELAXED')
  elif name=='normal_cycle':wait(lambda:v.hardware_status.get('phase')=='RELAXED')
  else:raise AssertionError(name)
  assert not v.viewport.errors
  events=[json.loads(x) for x in (Path(info['folder'])/'events.jsonl').read_text().splitlines()]
  if name=='relax_while_moving':
   assert 'RETURN' not in [x['phase'] for x in events]
   assert any(x['phase']=='RELAXED' and x.get('operator_requested') for x in events)
  if name=='one_click_controller_exit':assert not any(x['phase'] in ('RELAXING','RELAXED') for x in events)
  if name=='normal_cycle':
   assert not any(x.get('operator_requested') for x in events)
   enter=next(x['monotonic'] for x in events if x['phase']=='GOAL_HOLD')
   leave=next(x['monotonic'] for x in events if x['phase']=='RETURN');assert leave-enter>=2.
  v.viewport.save_window(v.window,d/(name+'.png'))
  reports.append(dict(case=name,passed=True,status=v.hardware_status,events=events,render=v.report(),controller=info))
  print(name,'PASSED',flush=True)
 finally:
  if v.active and v.client:
   assert v.client_info['simulated']
   v.send('inject_fault',kind='clear');pump(.2);v.send('relax');pump(1.2)
   if v.active:v.send('supported_exit');pump(.2)
  v.close()
for name in ('relax_while_moving','one_click_reverse','manual_relax_after_can_fault','one_click_controller_exit','close_window_and_reconnect','normal_cycle'):case(name)
atomic_json(d/'buttons_report.json',dict(passed=True,cases=reports,can_access=False,physical_motor_commands=0))
print('ALL DIRECT BUTTON MOCK UI TESTS PASSED',flush=True)
