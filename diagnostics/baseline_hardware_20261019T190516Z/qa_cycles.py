"""Native GUI + independent MOCK owner. Kernel denies CAN to both processes."""
import json,os,subprocess,sys,time
from pathlib import Path
from drum_planner.baseline.app import View
from drum_planner.common import DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can
no_can()
d=Path(sys.argv[1]);plan=next(Path('diagnostics/planner_zone3_20261019T172750Z/actual_camera/plans').glob('baseline-*/plan.json')).resolve()
reports=[]
def test_case(name,kind):
 folder=d/name;folder.mkdir()
 v=View(plan,folder,DEFAULT_CALIBRATION,simulated=True)
 def wait_for(pred,timeout=60):
  deadline=time.monotonic()+timeout
  while time.monotonic()<deadline:
   v.poll();time.sleep(.01)
   if pred():return
  raise AssertionError((name,v.hardware_status,v.last_error))
 def pump(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:v.poll();time.sleep(.01)
 try:
  v.start_controller();wait_for(lambda:v.hardware_status.get('phase')=='OUTWARD')
  if kind=='normal':
   wait_for(lambda:v.hardware_status.get('phase')=='RELAXED')
  else:
   wait_for(lambda:v.hardware_status.get('tau',0)>.9)
   if kind=='can':
    v.send('inject_fault',kind='can');wait_for(lambda:v.hardware_status.get('phase')=='FAULT_HOLD')
    assert not v.hardware_status['hold_confirmed'] and not v.reverse_button.isEnabled()
    v.send('reverse',scene_confirmed=True);pump(.2)
    assert v.hardware_status['phase']=='FAULT_HOLD' and not v.hardware_status['disabled']
    v.send('inject_fault',kind='clear');pump(.6)
    v.retry_hold_button.click();wait_for(lambda:v.hardware_status.get('hold_confirmed'));pump(.7)
   elif kind=='heartbeat':
    time.sleep(1.3) # stopped GUI heartbeat, socket stays open
    wait_for(lambda:v.hardware_status.get('phase')=='FAULT_HOLD')
    assert 'heartbeat' in v.hardware_status['error'];pump(.7)
   elif kind=='signal':
    subprocess.run(['systemctl','--user','kill','--signal=SIGTERM',v.client_info['unit']+'.service'],check=True)
    wait_for(lambda:v.hardware_status.get('phase')=='FAULT_HOLD');pump(.7)
   v.send('reverse',scene_confirmed=True);wait_for(lambda:v.hardware_status.get('phase')=='RECOVERY')
   wait_for(lambda:v.hardware_status.get('phase')=='RELAXED')
  assert v.hardware_status['disabled'] and not v.hardware_status['hold_confirmed'] and not v.viewport.errors
  events=[json.loads(x) for x in (Path(v.client_info['folder'])/'events.jsonl').read_text().splitlines()]
  if kind=='normal':
   enter=next(x['monotonic'] for x in events if x['phase']=='GOAL_HOLD')
   leave=next(x['monotonic'] for x in events if x['phase']=='RETURN')
   assert leave-enter>=2.
  v.viewport.save_window(v.window,d/(name+'.png'))
  reports.append(dict(case=name,passed=True,status=v.hardware_status,events=events,render=v.report()))
  print(name,'PASSED',flush=True)
 finally:
  if v.active and v.client:
   v.emergency_hold();pump(.2);v.send('supported_exit',confirmation='ARM PHYSICALLY SUPPORTED');pump(.1)
  v.close()
for name,kind in [('normal_cycle','normal'),('moving_can_fault','can'),('lost_ui_heartbeat','heartbeat'),('controller_sigterm','signal')]:test_case(name,kind)
atomic_json(d/'cycles_report.json',dict(passed=True,cases=reports,can_access=False,motor_commands=0))
print('ALL MOCK CYCLES PASSED',flush=True)
