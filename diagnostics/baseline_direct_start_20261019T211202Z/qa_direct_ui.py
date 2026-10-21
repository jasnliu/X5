"""Exercise the real Start UI with test-only acquisition and a MOCK owner.
No camera/CAN is opened. Prepared fixtures are deliberately non-executable.
"""
import json,time,sys
from pathlib import Path
from PyQt5 import QtWidgets
from drum_planner.baseline import app,service
from drum_planner.common import DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can
no_can()
d=Path(sys.argv[1]);root=Path.cwd()
plan=root/'diagnostics/baseline_live_retry_20261019T205948Z/actual_startup/plan/result/plan.json'
obj=json.loads(plan.read_text());req=json.loads((plan.parent/'backend_request.json').read_text())
jobs=[];launches=[];v=None
real_launch=service.launch
class TestJob:
 def __init__(self,action,folder,**kw):
  assert action in ('capture','prepare') and 'review' not in kw
  self.action=action;self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
  self.done=time.monotonic()+.25;self.cancelled=False;self.closed=False
  self.log_path=self.folder/'worker.log';self.log_path.write_text('TEST ONLY: no devices used\n')
  if action=='prepare':
   atomic_json(self.folder/'prepared.json',dict(schema='TEST_ONLY_NON_EXECUTABLE',scene_stamp=time.monotonic(),
    fresh_scene=obj['provenance']['scene'],request=dict(scene=req['scene'])))
  jobs.append(self)
 def poll(self):return 0 if self.cancelled or time.monotonic()>=self.done else None
 def close(self):self.closed=True
 def cancel(self):self.cancelled=True

def launch_mock(folder,*,prepared=None,simulate_plan=None,rate=1.):
 assert prepared is not None and simulate_plan is None
 assert json.loads(Path(prepared).read_text())['schema']=='TEST_ONLY_NON_EXECUTABLE'
 launches.append(str(folder))
 return real_launch(folder,simulate_plan=plan,rate=rate)
def forbidden(*a,**kw):raise AssertionError('Startup must not show a modal confirmation')
app.job=TestJob;service.launch=launch_mock
QtWidgets.QInputDialog.getText=forbidden
folder=d/'direct_ui';folder.mkdir()
v=app.View(plan,folder,DEFAULT_CALIBRATION,simulated=False)
v.window.setWindowTitle('AUTOMATED TEST ONLY — MOCK MOTOR OWNER / NO CAN')
v.banner.setText('TEST ONLY — physical-mode callbacks, MOCK devices, CAN forbidden')
def wait_for(pred,timeout=60):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  v.poll();time.sleep(.01)
  if pred():return
 raise AssertionError((v.hardware_status,v.last_error))
try:
 assert v.execute_button.isEnabled() and v.execute_button.text()=='START REAL ARM'
 assert not hasattr(v,'physical_review')
 assert not v.window.findChildren(QtWidgets.QPlainTextEdit)
 v.rate_box.setValue(.75)
 v.toggle_play();wait_for(lambda:v.clock.position>.2);v.toggle_play();v.seek(.1)
 v.execute_button.click();assert v.start_when_ready and not launches
 v.hold_button.click();wait_for(lambda:v.job is None)
 assert not launches and not v.start_when_ready and v.execute_button.isEnabled()
 v.prepare_button.click();wait_for(lambda:v.prepared is not None)
 assert not launches and not v.start_when_ready and not v.active
 # A stale readiness-only result must be refreshed automatically by Start.
 packet=json.loads(v.prepared.read_text());packet['scene_stamp']=time.monotonic()-31
 atomic_json(v.prepared,packet)
 v.execute_button.click();assert v.start_when_ready and not launches
 v.last_error='' # expected cancellation diagnostic from the first test
 wait_for(lambda:v.hardware_status.get('phase')=='OUTWARD')
 assert len(launches)==1 and not v.rate_box.isEnabled() and v.hardware_status['simulated']
 assert not v.start_when_ready and not v.execute_button.isEnabled()
 wait_for(lambda:v.hardware_status.get('phase')=='RELAXED')
 assert v.hardware_status['disabled'] and v.hardware_status['safe_to_close'] and not v.viewport.errors
 events=[json.loads(x) for x in (Path(v.client_info['folder'])/'events.jsonl').read_text().splitlines()]
 phases=[x['phase'] for x in events]
 assert 'VERIFIED_START_POSITION' in phases and phases[-1]=='RELAXED'
 enter=next(x['monotonic'] for x in events if x['phase']=='GOAL_HOLD')
 leave=next(x['monotonic'] for x in events if x['phase']=='RETURN')
 assert leave-enter>=2 and all(j.closed for j in jobs) and not list(folder.rglob('*review*'))
 v.viewport.save_window(v.window,d/'direct_ui.png')
 atomic_json(d/'direct_ui_report.json',dict(passed=True,can_access=False,physical_motor_commands=0,
  checks=['no_modal_review_or_typed_phrase','preview','speed','cancel_pending_start','optional_readiness_without_motion',
   'expired_readiness_automatically_refreshed','one_click_start_after_checks','no_duplicate_start',
   'outward_goal_2s_hold_reverse_settle_release'],jobs=[j.action for j in jobs],launches=launches,
  status=v.hardware_status,events=events,render=v.report()))
 print('DIRECT START MOCK UI PASSED',v.hardware_status,flush=True)
finally:
 if v.active and v.client:
  v.emergency_hold()
  for _ in range(25):v.poll();time.sleep(.01)
  v.send('supported_exit',confirmation='ARM PHYSICALLY SUPPORTED')
  for _ in range(25):v.poll();time.sleep(.01)
 v.close()
