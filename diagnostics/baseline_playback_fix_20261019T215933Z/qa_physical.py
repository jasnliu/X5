"""Authorized real baseline UI test: Start cycle, optionally Hold/Reverse.
NEVER clicks manual Relax or disables on an error. If recovery fails, preserve
UI heartbeat and owner indefinitely so the operator can inspect the arm.
"""
import importlib.util,json,sys,time,subprocess,os
from pathlib import Path
from drum_planner.baseline import app
from drum_planner.common import atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.cli import main
no_can() # GUI cannot send motor packets; normal independent owner does playback.
d=Path(sys.argv[1]).resolve();d.mkdir(parents=True,exist_ok=True)
trial=sys.argv[2] if len(sys.argv)>2 else 'normal';rate=float(sys.argv[3]) if len(sys.argv)>3 else 1.
notify_path=Path.home()/'.local/bin/codex-ntfy.py'
spec=importlib.util.spec_from_file_location('motion_ntfy',notify_path);ntfy=importlib.util.module_from_spec(spec);spec.loader.exec_module(ntfy)
Original=app.View
class QAView(Original):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw);self.qa_stage='waiting';self.qa_deadline=0.;self.qa_phase=None;self.qa_last_error=None
  self.qa_return_attempted=False;self.qa_hold=False;self.qa_fault_at=None;self.qa_last_status=0.;self.qa_summary=False
  print('PHYSICAL TEST UI READY; waiting for authorized test-start file.',flush=True)
 def notify(self,body):
  ok=ntfy.post_ntfy(body,'Robot physical test','warning,robot')
  with (d/'notifications.jsonl').open('a') as f:f.write(json.dumps(dict(time=time.time(),body=body,accepted=ok))+'\n')
  if not ok:raise RuntimeError('Motion notification was not accepted; no Start click')
  ntfy.play_sound('{"type":"robot-test"}')
 def poll(self,*a,**kw):
  key=super().poll(*a,**kw);now=time.monotonic();s=self.hardware_status;phase=s.get('phase')
  if phase!=self.qa_phase:
   self.qa_phase=phase;print('PHYSICAL PHASE',json.dumps(s),flush=True)
   if phase in ('HELD','FAULT_HOLD'):self.qa_fault_at=now
  if self.last_error and self.last_error!=self.qa_last_error:
   self.qa_last_error=self.last_error;print('UI ERROR',self.last_error,flush=True)
  if self.qa_stage=='waiting' and (d/'start_authorized').exists():
   self.notify('Physical robot test starts in 10 seconds. Keep clear. The arm will return before relaxing.')
   self.qa_deadline=now+10;self.qa_stage='countdown'
  elif self.qa_stage=='countdown' and now>=self.qa_deadline:
   self.execute_button.click();self.qa_stage='running'
   print('Authorized START REAL ARM button clicked.',flush=True)
  if trial=='hold_return' and phase=='OUTWARD' and s.get('tau',0)>=1.5 and not self.qa_hold:
   self.hold_button.click();self.qa_hold=True;print('Authorized EMERGENCY HOLD button clicked.',flush=True)
  if phase in ('HELD','FAULT_HOLD') and self.qa_fault_at and now-self.qa_fault_at>.8 and not self.qa_return_attempted:
   error=(s.get('error','')+' '+s.get('feedback_error','')).lower()
   if any(word in error for word in ('stall','too hot','overtemperature','motor 1: fault','motor 2: fault','motor 3: fault','motor 4: fault','motor 5: fault','motor 6: fault','motor 7: fault','motor 8: fault')):
    self.qa_return_attempted=True;self.qa_stage='manual_attention'
    self.notify('Robot test stopped with a drive fault. No relaxation was commanded. Check the robot; manual attention required.')
   elif s.get('can_return'):
    self.qa_return_attempted=True;self.reverse_button.click()
    print('Authorized reverse traveled path requested; no midair relaxation.',flush=True)
  request=d/'request.json'
  if request.exists():
   cmd=json.loads(request.read_text());request.unlink();op=cmd['op']
   if op=='reverse':self.reverse_button.click()
   elif op=='hold':self.hold_button.click()
   elif op=='retry_hold':self.retry_hold_button.click()
   elif op=='close' and not self.active:self.close_button.click()
   else:print('Unimplemented/unsafe QA request',op,flush=True)
  if phase=='RELAXED' and not self.qa_summary:
   self.qa_summary=True;self.qa_stage='complete';self.viewport.save_window(self.window,d/'physical_complete.png')
   atomic_json(d/'result.json',dict(passed=not self.qa_return_attempted if trial=='normal' else self.qa_hold,
       trial=trial,status=s,controller=self.client_info,plan=str(self.plan.path),last_error=self.last_error))
   print('PHYSICAL TEST COMPLETE: returned to start and disabled feedback confirmed.',flush=True)
   self.close_button.click()
  if now-self.qa_last_status>.2:
   self.qa_last_status=now;atomic_json(d/'ui_status.json',dict(stage=self.qa_stage,hardware=s,error=self.last_error,controller=self.client_info,plan=str(self.plan.path)))
  return key
app.View=QAView
code=main(['baseline','--output',str(d/'session'),'--speed',str(rate)])
atomic_json(d/'process_result.json',dict(exit_code=code))
raise SystemExit(code)
