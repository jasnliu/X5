"""Actual CLI startup + real query-only capture + native planning + scripted GUI.

The GUI process is kernel-denied CAN. Only the existing audited capture worker
can query CAN. No controller launch, authorization, or physical-review approval.
"""
import json,runpy,sys,time
from pathlib import Path
import numpy as np
from PyQt5 import QtCore,QtTest,QtWidgets
from drum_planner.baseline import app,service,transport,tasks
from drum_planner.common import atomic_json,DEFAULT_CALIBRATION
from drum_planner.experiment.config import no_can
no_can()
d=Path(sys.argv[1]).resolve();session=d/'actual_startup';attempts=[];saved_plan=Path(sys.argv[2]).resolve() if len(sys.argv)>2 else None
def forbidden(*a,**k):
 attempts.append('FORBIDDEN_CONTROLLER_OR_MOTORS');raise AssertionError('This QA run must not create a physical controller')
service.launch=forbidden;transport.BaselineMotors=forbidden
OriginalView=app.View
class QAView(OriginalView):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw);self.qa_started=time.monotonic();self.qa_done=False
  assert not self.simulated and not self.active and self.client is None
  assert self.plan.playable and self.plan.path==(saved_plan or session/'plan/result/plan.json')
  tasks.load_checked(self.plan.path,DEFAULT_CALIBRATION)
  assert self.prepare_button.isEnabled() and not self.execute_button.isEnabled()
  assert not self.reverse_button.isEnabled() and not self.support_button.isEnabled()
  self.qa_stages=(['actual_query_capture','native_candidate_planning'] if saved_plan is None else ['saved_plan_real_mode_view_only'])+['load_checked','real_mode_preview_controls']
  print('QA: fresh startup reached the real-mode preview; physical controller launch is forbidden.',flush=True)
 def poll(self,*a,**kw):
  key=super().poll(*a,**kw)
  if not hasattr(self,'qa_staged'):
   self.qa_staged=True
   # Open the genuine physical-review dialog and CANCEL. Do not assert facts
   # about physical support, mounting or calibration we have not verified.
   QtCore.QTimer.singleShot(0,lambda:QtWidgets.QApplication.activeModalWidget().reject())
   self.prepare_button.click()
   assert self.review is None and self.job is None and self.prepared is None and not self.locked
   self.execute_clicked();self.reverse_clicked();self.hold_button.click()
   assert self.client_info is None and not self.active and not attempts
   self.qa_stages+=['physical_review_cancel','unprepared_play_blocked','idle_hold_no_commands','idle_reverse_blocked']
   for mode in range(self.mode.count()):
    self.mode.setCurrentIndex(mode)
    for color in range(self.color.count()):self.color.setCurrentIndex(color);self.app.processEvents()
   self.mode.setCurrentIndex(0);self.color.setCurrentIndex(0)
   self.viewport.set_angles(35,-30);self.draw_canvas();self.reset_view();self.focus_arm()
   self.rate_box.setValue(.5)
   assert np.isclose(self.plan.duration,2*self.plan.obj['outward'][-1]['time_s']+2.)
   self.seek(self.plan.obj['outward'][-1]['time_s'])
   assert np.allclose(self.plan.positions(self.clock.position),self.plan.req['goal'],atol=1e-10)
   self.timeline.setValue(0);self.rate_box.setValue(1.)
   self.save_button.click()
   self.window.activateWindow();self.window.setFocus();self.app.processEvents()
   QtTest.QTest.keyClick(self.window,QtCore.Qt.Key_P);self.app.processEvents()
   assert self.clock.playing
   self.qa_stages+=['layers','colors','view_rotation','reset_fit','speed_scale','goal_seek','timeline','save_view','keyboard_play']
  if self.clock.completed_cycles>=1 and not self.qa_done:
   self.qa_done=True
   assert {'outward','HOLD (2 seconds)','return'}.issubset(self.phases_seen),self.phases_seen
   assert not self.viewport.errors and not self.last_error and not attempts
   assert self.mesh_builds==1 and self.client is None and self.prepared is None
   assert (self.session/'preview_view.png').is_file()
   self.viewport.save_window(self.window,d/'actual_startup_complete.png')
   atomic_json(d/'live_ui_report.json',dict(passed=True,checks=self.qa_stages+['complete_animation','no_physical_controller','clean_close'],
       physical_motor_commands=0,controller_launch_attempts=attempts,fresh_startup=saved_plan is None,rendering=self.report()))
   self.close_button.click()
   print('QA: full live-startup preview cycle and read-only GUI controls PASSED. No physical controller started.',flush=True)
  if time.monotonic()-self.qa_started>120:raise AssertionError('Timed out during preview-only GUI QA')
  return key
app.View=QAView
sys.argv=['plan_drums.sh','baseline','--output',str(session)]
try:
 if saved_plan is None:runpy.run_module('drum_planner.cli',run_name='__main__')
 else:app.show(saved_plan,session,DEFAULT_CALIBRATION)
finally:assert not attempts,attempts
