"""Real CLI, actual encoders/depth, native planner, non-motion GUI features.
GUI denied CAN; every controller-launch path is forbidden. Only the existing
query-only capture worker can open CAN through its exact packet allow-list.
"""
import json,sys,time
from pathlib import Path
import numpy as np
from PyQt5 import QtCore,QtTest,QtWidgets
from drum_planner.baseline import app,service,transport,tasks
from drum_planner.common import atomic_json,DEFAULT_CALIBRATION
from drum_planner.experiment.config import no_can
from drum_planner.cli import main
no_can()
d=Path(sys.argv[1]).resolve();d.mkdir(parents=True,exist_ok=True);session=d/'actual_startup';attempts=[]
def forbidden(*a,**kw):
 attempts.append('FORBIDDEN');raise AssertionError('No physical controller or motors may be created')
service.launch=forbidden;transport.BaselineMotors=forbidden
QtWidgets.QInputDialog.getText=forbidden;QtWidgets.QMessageBox.question=forbidden
Original=app.View
class QAView(Original):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw);self.qa_stage='preview';self.qa_started=time.monotonic();self.qa_steps=[]
  assert not self.simulated and self.plan.playable and self.client is None and not self.active
  tasks.load_checked(self.plan.path,DEFAULT_CALIBRATION)
  assert self.execute_button.isEnabled() and self.prepare_button.isEnabled()
  assert not self.relax_button.isEnabled() and not self.reverse_button.isEnabled() and not self.support_button.isEnabled()
  print('LIVE QA: actual startup succeeded; testing preview and read-only readiness. No physical controller permitted.',flush=True)
 def poll(self,*a,**kw):
  key=super().poll(*a,**kw)
  if self.qa_stage=='preview':
   self.qa_stage='animation'
   self.hold_button.click();self.reverse_button.click();self.relax_button.click();self.support_button.click()
   assert self.client is None and not self.active and not attempts
   for mode in range(self.mode.count()):
    self.mode.setCurrentIndex(mode)
    for color in range(self.color.count()):self.color.setCurrentIndex(color);self.app.processEvents()
   self.mode.setCurrentIndex(0);self.color.setCurrentIndex(0)
   self.viewport.set_angles(35,-30);self.draw_canvas();self.reset_view();self.focus_arm()
   self.rate_box.setValue(.5)
   assert np.isclose(self.plan.duration,2*self.plan.obj['outward'][-1]['time_s']+2.)
   self.seek(self.plan.obj['outward'][-1]['time_s']);assert np.allclose(self.plan.positions(self.clock.position),self.plan.req['goal'],atol=1e-10)
   self.timeline.setValue(0);self.rate_box.setValue(1.);self.save_button.click()
   self.window.activateWindow();self.window.setFocus();self.app.processEvents()
   QtTest.QTest.keyClick(self.window,QtCore.Qt.Key_P);self.app.processEvents();assert self.clock.playing
   self.qa_steps+=['actual_encoder_and_five_depth_frame_capture','native_baseline_candidates','joint_model_scene_integrity',
    'disabled_motion_controls','idle_hold_no_motor_commands','mesh_points_layers','depth_height_colors','orbit_fit_reset',
    'speed_scale','goal_seek','timeline','save_view','keyboard_preview_play']
  elif self.qa_stage=='animation' and self.clock.position>=.3:
   self.qa_steps.append('repeat_launch_short_animation_check')
   self.prepare_button.click();assert self.job and not self.start_when_ready
   self.qa_stage='readiness'
  elif self.qa_stage=='readiness' and self.job is None:
   assert self.prepared is not None and not self.last_error,self.last_error
   packet,curve=tasks.check_packet(self.prepared)
   assert packet['proof']['all_valid'] and not self.active and self.client is None and not self.start_when_ready
   assert not attempts and not self.viewport.errors
   self.qa_steps+=['readiness_actual_fresh_capture','native_exact_curve_collision_zone_check','prepared_integrity_check',
    'readiness_does_not_start_motors','frozen_scene_replaced_once','clean_close']
   self.save_view();assert (self.session/'preview_view.png').exists()
   self.viewport.save_window(self.window,d/'live_complete.png')
   atomic_json(d/'live_ui_report.json',dict(passed=True,checks=self.qa_steps,motor_commands=0,controller_launch_attempts=attempts,
    prepared=str(self.prepared),plan=str(self.plan.path),proof=packet['proof'],render=self.report()))
   self.qa_stage='done';self.close_button.click()
   print('LIVE QA PASSED: actual camera/encoders + native planning + preview + read-only readiness; zero physical controller launches.',flush=True)
  if self.last_error:raise AssertionError(self.last_error)
  if time.monotonic()-self.qa_started>150:raise AssertionError('Live non-motion UI QA timeout')
  return key
app.View=QAView
code=main(['baseline','--output',str(session)])
assert code==0 and not attempts,(code,attempts)
