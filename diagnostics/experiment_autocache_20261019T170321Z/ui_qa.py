"""Native UI + actual Gemini snapshot; saved arm state, never CAN/motors."""
import json,time
from pathlib import Path
from PyQt5 import QtWidgets,QtCore,QtTest
from drum_planner.experiment.app import prepare,make_view_class
from drum_planner.experiment.config import METHODS,no_can
from drum_planner.common import atomic_json
no_can();D=Path(Path('/tmp/x5_autocache_task').read_text().strip());session=D/'actual_camera'
prepare(session)
v=make_view_class()(session,autoplay=True);checks={};started=time.monotonic()
def pump(seconds=.1):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  key=v.poll()
  if key==ord('s'):v.save_view()
  time.sleep(.01)
def wait_idle(limit=150):
 end=time.monotonic()+limit
 while v.job or v.pending_methods:
  pump()
  assert time.monotonic()<end,'Automatic queue did not finish'
def click(button):
 QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton);pump()
def cancel():
 click(v.cancel_button);wait_idle(12)
 assert not v.job and not v.pending_methods
try:
 assert v.job_method=='clearance' and len(v.pending_methods)==8
 assert v.method_box.isEnabled() and not v.settings_button.isEnabled()
 checks['startup_immediate_queue']=True
 seen=set();background_playback=False;completed_while_planning=False;initial_clock=None
 while v.job or v.pending_methods:
  pump()
  assert time.monotonic()-started<1000
  assert not v.viewport.errors
  assert v.method_box.currentData()=='clearance','Background job changed selected method'
  if v.job_method:seen.add(v.job_method)
  if v.cached('clearance') and v.job and v.job_method!='clearance' and v.plan.playable:
   assert v.play_button.isEnabled() and v.timeline.isEnabled()
   if initial_clock is None:initial_clock=v.clock
   assert v.clock is initial_clock,'Background result reset playback'
   background_playback |= v.clock.playing
   completed_while_planning |= v.clock.completed_cycles>=1
   if v.clock.playing and not (D/'background_playback.png').exists():v.viewport.save_window(v.window,D/'background_playback.png')
 assert len(v.cache)==9 and v.generation_count==9 and set(v.cache)==set(METHODS)
 assert all(state in ('ready','no feasible route') for state in v.method_states.values())
 assert not v.viewport.errors and v.viewport.surface.geometry_uploads==1
 assert v.active_method==v.displayed_method=='clearance'
 assert background_playback and completed_while_planning
 checks['all_nine_cached_without_any_selection']=True
 checks['complete_playback_while_background_planning']=True
 atomic_json(D/'automatic_all_report.json',v.report())
 for method in METHODS:
  count=v.generation_count;v.method_box.setCurrentIndex(list(METHODS).index(method));pump()
  assert v.displayed_method==method and not v.job and v.generation_count==count
  assert v.play_button.isEnabled()==v.plan.playable
  assert v.plan.obj['experiment']['method']==method
 checks['cached_selection_does_not_regenerate']=True
 # Explicitly regenerate, cancel, then resume only the missing entry.
 v.method_box.setCurrentIndex(list(METHODS).index('clearance'));pump();click(v.generate_button)
 assert v.job_method=='clearance' and len(v.cache)==8
 cancel();assert len(v.cache)==8
 v.autoplay_result=False
 ready=next(k for k in v.cache if json.loads(v.cache[k][1].read_text())['status']=='planned')
 v.method_box.setCurrentIndex(list(METHODS).index(ready));pump();v.seek(0);v.speed.setCurrentIndex(2)
 click(v.play_button);clock=v.clock;assert clock.playing
 before=v.generation_count;click(v.cache_all_button)
 assert v.job_method=='clearance' and not v.pending_methods
 wait_idle();assert v.generation_count==before+1 and len(v.cache)==9
 assert v.active_method==ready and v.clock is clock and v.clock.completed_cycles>=1
 checks['cancel_and_resume_only_missing_methods']=True
 checks['background_result_keeps_other_selected_cached_plan']=True
 # New settings automatically invalidate/requeue all, with no Generate click.
 v.budget_spin.setValue(10);pump()
 assert not v.cache and v.job is not None and len(v.pending_methods)==8 and v.job_signature[-1]==10
 cancel();checks['budget_change_requeues_all']=True
 def accept_settings():
  dialog=QtWidgets.QApplication.activeModalWidget();assert dialog
  combo=dialog.findChild(QtWidgets.QComboBox);combo.setCurrentIndex(1)
  buttons=dialog.findChild(QtWidgets.QDialogButtonBox)
  QtTest.QTest.mouseClick(buttons.button(QtWidgets.QDialogButtonBox.Ok),QtCore.Qt.LeftButton)
 QtCore.QTimer.singleShot(100,accept_settings);click(v.settings_button)
 assert v.params['depth_mm']==1 and v.job and len(v.pending_methods)==8
 cancel();checks['uncertainty_change_requeues_all']=True
 # Explicit real recapture replaces the frozen scene, then automatically starts
 # all methods again. Cancel once that startup has been verified.
 oldscene=v.context['scene'];uploads=v.viewport.surface.geometry_uploads
 click(v.recapture_button);deadline=time.monotonic()+70
 while v.context['scene']==oldscene or v.job_kind!='plan':
  pump();assert time.monotonic()<deadline
 assert v.scene_generation==1 and not v.cache and len(v.pending_methods)==8
 assert v.viewport.surface.geometry_uploads==uploads+1
 assert json.loads(Path(v.context['scene']).read_text())['camera_released']
 cancel();checks['recapture_requeues_all_and_camera_stops']=True
 assert not v.viewport.errors
 atomic_json(D/'ui_qa.json',dict(passed=True,checks=checks,automatic_all_report=str(D/'automatic_all_report.json'),final=v.report()))
 v.viewport.save_window(v.window,D/'final_ui.png')
 print('AUTOCACHE UI QA PASSED',json.dumps(checks),flush=True)
finally:v.close()
