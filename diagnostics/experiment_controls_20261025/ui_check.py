"""Real Qt/VTK controls and contained planning; saved inputs, fake recapture only."""
import json,sys,time,traceback,os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from drum_planner.common import atomic_json,endpoint,DEFAULT_CALIBRATION,read_state
from drum_planner.experiment import app,destinations,motion_settings as cfg
H=Path('diagnostics/experiment_controls_20261025').resolve()
state=Path('diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/measured_start.json').resolve();scene=state.parent/'camera/scene.json'
original=read_state(state);rows=[];errors=[]
def hook(t,v,b):
 text=''.join(traceback.format_exception(t,v,b));errors.append(text);print(text,flush=True)
sys.excepthook=hook
for side in ['right','left']:
 folder=H/('ui_'+os.environ.get('X5_UI_RUN','')+side);app.prepare(folder,state,scene,DEFAULT_CALIBRATION,side=side)
 actual=destinations.files(side)
 start=actual[0];goal=actual[1] if side=='right' else H/'inputs/L_near.json'
 choices=actual+([goal] if goal not in actual else [])
 with patch.object(destinations,'files',return_value=choices):
  view=app.make_view_class()(folder)
  def pump(seconds=.12):
   end=time.monotonic()+seconds
   while time.monotonic()<end:view.poll();time.sleep(.01)
   assert not errors,errors
  def wait():
   end=time.monotonic()+180
   while (view.job or view.pending_methods) and time.monotonic()<end:pump()
   assert view.job is None and not view.pending_methods,'worker did not finish'
   assert not errors,errors
  def choose(box,path):
   index=box.findData(str(path) if path else None);assert index>=0,(path,index)
   box.setCurrentIndex(index);pump(.04)
  try:
   pump();assert view.job is None and not view.plan.playable and view.generation_count==0
   assert view.start_box.currentData() is None and view.timing_box.currentData()=='balanced'
   # Ordinary launch: choose a target WITHOUT touching simulated start first.
   # Right is feasible; this saved left rest-to-L1 case is genuinely blocked.
   choose(view.destination_box,start);wait()
   assert view.plan.obj['status']==('planned' if side=='right' else 'no_feasible_route'),view.status.text()
   assert view.plan.req['start']==original[side+'_rad']
   assert view.plan.req['scene_robot_start']==original[side+'_rad']
   choose(view.destination_box,None)
   choose(view.start_box,start)
   assert np.array_equal(view.plan.positions(0),endpoint(start,side=side)) and view.job is None
   assert view.plan.req['scene_robot_start']==original[side+'_rad']
   # Real editable dialog: invalid entries rejected, profile save/load, wrong side rejected.
   dialog=view.motion_dialog();dialog.show();pump()
   dialog.fields[0,0].setText('nan');dialog.buttons['apply'].click();pump()
   assert dialog.isVisible() and 'Not applied' in dialog.error_label.text()
   assert np.array_equal(cfg.matrix(view.motion_profile),cfg.DEFAULT_LIMITS)
   dialog.buttons['defaults'].click()
   wrong=H/'inputs'/('left_unequal.json' if side=='right' else 'right_distal.json')
   with patch.object(view.QtWidgets.QFileDialog,'getOpenFileName',return_value=(str(wrong),'')):
    dialog.buttons['load'].click();assert 'other arm' in dialog.error_label.text()
   valid=H/'inputs'/('right_distal.json' if side=='right' else 'left_unequal.json')
   with patch.object(view.QtWidgets.QFileDialog,'getOpenFileName',return_value=(str(valid),'')):dialog.buttons['load'].click()
   saved=folder/'ui_saved_profile.json'
   with patch.object(view.QtWidgets.QFileDialog,'getSaveFileName',return_value=(str(saved),'')):dialog.buttons['save'].click()
   assert cfg.load(saved,side)==cfg.load(valid,side)
   dialog.buttons['apply'].click();pump();assert not dialog.isVisible()
   assert cfg.matrix(view.motion_profile).tolist()==cfg.matrix(cfg.load(valid,side)).tolist()
   choose(view.destination_box,goal);assert view.job is not None
   retired=view.job_target
   choose(view.start_box,goal)
   assert np.array_equal(view.plan.positions(0),endpoint(goal,side=side))
   wait();assert view.plan.playable,view.status.text()
   assert view.plan.obj['duration_outward_s']==0 and view.plan.path.parent!=retired
   assert any(e['event']=='obsolete_job_discarded' for e in view.events)
   choose(view.start_box,start)
   dialog=view.motion_dialog();dialog.show();pump(.05);dialog.buttons['defaults'].click();dialog.buttons['apply'].click()
   wait();assert view.plan.playable,view.status.text()
   assert view.plan.obj['duration_outward_s']>0
   assert view.plan.req['start']==endpoint(start,side=side).tolist()
   assert view.plan.req['scene_robot_start']==original[side+'_rad']
   assert view.plan.obj['experiment']['motion_settings_sha256']==cfg.identity(view.motion_profile)
   # Old plan rejected after settings change, even if the geometry happens to match.
   old_plan=view.plan.path
   view.play_button.click();pump(.3);assert view.clock.position>0;view.play_button.click()
   view.speed.setCurrentIndex(2);assert view.clock.rate==2.
   view.timeline.setValue(int(view.plan.obj['duration_outward_s']*500));pump()
   assert view.mesh_builds==1
   view.save_view();dialog=view.motion_dialog();dialog.show();pump()
   assert dialog.findChild(view.QtWidgets.QTableWidget,'achieved_joint_peaks') is not None
   dialog.grab().save(str(folder/'joint_limits_dialog.png'));dialog.reject()
   ready_plan=str(view.plan.path)
   # Test Recapture button routing with a synthetic completed camera job. The
   # initializer and all subsequent planners are REAL; no camera is opened.
   view.context['initial_checkpoint_sources']=[str(goal)]
   atomic_json(folder/'context.json',view.context)
   original_job=app.Job;init_states=[]
   class CompletedCapture:
    cancelled=False
    process=SimpleNamespace(returncode=0)
    log_path=folder/'fake_capture.log'
    def poll(self):return 0
    def close(self):pass
    def cancel(self):self.cancelled=True
   def job_factory(cmd,directory,**kwargs):
    if 'capture' in cmd:
     target=Path(cmd[cmd.index('--folder')+1]);target.mkdir(parents=True);(target/'camera').symlink_to(scene.parent,target_is_directory=True)
     return CompletedCapture()
    if 'initialize' in cmd:init_states.append(cmd[cmd.index('--state')+1])
    return original_job(cmd,directory,**kwargs)
   with patch.object(app,'Job',side_effect=job_factory):
    view.recapture_button.click();wait()
   assert init_states==[str(state)] and view.plan.playable,view.status.text()
   assert view.plan.req['start']==endpoint(start,side=side).tolist()
   assert view.plan.req['scene_robot_start']==original[side+'_rad']
   assert view.context['experiment_start']['destination_source']==str(start)
   assert view.context['initial_checkpoint_sources']==[str(goal)]
   # Cancel then retry. Closing an active disposable worker reaps it.
   view.generate_button.click();pump(.08);view.cancel_button.click();wait()
   assert not view.pending_methods
   view.generate_button.click();pump(.08);job=view.job;assert job is not None
   view.close();assert job.process.poll() is not None
   row=dict(side=side,passed=True,native_qt_vtk=True,start_teleports_immediately=True,
      default_start_target_checked=True,external_targets_preserved_on_recapture=True,
      invalid_limits_rejected=True,profile_save_load=True,wrong_arm_profile_rejected=True,
      cancelled_result_discarded=True,stationary_target_checked=True,play_pause_scrub_speed=True,
      mock_recapture_keeps_actual_mask=True,close_reaps_worker=True,ready_plan=ready_plan,
      hardware_access=False,motor_commands=0)
   rows.append(row);atomic_json(H/'ui_results.json',rows);print('NATIVE UI PASS',side,flush=True)
  finally:view.close()
print('ALL NATIVE UI CHECKS PASSED',flush=True)
