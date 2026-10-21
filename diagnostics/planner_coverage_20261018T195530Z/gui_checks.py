from pathlib import Path
import hashlib,json,socket,sys,time
from types import SimpleNamespace
ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import common,planned_view,alignment_view,snapshot_preview,depth
from drum_planner.trajectory_view import PlanDisplay
D=Path(__file__).parent
attempts=[];original=socket.socket

def guarded(family=socket.AF_INET,*a,**kw):
 if family==socket.AF_CAN:attempts.append('CAN');raise AssertionError('No CAN in visualization checks')
 return original(family,*a,**kw)
socket.socket=guarded
original_camera=depth.DepthCamera

def denied(*a,**kw):raise AssertionError('Saved displays must not open the camera')
depth.DepthCamera=denied;snapshot_preview.DepthCamera=denied
Base=planned_view.SnapshotPlaybackView
class Blocked(Base):
 def poll(self,*a,**kw):
  result=super().poll(*a,**kw)
  if self.packet is not None and not getattr(self,'checked',False):
   assert not self.play_button.isEnabled() and not self.timeline.isEnabled()
   self.toggle_play();assert not self.clock.playing
   assert 'START: 11/11' in self.plan_label.text() and 'GOAL: 11/11' in self.plan_label.text()
   for mode in (1,2,0):self.mode.setCurrentIndex(mode)
   self.focus_arm();self.save_view();self.checked=True
  return result
planned_view.SnapshotPlaybackView=Blocked
reports={}
reports['blocked_matching_scene']=planned_view.show(D/'live_current_pose/plan/plan.json',seconds=2)
reports['measured_alignment']=alignment_view.show(D/'live_hardware_check',seconds=2)
# Restore actual depth startup ONLY for the explicitly camera-only playback test.
depth.DepthCamera=original_camera
holder=[]
def tracked_camera(*a,**kw):
 c=original_camera(*a,**kw);holder.append(c);return c
snapshot_preview.DepthCamera=tracked_camera
folder=D/'live_snapshot_playback'
checks={}
def fingerprint(v):
 h=hashlib.sha256()
 for a in (v.mesh.vertices,v.mesh.faces,v.packet.raw_image,v.packet.raw_points):h.update(a.tobytes())
 return h.hexdigest()
class Controls(snapshot_preview.SnapshotPlaybackView):
 def set_scene(self,packet,mesh):
  c=holder[0];assert c.count==1 and not c.thread.is_alive() and c.process.poll()==0
  super().set_scene(packet,mesh)
  self.before=fingerprint(self);self.started=time.monotonic();self.steps=set()
  self.files_before={p.name:common.digest(p) for p in folder.iterdir() if p.name in ('scene.json','surface_mesh.npz','raw_depth.npz')}
  checks['camera_released_before_mesh']=True
  self.window.activateWindow();self.window.setFocus();self.app.processEvents()
  self.clock.pause();self.idle_draws=self.animation_draws
 def poll(self,*a,**kw):
  key=super().poll(*a,**kw)
  if self.packet is None:return key
  c=holder[0];assert c.count==1 and not c.thread.is_alive() and c.process.poll()==0
  assert self.worker.submissions==0 and self.mesh_builds==1
  elapsed=time.monotonic()-self.started
  if elapsed>.5 and 'controls' not in self.steps:
   self.steps.add('controls');assert self.animation_draws==self.idle_draws
   checks['paused_idle_no_redraw']=True
   for mode in (1,2,0):
    self.mode.setCurrentIndex(mode);assert self.cloud.get_visible()==(mode!=0)
    assert self.surface.get_visible()==(mode!=1)
   self.color.setCurrentIndex(1);self.reset_view();self.focus_arm()
   self.ax.view_init(elev=20,azim=-40);self.zoom(SimpleNamespace(inaxes=self.ax,button='up'))
   assert self.ax.elev==20 and self.ax.azim==-40
   self.timeline.setValue(2000);assert self.clock.position==2 and not self.clock.playing
   goal=self.plan.obj['outward'][-1]['time_s'];self.seek(goal)
   import numpy as np
   assert np.array_equal(self.robot.last_positions,self.plan.req['goal'])
   self.seek(goal+1);assert self.plan.phase(goal+1)=='HOLD (2 seconds)'
   self.seek(self.plan.duration);assert np.array_equal(self.robot.last_positions,self.plan.req['start'])
   self.seek(0);self.speed.setCurrentIndex(2);self.loop_box.setChecked(True)
   from PyQt5 import QtCore,QtTest
   QtTest.QTest.keyClick(self.window,QtCore.Qt.Key_P);self.app.processEvents()
   assert self.clock.playing
   QtTest.QTest.keyClick(self.window,QtCore.Qt.Key_Space);self.app.processEvents()
   assert not self.freeze_button.isEnabled() and not self.frozen
   self.request_save();checks['controls_layers_color_fit_rotate_zoom_seek_speed_loop_keyboard']=True
  if self.clock.completed_cycles>=1 and 'cycle' not in self.steps:
   self.steps.add('cycle');assert fingerprint(self)==self.before
   assert self.files_before=={name:common.digest(folder/name) for name in self.files_before}
   self.save_view();checks['full_simulated_cycle_and_immutable_scene']=True
  return key
snapshot_preview.run(folder,PlanDisplay(D/'demo/plan.json'),seconds=12,viewer_factory=Controls)
assert len(checks)==4 and all(checks.values()) and not attempts
reports['camera_snapshot_playback']=json.loads((folder/'capture_report.json').read_text())
reports['checks']=checks;reports['can_attempts']=attempts;reports['passed']=True
common.atomic_json(D/'gui_checks.json',reports)
print('PASS: blocked matching-scene display, measured alignment, real-camera frozen mesh/playback and controls; zero CAN attempts')
