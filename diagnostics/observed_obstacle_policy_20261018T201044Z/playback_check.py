import hashlib,json,socket,sys,time
from pathlib import Path
import numpy as np
ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import common,depth,planned_view,snapshot_preview
from drum_planner.trajectory_view import PlanDisplay
D=Path(__file__).parent;path=D/'live_current_pose/plan/plan.json'
plan=PlanDisplay(path);assert plan.playable and plan.obj['collision_policy']=='observed_obstacles'
assert not plan.obj['strict_unknown_space'];attempts=[];old=socket.socket

def guarded(family=socket.AF_INET,*a,**kw):
 if family==socket.AF_CAN:attempts.append('CAN');raise AssertionError('Playback cannot access CAN')
 return old(family,*a,**kw)
socket.socket=guarded

def denied(*a,**kw):attempts.append('camera');raise AssertionError('Matching-scene playback cannot reopen the camera')
depth.DepthCamera=denied;snapshot_preview.DepthCamera=denied
checks={};holder=[]
def fingerprint(v):
 h=hashlib.sha256()
 for a in (v.mesh.vertices,v.mesh.faces,v.packet.raw_points,v.packet.raw_image):h.update(a.tobytes())
 return h.hexdigest()
Base=planned_view.SnapshotPlaybackView
class View(Base):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw);holder.append(self);self.controlled=False;self.finished=False
 def set_scene(self,packet,mesh):
  super().set_scene(packet,mesh);self.before=fingerprint(self)
 def poll(self,*a,**kw):
  key=super().poll(*a,**kw)
  if self.packet is None:return key
  if not self.controlled:
   self.controlled=True
   assert self.play_button.isEnabled() and self.timeline.isEnabled()
   assert 'DETECTED-OBSTACLE PLAN' in self.plan_label.text()
   assert not self.freeze_button.isEnabled()
   for m in (1,2,0):self.mode.setCurrentIndex(m)
   self.color.setCurrentIndex(1);self.reset_view();self.focus_arm()
   goal=self.plan.obj['outward'][-1]['time_s'];self.seek(goal)
   assert np.array_equal(self.robot.last_positions,self.plan.req['goal'])
   self.seek(goal+1);assert self.plan.phase(goal+1)=='HOLD (2 seconds)'
   assert np.array_equal(self.robot.last_positions,self.plan.req['goal'])
   self.seek(self.plan.duration);assert np.array_equal(self.robot.last_positions,self.plan.req['start'])
   self.save_view();(self.folder/'preview_view.png').rename(D/'goal_return_controls.png')
   self.seek(0);self.speed.setCurrentIndex(2);self.play_button.click();assert self.clock.playing
   self.play_button.click();assert not self.clock.playing
   self.play_button.click();assert self.clock.playing
   checks['controls_endpoint_hold_return']=True
  if self.clock.completed_cycles and not self.finished:
   self.finished=True;assert not self.clock.playing
   assert np.array_equal(self.robot.last_positions,self.plan.req['start'])
   assert self.phases_seen>={'outward','HOLD (2 seconds)','return'}
   assert fingerprint(self)==self.before and self.mesh_builds==1 and self.worker.submissions==0
   self.save_view();checks['complete_wall_clock_cycle']=True;checks['frozen_matching_scene']=True
   return ord('q')
  return key
planned_view.SnapshotPlaybackView=View
report=planned_view.show(path,seconds=plan.duration/2+10)
assert len(checks)==3 and all(checks.values()) and not attempts
common.atomic_json(D/'playback_check.json',dict(passed=True,checks=checks,device_access_attempts=attempts,report=report))
print('PASS: actual saved current-pose plan plays full outward / 2-second hold / reverse; matched frozen mesh; no devices')
