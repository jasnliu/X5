import sys,time,socket,json,hashlib,shutil
from pathlib import Path
import numpy as np
ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from PyQt5 import QtCore,QtGui,QtWidgets,QtTest
from drum_planner import depth,snapshot_preview,planned_view,common
from drum_planner.trajectory_view import PlanDisplay
D=Path(__file__).parent;folder=D/'native_playback';folder.mkdir(exist_ok=True)
src=D/'final_saved_scene_plan'
for file in ('plan.json','backend_request.json'):shutil.copy2(src/file,folder/file)
old=socket.socket;attempts=[]
def guard(family=socket.AF_INET,*a,**kw):
 if family==socket.AF_CAN:attempts.append('CAN');raise AssertionError('No CAN')
 return old(family,*a,**kw)
socket.socket=guard
def denied(*a,**kw):attempts.append('camera');raise AssertionError('No camera')
depth.DepthCamera=denied;snapshot_preview.DepthCamera=denied
p=PlanDisplay(folder/'plan.json');v=snapshot_preview.SnapshotPlaybackView(common.calibration(),folder,p)
checks={}
def camera_pose():
 c=v.viewport.camera;return (c.GetPosition(),c.GetFocalPoint(),c.GetViewUp())
def finger_print():
 h=hashlib.sha256()
 for arr in (mesh.vertices,mesh.faces,packet.raw_points,packet.raw_image):h.update(arr.tobytes())
 return h.hexdigest()
def drag(button,dx,dy):
 widget=v.viewport.widget;center=QtCore.QPoint(widget.width()//2,widget.height()//2)
 old=camera_pose();QtTest.QTest.mousePress(widget,button,QtCore.Qt.NoModifier,center)
 for i in range(1,21):
  QtTest.QTest.mouseMove(widget,center+QtCore.QPoint(round(dx*i/20),round(dy*i/20)),delay=3)
  v.app.processEvents()
 QtTest.QTest.mouseRelease(widget,button,QtCore.Qt.NoModifier,center+QtCore.QPoint(dx,dy));v.app.processEvents()
 assert camera_pose()!=old,(button,'camera did not change')
try:
 packet,mesh=planned_view.load(p,common.DEFAULT_CALIBRATION);before=finger_print();v.set_scene(packet,mesh)
 assert not v.viewport.offscreen
 v.window.activateWindow();v.viewport.widget.setFocus();v.app.processEvents()
 for name,button,dx,dy in [('rotate',QtCore.Qt.LeftButton,110,35),('pan',QtCore.Qt.MiddleButton,45,20),('zoom_drag',QtCore.Qt.RightButton,0,35)]:
  drag(button,dx,dy);checks[name]=True
 widget=v.viewport.widget;pos=QtCore.QPoint(widget.width()//2,widget.height()//2);old=camera_pose()
 event=QtGui.QWheelEvent(QtCore.QPointF(pos),QtCore.QPointF(widget.mapToGlobal(pos)),QtCore.QPoint(),QtCore.QPoint(0,120),QtCore.Qt.NoButton,QtCore.Qt.NoModifier,QtCore.Qt.NoScrollPhase,False)
 v.app.sendEvent(widget,event);v.app.processEvents();assert camera_pose()!=old;checks['wheel_zoom']=True
 for mode in (1,2,0):
  v.mode.setCurrentIndex(mode)
  assert v.cloud.get_visible()==(mode!=0) and v.surface.get_visible()==(mode!=1)
 v.color.setCurrentIndex(1);v.reset_view();v.focus_arm();checks['layers_colors_fit']=True
 v.window.activateWindow();widget.setFocus();v.app.processEvents()
 QtTest.QTest.keyClick(widget,QtCore.Qt.Key_W);v.app.processEvents()
 assert all(layer.actor.GetProperty().GetRepresentation()==2 for _,_,_,layer in v.robot.mesh_entries)
 checks['no_accidental_wireframe']=True
 QtTest.QTest.keyClick(widget,QtCore.Qt.Key_S);v.app.processEvents();assert v.poll()==ord('s')
 v.save_view();assert (folder/'preview_view.png').is_file();checks['native_gl_screenshot']=True
 QtTest.QTest.keyClick(widget,QtCore.Qt.Key_P);v.app.processEvents();assert v.clock.playing
 QtTest.QTest.keyClick(widget,QtCore.Qt.Key_P);v.app.processEvents();assert not v.clock.playing
 checks['native_keyboard_play_pause']=True
 goal=p.obj['outward'][-1]['time_s']
 for t,target in [(0,p.req['start']),(goal,p.req['goal']),(goal+1,p.req['goal']),(p.duration,p.req['start'])]:
  v.seek(t);assert np.array_equal(v.robot.last_positions,target)
 v.seek(goal);v.save_view();(folder/'preview_view.png').rename(D/'native_goal.png')
 checks['endpoint_hold_return']=True
 v.speed.setCurrentIndex(2);assert v.clock.rate==2
 v.speed.setCurrentIndex(0);assert v.clock.rate==.5
 v.speed.setCurrentIndex(1);assert v.clock.rate==1
 v.loop_box.setChecked(True);v.seek(p.duration-.02);v.play_button.click();time.sleep(.06);v.poll()
 assert v.clock.playing and v.clock.completed_cycles==1
 v.loop_box.setChecked(False);v.clock.completed_cycles=0;v.seek(0);v.play_button.click()
 started=time.monotonic()
 while v.clock.playing:
  assert time.monotonic()-started<p.duration+8
  v.poll();time.sleep(.003)
 assert v.clock.completed_cycles==1 and v.clock.position==p.duration
 assert np.array_equal(v.robot.last_positions,p.req['start'])
 assert v.phases_seen>={'outward','HOLD (2 seconds)','return'}
 checks['complete_realtime_cycle']=True
 uploads=[layer.geometry_uploads for _,_,_,layer in v.robot.mesh_entries]
 assert all(n==1 for n in uploads) and v.surface.geometry_uploads==1
 assert v.worker.submissions==0 and v.mesh_builds==1 and before==finger_print()
 checks['unchanged_geometry_and_frozen_scene']=True
 stopped=len(v.render_draw_seconds)
 for _ in range(15):v.poll();time.sleep(.003)
 assert len(v.render_draw_seconds)==stopped;checks['idle_no_repeated_renders']=True
 v.save_view();report=v.report();assert not report['renderer']['render_errors'] and not attempts
 QtTest.QTest.keyClick(widget,QtCore.Qt.Key_Q);v.app.processEvents();assert v.poll()==ord('q');checks['quit']=True
 (D/'native_playback_check.json').write_text(json.dumps(dict(passed=True,checks=checks,device_access_attempts=attempts,elapsed_cycle_s=time.monotonic()-started,report=report),indent=2))
 print('PASS native mouse/keyboard, all controls, full real-time cycle, unchanged scene, no devices',flush=True)
finally:v.close()
