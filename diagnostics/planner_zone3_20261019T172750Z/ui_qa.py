"""Actual camera and native UI; no CAN, no motors; independent legacy zone audit."""
import json,time,shutil
from pathlib import Path
import numpy as np
from scipy.interpolate import PPoly
from drum_planner.experiment.app import prepare,make_view_class
from drum_planner.experiment.config import METHODS,no_can
from drum_planner.common import ROOT,JOINTS,atomic_json
from safe_zone.geometry import Model,Zone,RIGHT_TCP,MEMBERSHIP_BUFFER_M
from camera_calibration.kinematics import HandKinematics
no_can();D=Path(Path('/tmp/x5_zone_task').read_text().strip());session=D/'actual_camera'
prepare(session);v=make_view_class()(session,autoplay=True);checks={};started=time.monotonic()
def pump():
 key=v.poll()
 if key==ord('s'):v.save_view()
 time.sleep(.01)
try:
 assert v.context['right_zone']['tcp']==RIGHT_TCP and len(v.pending_methods)==8
 while v.job or v.pending_methods:
  pump();assert time.monotonic()-started<1100
 assert len(v.cache)==9 and v.generation_count==9,v.method_states
 assert not v.viewport.errors
 batch=v.report();atomic_json(D/'automatic_all_report.json',batch)
 m=Model(ROOT/'model/openarmx.urdf');z=Zone.load(ROOT/'right_zones/zone3.json',m.digest,RIGHT_TCP)
 frames=m.transforms({});local=np.linalg.inv(frames['openarmx_right_hand'])@frames[RIGHT_TCP]
 fk=HandKinematics(ROOT/'model/openarmx.urdf');planes=z.hull.equations
 report={};planned=[]
 for method,(_,path) in v.cache.items():
  obj=json.loads(path.read_text());req=json.loads((path.parent/'backend_request.json').read_text())
  assert obj['right_zone_constraint']==v.context['right_zone']
  assert obj['right_zone_constraint']['constraint_sha256']==req['right_zone']['constraint_sha256']
  if obj['status']!='planned':
   assert obj['status']=='no_feasible_route';report[method]=dict(status=obj['status'],details=obj.get('details',{}));continue
  planned.append(method);t=obj['timing'];curves=[PPoly(np.asarray(c),np.asarray(x)) for c,x in zip(t['coefficients'],t['polynomial_breaks_unscaled_s'])]
  grid=np.r_[np.arange(0,t['duration_s'],.002),t['duration_s']]
  q=np.column_stack([p(grid/t['time_scale']) for p in curves]);worst=-float('inf')
  for offset in range(0,len(q),500):
   part=q[offset:offset+500];xyz=(fk.fk(part)@local)[:,:3,3]
   d=(xyz@planes[:,:3].T+planes[:,3])/np.linalg.norm(planes[:,:3],axis=1)
   worst=max(worst,float(d.max()))
   assert np.all(d<=MEMBERSHIP_BUFFER_M+1e-9),(method,worst)
  for j in np.linspace(0,len(q)-1,21,dtype=int):
   p=m.transforms(dict(zip(JOINTS,q[j])))[RIGHT_TCP][:3,3]
   assert z.contains(p,MEMBERSHIP_BUFFER_M)
   assert np.allclose(p,(fk.fk(q[j:j+1])@local)[0,:3,3],atol=2e-14,rtol=0)
  assert np.allclose(q[0],req['start'],atol=1e-10,rtol=0) and np.allclose(q[-1],req['goal'],atol=1e-10,rtol=0)
  assert obj['hold_at_goal_s']==2 and obj['reverse_checked_on_same_static_scene']
  assert np.array_equal(obj['cycle'][-1]['positions_rad'],req['start'])
  report[method]=dict(status='planned',passed=True,dense_samples=len(q),max_signed_plane_distance_m=worst,
       allowed_m=MEMBERSHIP_BUFFER_M+1e-9,original_model_and_tcp=True,outward_s=t['duration_s'])
 assert planned,'No valid trajectory for current scene'
 atomic_json(D/'independent_zone_audit.json',report)
 # Cache selection and full real-time playback of one accepted constrained plan.
 for method in METHODS:
  v.method_box.setCurrentIndex(list(METHODS).index(method));pump()
  assert v.displayed_method==method and v.job is None and v.generation_count==9
  assert v.play_button.isEnabled()==(method in planned)
 method='clearance' if 'clearance' in planned else planned[0]
 v.autoplay_result=False;v.method_box.setCurrentIndex(list(METHODS).index(method));pump()
 v.clock.pause();v.seek(0);v.speed.setCurrentIndex(2);v.toggle_play()
 end=time.monotonic()+v.plan.duration/2+5
 while v.clock.playing and time.monotonic()<end:pump()
 assert v.clock.completed_cycles>=1 and np.allclose(v.robot.last_positions,v.plan.req['start'],atol=1e-10)
 # A cached historical/unconstrained result may NOT be installed into this new session.
 bad=D/'unconstrained_preview_fixture';bad.mkdir();obj=dict(v.plan.obj);obj.pop('right_zone_constraint')
 atomic_json(bad/'plan.json',obj);shutil.copy2(v.plan.path.parent/'backend_request.json',bad/'backend_request.json')
 previous=v.plan
 try:v.install_plan(bad/'plan.json',method)
 except ValueError as e:assert 'zone3' in str(e)
 else:raise AssertionError('Unconstrained preview accepted into constrained cache')
 assert v.plan is previous
 assert v.viewport.surface.geometry_uploads==1 and not v.viewport.errors
 v.focus_arm();v.viewport.save_window(v.window,D/'zone3_ui.png')
 checks=dict(all_methods_attempted_and_cached=True,planned_methods=planned,independent_legacy_zone_check=True,
             full_playback=True,old_unconstrained_cache_rejected=True,frozen_mesh_uploads=1,render_errors=[],can_access=False,motor_commands=0)
 atomic_json(D/'ui_qa.json',dict(passed=True,checks=checks,report=v.report()))
 print('ZONE3 UI QA PASSED',json.dumps(checks),flush=True)
finally:v.close()
