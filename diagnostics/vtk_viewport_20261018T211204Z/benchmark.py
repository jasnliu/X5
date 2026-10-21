import sys,time,socket,json,os
from pathlib import Path
import numpy as np
ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import depth,snapshot_preview,planned_view,common
from drum_planner.trajectory_view import PlanDisplay
old=socket.socket

def guard(family=socket.AF_INET,*a,**kw):
 if family==socket.AF_CAN:raise AssertionError('No CAN during rendering benchmarks')
 return old(family,*a,**kw)
socket.socket=guard

def denied(*a,**kw):raise AssertionError('No camera during rendering benchmarks')
depth.DepthCamera=denied;snapshot_preview.DepthCamera=denied
D=Path(__file__).parent;p=PlanDisplay(ROOT/'diagnostics/observed_obstacle_policy_20261018T201044Z/live_current_pose/plan/plan.json')
v=snapshot_preview.SnapshotPlaybackView(common.calibration(),D,p)
try:
 packet,mesh=planned_view.load(p,common.DEFAULT_CALIBRATION);v.set_scene(packet,mesh);v.seek(p.obj['outward'][-1]['time_s']);v.app.processEvents()
 gpu=hasattr(v,'viewport');label='after' if gpu else 'before'
 if gpu:v.viewport.set_angles(25,-55)
 else:v.ax.view_init(elev=25,azim=-55)
 v.draw_canvas();v.save_view();(D/'preview_view.png').rename(D/(label+'_view.png'))
 elapsed=[]
 for i in range(36):
  start=time.perf_counter()
  if gpu:v.viewport.set_angles(25,-55+i*5)
  else:v.ax.view_init(elev=25,azim=-55+i*5)
  v.draw_canvas();v.app.processEvents();elapsed.append(time.perf_counter()-start)
 result=dict(renderer=label,rotation_frames=len(elapsed),median_frame_ms=1000*float(np.median(elapsed)),p95_frame_ms=1000*float(np.percentile(elapsed,95)),mean_fps=len(elapsed)/sum(elapsed),mesh_faces=len(mesh.faces),can_access=False,camera_access=False)
 if gpu:result['backend']=v.viewport.report()
 (D/(label+'_benchmark.json')).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:v.close()
