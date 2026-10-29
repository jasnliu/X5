"""Actual Qt/VTK playback and selection against incident capture, NO devices."""
import json,sys,time,traceback
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import atomic_json,endpoint
from drum_planner.experiment import app
D=Path(__file__).resolve().parent;ROOT=D.parents[1];errors=[];rows=[]
def hook(t,v,b):
 errors.append(''.join(traceback.format_exception(t,v,b)));print(errors[-1],flush=True)
sys.excepthook=hook
folder=D/'ui';source=ROOT/'planner_runs/20261026T040710Z-experiment-f81b35/acquisition'
app.prepare(folder,source/'measured_start.json',source/'camera/scene.json',side='right')
view=app.make_view_class()(folder)
def pump(seconds=.1):
 end=time.monotonic()+seconds
 while time.monotonic()<end:view.poll();time.sleep(.01)
 assert not errors,errors
try:
 pump();assert view.job is None and not view.plan.playable
 for target in ('R1','R2'):
  dest=ROOT/f'destinations/right/destination{target}.json';index=view.destination_box.findData(str(dest))
  assert index>=0;view.destination_box.setCurrentIndex(index)
  end=time.monotonic()+120
  while (view.job or view.pending_methods) and time.monotonic()<end:pump()
  assert view.job is None and view.plan.playable,view.status.text()
  assert view.plan.obj['status']=='planned';duration=view.plan.obj['duration_outward_s']
  assert view.plan.req['goal']==endpoint(dest,side='right').tolist()
  # Full native animation at 1x; every displayed frame is evaluated from the law.
  view.seek(0.);view.play_button.click();pump(duration+.2);view.play_button.click()
  for i,fraction in enumerate((0.,.25,.5,.75,1.)):
   view.seek(duration*fraction);pump(.1)
   view.viewport.fit(np.array([[-.25,-.65,-.3],[.85,.25,.7]]),elev=24,azim=-55)
   view.draw_canvas();pump(.05)
   view.viewport.save_window(view.window,folder/f'{target}_{i}.png')
  np.testing.assert_allclose(view.plan.positions(duration),endpoint(dest,side='right'),atol=1e-9)
  assert view.mesh_builds==1 and not view.viewport.errors
  rows.append(dict(target=target,passed=True,plan=str(view.plan.path),full_animation_seconds=duration,
      screenshots=5,native_qt_vtk=True,mesh_builds=view.mesh_builds,hardware_access=False,motor_commands=0))
  atomic_json(folder/'verification.json',rows)
 # Cancel a fresh solve and retry target selection; no stale result installed.
 view.generate_button.click();pump(.1);view.cancel_button.click()
 end=time.monotonic()+15
 while view.job and time.monotonic()<end:pump()
 assert view.job is None
 view.generate_button.click();pump(.1);job=view.job;assert job is not None
 view.close();assert job.process.poll() is not None
 atomic_json(folder/'controls.json',dict(passed=True,cancel_reaps=True,close_reaps=True,errors=errors))
 print('NATIVE UI PASS',flush=True)
finally:view.close()
