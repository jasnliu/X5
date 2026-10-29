"""Real Qt/VTK --fast controls, using saved scene; no camera/CAN/motor access."""
import json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path.cwd()))
from drum_planner.common import atomic_json,DEFAULT_CALIBRATION
from drum_planner.experiment.app import prepare
from drum_planner.fast_mode.app import make_view_class
ROOT=Path.cwd();HERE=ROOT/'diagnostics/fast_posture_20261024';rows=[]
state=ROOT/'diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/measured_start.json'
scene=ROOT/'diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/camera/scene.json'

def pump(v,seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:v.poll();time.sleep(.02)

def wait(v,limit=240):
 end=time.monotonic()+limit
 while v.job is not None and time.monotonic()<end:v.poll();time.sleep(.03)
 assert v.job is None,'UI worker timeout'

for side in ['right','left']:
 folder=HERE/('ui_'+side);prepare(folder,state,scene,DEFAULT_CALIBRATION,side=side,fresh=False)
 view=make_view_class()(folder)
 try:
  pump(view,.5);assert view.job is None and view.generation==0 and not view.plan.playable
  first=ROOT/f'destinations/{side}'/('destinationR1.json' if side=='right' else 'destinationL1.json')
  view.boxes[0].setCurrentIndex(view.boxes[0].findData(str(first)))
  if side=='right':
   second=ROOT/'destinations/right/destinationR2.json'
   view.boxes[1].setCurrentIndex(view.boxes[1].findData(str(second)))
   view.calculate_button.click();assert view.job is not None
   pump(view,.6);view.boxes[1].setCurrentIndex(view.boxes[1].findData(str(first)));wait(view,15)
   assert not view.plan.playable
   view.calculate_button.click();assert view.job is None and 'differ' in view.status.text()
   view.boxes[1].setCurrentIndex(view.boxes[1].findData(str(second)))
  view.calculate_button.click();wait(view)
  assert view.plan.playable,view.status.text()
  assert view.plan.side==side and view.plan.obj['motion_authorized'] is False
  view.play_button.click();pump(view,.5);assert view.clock.position>0
  view.play_button.click();assert not view.clock.playing
  view.speed.setCurrentIndex(2);assert view.clock.rate==2
  view.timeline.setValue(int(view.plan.duration*400));assert not view.clock.playing
  assert view.mesh_builds==1
  view.save_view();atomic_json(folder/'ready_report.json',view.report())
  ready_plan=str(view.plan.path)
  view.calculate_button.click();pump(view,.4);job=view.job;assert job is not None
  view.close();assert job.process.poll() is not None
  row=dict(side=side,passed=True,native_qt_vtk=True,no_startup_calculation=True,
    selection_cancellation_tested=side=='right',invalid_selection_retry_tested=side=='right',
    play_pause_scrub_speed_tested=True,close_cancels_worker=True,one_frozen_mesh=True,
    plan=ready_plan,motor_commands=0,camera_access=False,can_access=False)
  rows.append(row);atomic_json(HERE/'ui_results.json',rows)
  print('NATIVE UI PASS',side,flush=True)
 finally:view.close()
