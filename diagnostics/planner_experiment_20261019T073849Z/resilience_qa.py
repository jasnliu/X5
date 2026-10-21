import json,time,subprocess
from pathlib import Path
from drum_planner.experiment.app import make_view_class
from drum_planner.experiment.runtime import Job
from drum_planner.experiment.config import no_can
from drum_planner.common import atomic_json
no_can()
D=Path(Path('/tmp/x5_experiment_task').read_text().strip())
session=D/'actual_camera_all'; row=json.loads((D/'ui_full.json').read_text());v=make_view_class()(session)
closed=False;checks={}
def pump(seconds=.15):
 end=time.monotonic()+seconds
 while time.monotonic()<end:v.poll();time.sleep(.01)
def wait_job():
 end=time.monotonic()+12
 while v.job:
  pump(.05)
  assert time.monotonic()<end
try:
 v.install_plan(row['plan_file'],'clearance');assert v.plan.playable
 original=v.plan.path;scene=v.context['scene'];uploads=v.viewport.surface.geometry_uploads
 v.recapture();cancelled=v.job;assert not v.play_button.isEnabled();v.cancel_job();wait_job()
 assert cancelled.cancelled and cancelled.process.poll() is not None
 assert not (v.job_target/'camera/scene.json').exists(), 'Immediate cancellation ran the entire capture'
 assert v.plan.path==original and v.context['scene']==scene
 assert v.play_button.isEnabled() and v.timeline.isEnabled()
 assert v.viewport.surface.geometry_uploads==uploads
 v.toggle_play();pump();assert v.clock.playing;v.toggle_play()
 checks['cancel_recapture_preserves_scene_and_playback']=True
 # Controlled process failure, not a sensor fault: UI remains responsive and
 # retains only the already validated result, never a partial new trajectory.
 v.job=Job(['/usr/bin/python3','-c','import sys; sys.exit(7)'],D/'intentional_worker_failure',timeout=10)
 v.job_kind='plan';v.job_target=D/'intentional_worker_failure';v.locked(True);wait_job()
 assert 'Worker stopped' in v.status.text() and v.plan.path==original
 assert v.generate_button.isEnabled() and v.play_button.isEnabled()
 checks['controlled_worker_failure_handled']=True
 # Closing during actual planning must stop the full subprocess tree.
 v.budget_spin.setValue(5);assert v.planning_seconds==5
 v.generate();pump(.7);unit=v.job.unit;v.close();closed=True
 status=subprocess.run(['systemctl','--user','show',unit+'.service','-p','ActiveState','-p','MainPID'],capture_output=True,text=True).stdout
 assert 'ActiveState=active' not in status and 'MainPID=0' in status,status
 checks['close_during_planning_reaps_service']=True
 assert not v.viewport.errors
 atomic_json(D/'ui_resilience.json',dict(passed=True,checks=checks,unit_status=status))
 print('RESILIENCE QA PASSED',json.dumps(checks),flush=True)
finally:
 if not closed:v.close()
