from pathlib import Path
import json,sys,shutil,socket,time
ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import planned_view,depth
from drum_planner.trajectory_view import PlanDisplay
out=Path(Path('/tmp/x5_start_limit_diagnostics').read_text());source=ROOT/'planner_runs/20261018T193113Z-current-plan-78618c/plan'
legacy=out/'original_error_preview';shutil.copytree(source,legacy)
original=socket.socket;attempts=[]
def guarded(family=socket.AF_INET,*a,**kw):
 if family==socket.AF_CAN:attempts.append('CAN');raise AssertionError('No CAN access in saved-data preview')
 return original(family,*a,**kw)
socket.socket=guarded
def denied(*a,**kw):raise AssertionError('No camera in saved-data preview')
depth.DepthCamera=denied
Original=planned_view.SnapshotPlaybackView
class Checked(Original):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw);self.began=time.monotonic();self.saved=False
 def poll(self,*a,**kw):
  key=super().poll(*a,**kw)
  if self.packet is not None and not self.saved and time.monotonic()-self.began>.4:
   assert not self.play_button.isEnabled() and not self.plan.playable
   self.toggle_play();assert not self.clock.playing
   self.app.processEvents();self.save_view();self.saved=True
  return key
planned_view.SnapshotPlaybackView=Checked
reports=[]
for folder in (legacy,out/'saved_run_replanned'):
 model=PlanDisplay(folder/'plan.json');assert not model.playable
 report=planned_view.show(folder/'plan.json',seconds=2)
 assert report['same_planning_scene'] and report['scene_immutable']
 reports.append(dict(path=str(folder),legacy_error=model.legacy_error_report,report=report))
assert not attempts
(out/'preview_regression.json').write_text(json.dumps(dict(passed=True,actual_hardware_access=False,can_attempts=attempts,reports=reports),indent=2))
print('PASS: original schema-less error and corrected replay both render; Play stays disabled; no devices')
