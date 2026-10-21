import os,sys,json,time,socket
from pathlib import Path
import pytest
ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tests'))
from drum_planner import current_plan,planned_view
from test_current_plan import rig
folder=Path('/tmp/x5_current_planning_diagnostics').read_text();folder=Path(folder)
original=socket.socket;attempts=[]
def guarded(family=socket.AF_INET,*a,**kw):
 if family==socket.AF_CAN:attempts.append('CAN');raise AssertionError('No real CAN allowed')
 return original(family,*a,**kw)
socket.socket=guarded
# Fake encoders/depth only. Actual MoveIt sees these fabricated inputs and blocks.
with pytest.MonkeyPatch.context() as mp:
 r=rig.__wrapped__(folder,mp)
 path=current_plan.run(folder/'OFFLINE_SYNTHETIC_INPUTS_FINAL',headless=True)
obj=json.loads(path.read_text());assert obj['status']=='blocked_start'
Original=planned_view.SnapshotPlaybackView
class CheckedView(Original):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw);self.started=time.monotonic();self.saved=False
 def poll(self,*a,**kw):
  result=super().poll(*a,**kw)
  self.banner.setText('OFFLINE TEST ONLY — SYNTHETIC DEPTH + MOCK ENCODERS — NO HARDWARE ACCESSED')
  if self.packet is not None and not self.saved and time.monotonic()-self.started>1:
   assert not self.plan.playable
   self.mode.setCurrentIndex(2);self.reset_view();self.refresh();self.app.processEvents()
   self.save_view();self.saved=True
  return result
planned_view.SnapshotPlaybackView=CheckedView
report=planned_view.show(path,seconds=3)
assert not attempts and report['same_planning_scene'] and report['scene_immutable']
(folder/'offline_preview_check.json').write_text(json.dumps(dict(passed=True,actual_hardware=False,
 duration_requested_s=3,can_attempts=attempts,preview=report),indent=2))
print('PASS: brief offline matched-scene preview; no devices; blocked route remains unplayable')
