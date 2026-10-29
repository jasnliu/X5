"""Add CPU-label assertions/screenshots to existing real-window regression."""
import sys,json,runpy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from drum_planner.experiment import app
from drum_planner.common import atomic_json
output=Path(sys.argv[sys.argv.index('--output')+1]);labels=[]
Base=app.make_view_class()
class Checked(Base):
 def install_plan(self,*a,**k):
  result=super().install_plan(*a,**k)
  if self.plan.playable:
   meta=self.plan.obj['experiment']['retiming']
   assert meta['requested_backend']=='cpu_sparse'
   assert 'CPU' in meta['label'] and 'GPU unavailable' not in meta['label']
   labels.append(meta['label']);atomic_json(output/'backend_ui_labels.json',labels)
   self.viewport.save_window(self.window,output/'timing_backend.png')
  return result
app.make_view_class=lambda:Checked
runpy.run_path(str(ROOT/'tests/experiment_destinations_ui_check.py'),run_name='__main__')
