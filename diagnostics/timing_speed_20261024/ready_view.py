import sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from drum_planner.experiment import app,config
from drum_planner.common import atomic_json
config.no_can()
folder=Path(sys.argv[1]);v=app.make_view_class()(folder,method='workspace_curve')
try:
 assert not v.plan.playable and v.job is None
 v.destination_box.setCurrentIndex(1)
 until=time.monotonic()+90
 while (v.job or v.pending_methods) and time.monotonic()<until:v.poll();time.sleep(.02)
 assert v.job is None and not v.pending_methods and v.plan.playable
 for _ in range(10):v.poll();time.sleep(.02)
 assert 'CPU sparse timing' in v.status.text(),v.status.text()
 assert 'intentional CPU backend' in v.status.text(),v.status.text()
 v.viewport.save_window(v.window,folder/'timing_ready.png')
 atomic_json(folder/'ready_label.json',dict(status=v.status.text(),plan_label=v.plan_label.text(),plan=str(v.plan.path)))
 print('READY VIEW PASS',v.side,v.status.text(),flush=True)
finally:v.close()
