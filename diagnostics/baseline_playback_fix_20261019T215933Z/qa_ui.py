import json,sys,time,os,subprocess
from pathlib import Path
from drum_planner.baseline.app import View
from drum_planner.common import DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can
no_can();d=Path(sys.argv[1]);d.mkdir(parents=True,exist_ok=True)
plan=Path('planner_runs/20261019T215312Z-baseline-ba479b/plan/result/plan.json')
v=View(plan,d,DEFAULT_CALIBRATION,simulated=False)
try:
 for _ in range(30):v.poll();time.sleep(.02)
 print('TOP LEVELS',[(x.windowTitle(),x.isVisible(),x.geometry().getRect()) for x in v.app.topLevelWidgets()],flush=True)
 print(subprocess.check_output(['wmctrl','-lpG']).decode(),flush=True)
 v.viewport.save_window(v.window,d/'original.png')
 v.app.primaryScreen().grabWindow(0).save(str(d/'desktop.png'))
 atomic_json(d/'geometry.json',dict(pid=os.getpid(),windows=[dict(title=x.windowTitle(),visible=x.isVisible(),rect=x.geometry().getRect()) for x in v.app.topLevelWidgets()],widgets=[dict(type=x.metaObject().className(),text=x.text() if hasattr(x,'text') else '',rect=(x.mapTo(v.window,v.QtCore.QPoint(0,0)).x(),x.mapTo(v.window,v.QtCore.QPoint(0,0)).y(),x.width(),x.height())) for x in v.window.findChildren(v.QtWidgets.QWidget) if x.isVisible()]))
 time.sleep(1)
finally:v.close()
