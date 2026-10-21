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
 lines=subprocess.check_output(['wmctrl','-lpG']).decode().splitlines()
 own=[line for line in lines if line.split()[2]==str(os.getpid())]
 assert len(own)==1,own
 print('Exactly one native window:',own,flush=True)
 for w,h in [(1440,900),(1120,820),(1280,800)]:
  v.window.resize(w,h);v.app.processEvents()
  buttons=[b for b in v.window.findChildren(v.QtWidgets.QPushButton) if b.isVisible()]
  rects=[v.QtCore.QRect(b.mapTo(v.window,v.QtCore.QPoint()),b.size()) for b in buttons]
  for i,r in enumerate(rects):
   assert v.window.rect().contains(r),buttons[i].text()
   assert not any(r.intersects(x) for x in rects[i+1:]),buttons[i].text()
  v.viewport.save_window(v.window,d/f'layout_{w}x{h}.png')
 v.window.resize(1440,900);v.app.processEvents()
 v.viewport.save_window(v.window,d/'original.png')
 v.app.primaryScreen().grabWindow(0).save(str(d/'desktop.png'))
 atomic_json(d/'geometry.json',dict(pid=os.getpid(),windows=[dict(title=x.windowTitle(),visible=x.isVisible(),rect=x.geometry().getRect()) for x in v.app.topLevelWidgets()],widgets=[dict(type=x.metaObject().className(),text=x.text() if hasattr(x,'text') else '',rect=(x.mapTo(v.window,v.QtCore.QPoint(0,0)).x(),x.mapTo(v.window,v.QtCore.QPoint(0,0)).y(),x.width(),x.height())) for x in v.window.findChildren(v.QtWidgets.QWidget) if x.isVisible()]))
 time.sleep(1)
finally:v.close()
