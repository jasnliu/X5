import json,time
from pathlib import Path
import numpy as np
from drum_planner.depth import DepthCamera,snapshot
from drum_planner.common import calibration,DEFAULT_CALIBRATION
D=Path('/tmp/x5_repeat_stick_diag').read_text().strip(); out=Path(D)/'depth_series';out.mkdir();c=DepthCamera(out,calibration());last=0;start=time.monotonic();saved=0
try:
 while saved<8:
  if c.error: raise RuntimeError(c.error)
  if time.monotonic()-start>30: raise RuntimeError('timeout')
  if len(c.frames)==5 and c.latest[0]['sequence']>=last+10:
   frames=list(c.frames);last=frames[-1][0]['sequence'];folder=out/f'window_{last:03}';folder.mkdir();snapshot(folder,c.meta,frames,DEFAULT_CALIBRATION);saved+=1;print(folder,flush=True)
  time.sleep(.01)
finally:c.close()
