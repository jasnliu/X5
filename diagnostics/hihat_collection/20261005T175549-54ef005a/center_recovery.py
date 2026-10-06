import sys,time,json,math,importlib.util
from pathlib import Path
import numpy as np
sys.path.insert(0,'/home/jason/Proyectos3/X5')
from centering.motors import Motors
from camera_playback.playback_transport import PlaybackMotors
from camera_playback.recording_only import refresh_feedback
s=importlib.util.spec_from_file_location('ntfy','/home/jason/.local/bin/codex-ntfy.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
assert m.post_ntfy('Robot recovery now: maintain powered hold, verify customized right center, then relax. Keep clear.','Robot center recovery','warning,robot')
p=Path('/home/jason/Proyectos3/X5/diagnostics/hihat_collection/20261005T175549-54ef005a')
goal=np.array([0.]*6+[1.4]);b=PlaybackMotors.adopt(Motors('right',True),goal,directory=p)
b.require_center_before_relax=True
try:
 refresh_feedback(b,.15)
 q=np.array([b.states['right',i][0] for i in range(1,8)])
 print('RECOVERY INITIAL',q.tolist(),flush=True)
 assert np.max(np.abs(q-goal))<math.radians(2),'Not a near-center recovery'
 assert all(b.states['right',i][1]==2 for i in range(1,9))
 b.active=True;b.request_right_joint7_mode_readback(clear=True)
 deadline=time.monotonic()+1
 while b.right_joint7_mode_readback() is None:
  b.poll();time.sleep(.002);assert time.monotonic()<deadline
 assert b.right_joint7_mode_readback()==0
 b.begin_mit_center_return(0.)
 started=time.monotonic();lastprint=0
 while time.monotonic()-started<20:
  b.poll();b.set_positions(goal);now=time.monotonic()
  if now-lastprint>.5:
   print('CENTER ERROR deg',math.degrees(b.states['right',7][0]-1.4),flush=True);lastprint=now
  try:e=b.center_evidence()
  except RuntimeError:time.sleep(.003);continue
  b.relax();print('CENTER VERIFIED AND RELAXED',json.dumps(e),flush=True)
  deadline=time.monotonic()+2
  while time.monotonic()<deadline:
   b.poll()
   if b.fresh() and all(v[1]==0 for v in b.states.values()):
    (p/'recovery_verified.json').write_text(json.dumps(dict(center=e,all_16_disabled=True),indent=2));print('ALL 16 DISABLED',flush=True);break
   time.sleep(.003)
  else:raise RuntimeError('disable not confirmed')
  break
 else:raise RuntimeError('Center not settled; remains powered')
finally:b.close()
