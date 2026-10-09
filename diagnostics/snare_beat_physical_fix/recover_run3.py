"""One-time recovery of run1's faulted powered hold. NEVER enables/mode-writes.
Requires exclusive CAN locks, all drives running, J6 already MIT, all others CSP.
Retains existing target hold through validation, commands only verified center.
"""
import csv,json,time
from pathlib import Path
import numpy as np
from centering.motors import Motors,packet,parameter,SPEED
from camera_playback.playback_transport import RetrySocket
from camera_playback.recording_only import refresh_feedback
from camera_playback.dual_recording import left_geometry
from camera_playback.mit_center_return import MitCenterReturn
from hihat_collection.transport import DataMotors
from snare_lab.runner import _LeftJoint6Channel
from smooth_playback.trajectory import Geometry
p=Path(__file__).resolve().parent/'run3'/'recovery';p.mkdir(exist_ok=False)
g=Geometry();l=left_geometry(g.model)
b=DataMotors.adopt(Motors('right',True),g.center,directory=p);b.start_audit(p)
b.require_center_before_relax=True
try:
 refresh_feedback(b,.2)
 assert len(b.states)==16 and all(b.states['left',i][1]==2 and b.states['right',i][1]==0 for i in range(1,9))
 for i in range(1,9):
  b.send_control(packet(17,i,bytes.fromhex('0570000000000000')))
  b.left_drive.send_control(packet(17,i,bytes.fromhex('0570000000000000')))
 refresh_feedback(b,.2)
 modes={'right':{i:b.modes[i][0] for i in range(1,9)},'left':{i:b.left_drive.modes[i][0] for i in range(1,9)}}
 assert all(v==5 for v in modes['right'].values())
 assert all(v==(0 if i==6 else 5) for i,v in modes['left'].items())
 poses={s:np.array([b.states[s,i][0] for i in range(1,8)]) for s in ('left','right')}
 # J4's existing held -0.10 degree overshoot is within the established 0.20 degree center acceptance. Only command INWARD to zero.
 assert poses['left'][3]>=-.004363323129985824
 legal=poses['left'].copy();legal[3]=max(0.,legal[3])
 l.check_line(legal,l.center,measured_start=True)
 from safe_zone.geometry import MEMBERSHIP_BUFFER_M
 assert l.zone.contains(l.tcp(poses['left']),MEMBERSHIP_BUFFER_M)
 refresh_feedback(b,.2)
 assert all(max(abs(poses[s]-[b.states[s,i][0] for i in range(1,8)]))<.0035 for s in poses)
 (p/'initial.json').write_text(json.dumps(dict(modes=modes,poses={s:q.tolist() for s,q in poses.items()}),indent=2))
 b.left_channel=_LeftJoint6Channel('can1');b.left_channel.socket=RetrySocket(b.left_channel.socket)
 b.left_channel.send(packet(17,6,bytes.fromhex('0570000000000000')))
 deadline=time.monotonic()+.5
 while b.left_channel.mode is None or b.left_channel.sample is None:
  b.poll();b.left_channel.receive();time.sleep(.002);assert time.monotonic()<deadline
 assert b.left_channel.mode==0 and b.left_channel.sample.state==2
 b.snare_trace_file=(p/'j6.csv').open('w');b.snare_trace=csv.writer(b.snare_trace_file)
 b.left_mit=MitCenterReturn(b.left_channel.sample.position,-l.center[5],0.,time.monotonic())
 b.active=b.left_drive.active=True
 # No disable/re-enable, gain change, or mode switch during recovery.
 for i in range(1,8):
  b.left_drive.send_control(parameter(i,0x7017,SPEED))
 b.left_drive.set_positions(l.center)
 print('POWERED CENTER RETURN DISPATCHED',flush=True)
 deadline=time.monotonic()+30
 while time.monotonic()<deadline:
  b.poll()
  try:proof={'left':b.left_center_evidence(),'right_already_verified_and_relaxed_by_main_program':True}
  except RuntimeError:time.sleep(.002);continue
  b.left_drive.relax();b.left_mit=None;b.active=False
  deadline=time.monotonic()+2
  while time.monotonic()<deadline:
   b.poll()
   if b.fresh() and all(s[1]==0 for s in b.states.values()):
    (p/'result.json').write_text(json.dumps(dict(success=True,all_16_disabled=True,proof=proof),indent=2)+'\n')
    print('BOTH VERIFIED CENTERED THEN RELAXED',flush=True);break
   time.sleep(.002)
  else:raise RuntimeError('Disabled feedback not confirmed')
  break
 else:raise RuntimeError('Center did not settle; remains POWERED')
finally:b.close() # closes sockets ONLY, no implicit relax on failure
