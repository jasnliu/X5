"""One-time powered hold handoff. No enable/mode writes; center-only disable.
Restores only the exact saved volatile gains from the failed recording worker.
"""
import csv,json,math,os,time
from pathlib import Path
import numpy as np
from centering.motors import Motors,packet,parameter,SPEED
from camera_playback.playback_transport import RetrySocket
from camera_playback.recording_only import refresh_feedback
from camera_playback.dual_recording import left_geometry
from camera_playback.mit_center_return import MitCenterReturn
from camera_playback.smooth_recording_worker import LeftRecordingMotors
from hihat_collection.transport import DataMotors
from snare_lab.runner import _LeftJoint6Channel
from smooth_playback.trajectory import Geometry,transition
p=Path(__file__).resolve().parent/'run1'/'recovery';p.mkdir(exist_ok=False)
g=Geometry();l=left_geometry(g.model)
# Exclusive locks must be released by both stopped old writers before ownership.
deadline=time.monotonic()+30
while True:
 try: native=Motors('right',True);break
 except BlockingIOError:
  if time.monotonic()>deadline:raise
  time.sleep(.01)
b=DataMotors.adopt(native,g.center,directory=p);b.start_audit(p)
b.require_center_before_relax=True
gb=None
try:
 refresh_feedback(b,.2)
 assert len(b.states)==16 and all(s[1]==2 for s in b.states.values())
 for i in range(1,9):
  b.send_control(packet(17,i,bytes.fromhex('0570000000000000')))
  b.left_drive.send_control(packet(17,i,bytes.fromhex('0570000000000000')))
 refresh_feedback(b,.2)
 modes={'right':{i:b.modes[i][0] for i in range(1,9)},'left':{i:b.left_drive.modes[i][0] for i in range(1,9)}}
 assert all(v==5 for v in modes['right'].values())
 assert all(v==(0 if i==6 else 5) for i,v in modes['left'].items())
 b.left_channel=_LeftJoint6Channel('can1');b.left_channel.socket=RetrySocket(b.left_channel.socket)
 deadline=time.monotonic()+.5
 while b.left_channel.sample is None:
  b.left_channel.send(packet(17,6,bytes.fromhex('0570000000000000')))
  b.poll();b.left_channel.receive();time.sleep(.002);assert time.monotonic()<deadline
 assert b.left_channel.sample.state==2
 b.snare_trace_file=(p/'j6.csv').open('w');b.snare_trace=csv.writer(b.snare_trace_file)
 class NativeVelocityPath(MitCenterReturn):
  def update(self,actual,now):
   return super().update(actual,now,measured_velocity=b.left_channel.sample.velocity)
 s=b.left_channel.sample
 b.left_mit=NativeVelocityPath(s.position,s.position,s.torque,time.monotonic())
 b.active=b.left_drive.active=True
 q=np.array([b.states['left',i][0] for i in range(1,8)])
 b.left_drive.set_positions(q)
 print('EXCLUSIVE POWERED HOLD ACQUIRED; NO ENABLE/DISABLE/MODE CHANGE',flush=True)
 deadline=time.monotonic()+3
 while time.monotonic()<deadline:b.poll();time.sleep(.002)
 # Existing bounded readback-verified gain ramp, with hold supervision.
 class Handle:
  def __init__(self,fd):self.fd=fd
  def detach(self):return os.dup(self.fd)
 gain_dir=p/'gains';gain_dir.mkdir()
 gb=LeftRecordingMotors(gain_dir,[Handle(fd) for fd in b.locks])
 saved=json.loads(Path('playback_results/start_beat/20261009T211339-0f7d4fc7/result.json').read_text())['original_gains']
 gb.gains_original={int(k):v for k,v in saved.items()};gb.gains_restored=False
 def hold():
  b.poll()
  assert all(st[1]==2 for st in b.states.values())
  actual=np.array([b.states['left',i][0] for i in range(1,8)])
  if max(abs(actual-q))>math.radians(.25):raise RuntimeError('Recovery gain hold exceeds unchanged .25 degree envelope')
  b.left_drive.set_positions(q)
 gb.restore_position_gains(ramp_seconds=.5,tick=hold)
 assert gb.gains_restored
 gb.close();gb=None
 print('ORIGINAL LEFT POSITION GAINS RESTORED AND READ BACK',flush=True)
 poses={s:np.array([b.states[s,i][0] for i in range(1,8)]) for s in ('left','right')}
 for side,geom in [('left',l),('right',g)]:geom.check_line(poses[side],geom.center,measured_start=True)
 refresh_feedback(b,.2)
 assert all(max(abs(poses[s]-[b.states[s,i][0] for i in range(1,8)]))<math.radians(.2) for s in poses)
 (p/'initial.json').write_text(json.dumps(dict(modes=modes,poses={s:q.tolist() for s,q in poses.items()}),indent=2))
 curves={s:transition(poses[s],geom.center) for s,geom in [('left',l),('right',g)]}
 for i in range(1,8):
  b.send_control(parameter(i,0x7017,SPEED));b.left_drive.send_control(parameter(i,0x7017,SPEED))
 print('VALIDATED SMOOTH CENTER RETURN STARTED',flush=True)
 elapsed=0.;last=time.monotonic();deadline=last+30;next_target=last
 while time.monotonic()<deadline:
  b.poll();now=time.monotonic()
  if now>=next_target:
   elapsed+=min(.02,now-last);last=now;next_target=now+.01
   b.set_positions(curves['right'][1](min(elapsed,curves['right'][0])))
   b.left_drive.set_positions(curves['left'][1](min(elapsed,curves['left'][0])))
  try:proof={'left':b.left_center_evidence(),'right':b.center_evidence()}
  except RuntimeError:time.sleep(.002);continue
  b.relax()
  deadline=time.monotonic()+2
  while time.monotonic()<deadline:
   b.poll()
   if b.fresh() and all(s[1]==0 for s in b.states.values()):
    (p/'result.json').write_text(json.dumps(dict(success=True,all_16_disabled=True,proof=proof,gains_restored=True),indent=2)+'\n')
    print('BOTH VERIFIED CENTERED THEN RELAXED; ALL 16 DISABLED',flush=True);break
   time.sleep(.002)
  else:raise RuntimeError('Disabled feedback not confirmed')
  break
 else:raise RuntimeError('Center did not settle; remains POWERED')
finally:
 if gb:gb.close()
 b.close() # Socket close only; NEVER implicitly relax on failure.
