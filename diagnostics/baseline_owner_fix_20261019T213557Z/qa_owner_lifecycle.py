"""Real service tick + real file locks + simulated drives. No CAN."""
import fcntl,json,os,sys,time
from pathlib import Path
from drum_planner.baseline import simulation
from drum_planner.baseline.service import Server
from drum_planner.common import atomic_json
from drum_planner.bus_ownership import owners
from drum_planner.experiment.config import no_can
no_can()
d=Path(sys.argv[1]).resolve();folder=d/'mock_owner_retirement';folder.mkdir();locks=folder/'locks';locks.mkdir()
original=simulation.SimulatedBus;created=[]
class LockedMockBus(original):
 def __init__(self,*a,**k):
  super().__init__(*a,**k);self.lock_fds=[];self.closed=False;created.append(self)
  for bus in ('can0','can1'):
   fd=os.open(locks/f'openarmx-cartesian-{bus}.lock',os.O_CREAT|os.O_RDWR,0o600)
   fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB);self.lock_fds.append(fd)
 def close(self):
  for fd in self.lock_fds:os.close(fd)
  self.lock_fds=[];self.closed=True
simulation.SimulatedBus=LockedMockBus
plan=Path.cwd()/'diagnostics/baseline_live_retry_20261019T205948Z/actual_startup/plan/result/plan.json'
info=dict(folder=str(folder),socket=str(folder/'owner.sock'),simulated=True,plan=str(plan),rate=1.,token='test-only')
s=Server(info)
try:
 s.command(dict(op='start',confirmation='MOVE RIGHT ARM'))
 deadline=time.monotonic()+10
 while s.engine.phase!='OUTWARD':
  s.heartbeat=time.monotonic();s.tick();time.sleep(.008)
  assert time.monotonic()<deadline,s.engine.status()
 bus=created[0];s.engine.halt('TEST: GUI disconnected; keep ownership')
 assert len(owners(lock_dir=locks))==2 and not bus.closed
 # An ordinary held owner remains alive, without relaxation.
 end=time.monotonic()+.8
 while time.monotonic()<end:s.tick();time.sleep(.008)
 assert not s.quit and not bus.closed and bus.releases==0
 # Simulate the operator having power-cycled the arm. This is a FAKE bus only.
 bus.enabled=False;targets=bus.targets
 deadline=time.monotonic()+3
 while not s.quit:
  s.tick();time.sleep(.008)
  assert time.monotonic()<deadline,s.engine.status()
 assert s.engine.phase=='DISABLED_EXTERNALLY' and s.engine.disabled
 assert bus.closed and bus.releases==0 and bus.targets==targets and not owners(lock_dir=locks)
 # A new process can acquire the same locks immediately, without unlinking.
 for path in locks.iterdir():
  fd=os.open(path,os.O_RDWR);fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB);os.close(fd)
 atomic_json(d/'owner_lifecycle_report.json',dict(passed=True,phase=s.engine.phase,
  locked_while_held=True,released_after_fresh_external_disable=True,new_lock_acquisition_succeeded=True,
  simulated_disable_commands=bus.releases,extra_targets_after_external_disable=bus.targets-targets,
  physical_motor_commands=0,can_access=False,status=s.status()))
 print('OWNER LIFECYCLE PASSED: held owner retained; externally disabled owner retired; both file locks reusable; zero release commands.',flush=True)
finally:
 s.quit=True;s.run()  # descriptor-only finalization
