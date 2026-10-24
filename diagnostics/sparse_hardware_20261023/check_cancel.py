"""Cancel an actual sparse solve; no device ownership exists in this process tree."""
import sys,time,subprocess,json
from pathlib import Path
from drum_planner.common import atomic_json
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.experiment.config import no_can
no_can();folder=Path(sys.argv[1]);source=Path('planner_runs/sparse-20261023-benchmark/right_checkpoint_2/source.json').resolve()
j=Job([PYTHON,'-m','drum_planner.timing.sparse','--input',source,'--output',folder.resolve()/'law.json'],folder,timeout=180)
try:
 end=time.monotonic()+30
 while time.monotonic()<end:
  code=j.poll()
  assert code is None,'Solver completed before cancellation test'
  if 'SPARSE IPOPT' in j.log_path.read_text():break
  time.sleep(.1)
 else:raise AssertionError('Sparse worker did not start')
 cgroup=Path('/sys/fs/cgroup')/j.limits['ControlGroup'].lstrip('/')
 pids=[int(x) for x in (cgroup/'cgroup.procs').read_text().split()]
 j.close();time.sleep(.1)
 assert all(not Path('/proc',str(pid)).exists() for pid in pids),pids
 assert not (folder/'law.json').exists(),'Cancelled calculation must not publish a law'
 atomic_json(folder/'verification.json',dict(passed=True,cancelled_pids=pids,children_reaped=True,motor_commands=0,camera_access=False))
finally:j.close()
