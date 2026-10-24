"""One 3 GiB, no-CAN worker. Stream logs and preserve its resource report."""
import sys,time
from pathlib import Path
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.experiment.config import no_can
no_can()
folder=Path(sys.argv[1]);job=Job([PYTHON,*sys.argv[2:]],folder,timeout=2400)
try:
 while (code:=job.poll()) is None:time.sleep(.4)
 if code:raise SystemExit(code)
finally:job.close()
