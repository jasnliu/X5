import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.experiment.runtime import Job,PYTHON
folder=Path(sys.argv[1]);job=Job(['/usr/bin/env','MESA_SHADER_CACHE_DISABLE=true',PYTHON,*sys.argv[2:]],folder,timeout=900)
try:
 while job.poll() is None:time.sleep(.1)
 if job.poll():raise SystemExit(job.poll())
finally:job.close()
