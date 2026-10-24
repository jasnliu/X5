"""Reproducible serial offline checks under the existing 3 GiB job cap."""
import argparse,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.experiment.config import no_can
p=argparse.ArgumentParser();p.add_argument('--task',choices=['benchmark','checkpoint','validate'],required=True);p.add_argument('--output',required=True,type=Path);a=p.parse_args();no_can()
script={'benchmark':'benchmark_gpu.py','checkpoint':'check_checkpoint.py','validate':'validate_replay.py'}[a.task]
job=Job([PYTHON,Path(__file__).with_name(script),'--output',a.output],a.output.parent/('job-'+a.output.name),timeout=900)
try:
 while job.poll() is None:time.sleep(.2)
 if job.poll():raise SystemExit(job.poll())
finally:job.close()
