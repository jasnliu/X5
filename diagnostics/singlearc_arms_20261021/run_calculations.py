"""Serial, cgroup-limited, AF_CAN-denied full software acquisitions."""
import time
from pathlib import Path
from drum_planner.common import ROOT,atomic_json
from drum_planner.experiment.runtime import Job
from drum_planner.experiment.config import no_can
no_can()
reports={}
for side in ('right','left'):
    folder=ROOT/'planner_runs/singlearc_arms_20261021'/side/'calculation'
    job=Job(['/usr/bin/python3','-m','diagnostics.singlearc_arms_20261021.verify_calculation',folder,side],folder,timeout=420,allow_can=False)
    try:
        while (code:=job.poll()) is None:time.sleep(.2)
        assert code==0,(side,code,str(job.log_path))
        reports[side]=dict(folder=str(folder),containment=job.limits)
    finally:job.close()
atomic_json(ROOT/'diagnostics/singlearc_arms_20261021/calculation_jobs.json',reports)
