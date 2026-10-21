import sys
from pathlib import Path
from drum_planner.baseline.app import job,wait
from drum_planner.baseline.tasks import load_checked
from drum_planner.common import DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can
no_can()
d=Path(sys.argv[1]).resolve();folder=d/'saved_capture_plan'
capture=Path('planner_runs/20261019T195421Z-baseline-907f55/capture').resolve()
wait(job('plan',folder,capture=capture,calibration=DEFAULT_CALIBRATION))
o,r,c=load_checked(folder/'result/plan.json',DEFAULT_CALIBRATION)
assert o['status']=='planned' and (folder/'worker.log').is_file()
atomic_json(d/'worker_replay_report.json',dict(passed=True,saved_capture=str(capture),not_live_acquisition=True,
 plan=str(folder/'result/plan.json'),duration_s=c.duration,motor_commands=0,can_access=False,camera_access=False))
print('Actual worker entrypoint + native planner + checked plan load PASSED on saved acquisition; no devices.',flush=True)
