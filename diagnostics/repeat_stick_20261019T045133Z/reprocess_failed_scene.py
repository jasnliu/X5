import json
from pathlib import Path
import numpy as np
from drum_planner.depth import snapshot
from drum_planner.common import DEFAULT_CALIBRATION
D=Path(Path('/tmp/x5_repeat_stick_diag').read_text().strip());old=Path('planner_runs/20261019T044701Z-current-plan-a7617e/camera');s=json.loads((old/'scene.json').read_text());raw=np.load(old/'raw_depth.npz');out=D/'reported_failure_reprocessed';out.mkdir();frames=[(dict(h,value_scale_m=float(scale)),a) for h,a,scale in zip(s['frames'],raw['depth'],raw['scales_m'])];snapshot(out,s['camera'],frames,DEFAULT_CALIBRATION)
