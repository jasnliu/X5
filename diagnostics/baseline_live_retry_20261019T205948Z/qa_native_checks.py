"""Offline native-FCL check using saved actual-camera data. No current CAN/camera."""
import copy,json,sys,time
from pathlib import Path
import numpy as np
from drum_planner.common import atomic_json,DEFAULT_CALIBRATION
from drum_planner.baseline.tasks import hard_check
from drum_planner.baseline.paths import recovery,recovery_samples
from drum_planner.execution import Curve
from drum_planner.experiment.config import no_can
no_can()
d=Path(sys.argv[1]);source=d/'actual_startup/plan/result/plan.json'
o=json.loads(source.read_text());r=json.loads((source.parent/'backend_request.json').read_text());c=Curve(o)
times=np.r_[np.arange(0,c.duration,.002),c.duration]
report=dict(source=str(source),can_access=False,motor_commands=0,results={})
for name,values in [('full_curve',[c.at(t) for t in times]),('return_prefix',recovery_samples(c,recovery(c,c.at(c.duration*.45),c.duration*.45,1.)))]:
 try:report['results'][name]=hard_check(r,values,d/('native_'+name))
 except Exception as e:report['results'][name]=dict(rejected=True,error=str(e))
# Deliberately invalid single pose must be rejected with native zone checking.
try:
 hard_check(r,[np.ones(7)*2],d/'native_invalid');raise AssertionError('Invalid state accepted')
except ValueError as e:report['invalid_rejected']=str(e)
atomic_json(d/'native_checks.json',report);print(json.dumps(report,indent=2))
for name in ('full_curve','return_prefix'):
 proof=report['results'][name]
 assert proof.get('all_valid') is True, (name,proof)
 assert proof['collision_margin_policy']=='baseline_request'
 assert proof['obstacle_padding_m']==r['scene']['obstacle_padding_m']
 assert proof['minimum_self_clearance_m']==r.get('minimum_self_clearance_m',0.)
 checked=json.loads((d/('native_'+name)/'checked_curve_request.json').read_text())
 assert checked['scene']==r['scene'] and checked.get('minimum_self_clearance_m')==r.get('minimum_self_clearance_m')
 assert checked['right_zone']==r['right_zone']
 assert checked['check_path'] and checked['stop_on_invalid']
assert report['invalid_rejected']
print('NATIVE BASELINE FULL CURVE + PREFIX RECOVERY PASSED; INVALID POSE REJECTED; NO CAN / NO MOTORS',flush=True)
