"""Diagnostic-only matched old/new path limits. Not imported by --fast mode."""
import json,sys,time
from pathlib import Path
ROOT=Path.cwd();HERE=ROOT/'diagnostics/fast_posture_20261024'
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'diagnostics/r1_r2_wrist_speed_20261024'))
import numpy as np
from drum_planner.common import atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.experiment import fast_timing
import joint_timing
from study import validate,bounds
no_can();fast_timing.MAX_ITERATIONS=600;fast_timing.SOLVE_WALL_SECONDS=8.
out=HERE/'limit_followup';out.mkdir(exist_ok=False)
def read(p):return json.loads(Path(p).read_text())
sources=read(HERE/'inputs/timing_sources.json')
profiles={};original=np.tile(np.array([.6,1.2,6.])[:,None],(1,7));profiles['original']=original
limits=original.copy();limits[:,4]*=[2,4,8];limits[:,5:]*=np.array([4,16,64])[:,None];profiles['distal_faster_body_unchanged']=limits
limits=original.copy();limits[2,:4]*=4;limits[:,4:]*=np.array([4,16,64])[:,None];profiles['proximal_va_fixed_jerk4_distal4']=limits
limits=original.copy();limits[1,:4]*=4;limits[2,:4]*=8;limits[:,4]*=[3,9,27];limits[:,5:]*=np.array([4,16,64])[:,None];profiles['proximal_v_fixed_acc4_jerk8']=limits
rows=[]
for name,source in sources.items():
 for profile,limits in profiles.items():
  t=time.monotonic();law=joint_timing.optimize(source,limits,log=print);calculation=time.monotonic()-t
  proof=validate(law,source,limits,out/(name+'_'+profile),collision=True)
  result=dict(path=name,profile=profile,duration_s=law['duration_s'],calculation_s=calculation,limits=limits.tolist(),
              proof=proof,bounds=bounds(source,limits),warnings=law['details']['warnings'],installed=False,motor_commands=0)
  atomic_json(out/(name+'_'+profile+'.law.json'),law);rows.append(result);atomic_json(out/'results.json',rows)
  print('MATCHED LIMITS',name,profile,round(law['duration_s'],6),law['details']['warnings'],flush=True)
