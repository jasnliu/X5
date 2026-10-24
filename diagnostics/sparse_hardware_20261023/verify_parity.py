"""Compare actual sparse UI calculations with preserved default UI calculations."""
import json
from pathlib import Path
import numpy as np
from drum_planner.common import atomic_json
from drum_planner.timing.motion_quality import law_metrics
rows=[]
for side in ('right','left'):
 old=json.loads(Path(f'planner_runs/optimization-20261022-ui-{side}/verification.json').read_text())
 new=json.loads(Path(f'planner_runs/sparse-20261023-ui-{side}/verification.json').read_text())
 for aa,bb in zip(old['rows'],new['rows']):
  a=json.loads(Path(aa['plan']).read_text());b=json.loads(Path(bb['plan']).read_text())
  assert b['hardware_variant']=='sparse_ipopt'
  if 'checkpoint_sequence' not in a:
   assert a['path_waypoints']==b['path_waypoints'] and a['source_timing']==b['source_timing']
   pairs=[(a['timing'],b['timing'])]
  else:
   sa=a['checkpoint_sequence'];sb=b['checkpoint_sequence']
   pairs=[(x['timing'],y['timing']) for x,y in zip(sa['segments']+[sa['return_leg']],sb['segments']+[sb['return_leg']])]
  for i,(x,y) in enumerate(pairs):
   assert x['path']==y['path'];before=law_metrics(x);after=law_metrics(y)
   assert abs(x['duration_s']-y['duration_s'])<1e-5
   assert after['internal_stops']<=before['internal_stops']
   assert after['deep_slowdowns']<=before['deep_slowdowns']
   assert abs(after['interior_speed_cv']-before['interior_speed_cv'])<1e-4
   rows.append(dict(side=side,checkpoints=aa['checkpoints'],leg=i+1,path_bit_identical=True,
      original_duration_s=x['duration_s'],sparse_duration_s=y['duration_s'],before_quality=before,after_quality=after))
atomic_json('diagnostics/sparse_hardware_20261023/parity.json',dict(passed=True,rows=rows,motor_commands=0,camera_access=False))
print('PASS: identical path coefficients and matching duration/fluidity across',len(rows),'legs')
