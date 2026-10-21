import json,sys
from pathlib import Path
from collections import Counter
import numpy as np
root=Path(__file__).resolve().parent
out={}
for stage in ['development','refine','heldout']:
 rows=[json.loads(p.read_text()) for p in sorted((root/stage).glob('*/evaluation.json'))]
 if not rows:continue
 data={}
 for method in rows[0]['methods']:
  found=[r['methods'][method] for r in rows if r['methods'][method]]
  measure={
   'excess_deg':lambda r:r['gesture']['joint_excess_deg'],
   'wrist_detour':lambda r:r['gesture']['wrist']['detour_ratio'],
   'wrist_turn_deg':lambda r:r['gesture']['wrist']['broad_turning_deg'],
   'tip_overshoot_cm':lambda r:100*r['gesture']['tip']['positive_x_goal_overshoot_m'],
   'tip_shape_rms_cm':lambda r:r['similarity']['tip_chord_residual_rms_cm'],
   'joint_shape_rms_deg':lambda r:r['similarity']['joint_chord_residual_rms_deg'],
   'clearance_estimate_mm':lambda r:1000*r['clearance_m'],
   'proximity':lambda r:r['proximity'],
   'duration_s':lambda r:r['duration_s']}
  data[method]=dict(valid=len(found),total=len(rows),sources=dict(Counter(r['source'] for r in found)),
   metrics={name:dict(median=float(np.median([fn(r) for r in found])),minimum=float(min(fn(r) for r in found)),maximum=float(max(fn(r) for r in found))) for name,fn in measure.items()})
 out[stage]=dict(cases=len(rows),methods=data,identical_selections={a:{b:sum(r['methods'][a]['index']==r['methods'][b]['index'] for r in rows if r['methods'][a] and r['methods'][b]) for b in data} for a in data})
 print('\n'+stage.upper(),len(rows),'trials; medians')
 for method,d in data.items():print(method,d['valid'],d['sources'],{n:round(v['median'],2) for n,v in d['metrics'].items()})
(root/'summary.json').write_text(json.dumps(out,indent=2)+'\n')
