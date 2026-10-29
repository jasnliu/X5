import json
from pathlib import Path
D=Path(__file__).resolve().parent;ROOT=D.parents[1]
cases=[['incident','right','R1',['R2']],['incident','right','R2',['R1']],
       ['incident','right','default',['R1','R2']],['second','right','default',['R1','R2']],
       ['reference','left','L1',[str(ROOT/'diagnostics/experiment_controls_20261025/inputs/L_near.json')]],
       ['incident','left','L1',[str(ROOT/'diagnostics/experiment_controls_20261025/inputs/L_near.json')]]]
folder=D/'variations';folder.mkdir(exist_ok=True)
start=json.loads((ROOT/'planner_runs/20261026T040710Z-experiment-f81b35/acquisition/measured_start.json').read_text())['right_rad']
# Derived simulated starts/targets only; original acquired state remains the
# self-mask pose. Do not rewrite destinations or claim these are measurements.
for sign in (-1,1):
 for name in ('R1','R2'):
  obj=json.loads((ROOT/f'destinations/right/destination{name}.json').read_text())
  for kind in ('start','goal'):
   out=json.loads(json.dumps(obj));q=(start if kind=='start' else obj['samples'][0]['positions_rad'])
   out['samples'][0]['positions_rad']=[v+sign*.005 for v in q]
   out['source']='SOFTWARE sensitivity variation; NOT an encoder measurement'
   target=folder/f'{name}_{kind}_{"minus" if sign<0 else "plus"}.json'
   target.write_text(json.dumps(out,indent=2)+'\n')
   if kind=='start':startfile=target
   else:cases.append(['incident','right',str(startfile),[str(target)]])
(D/'extra_cases.json').write_text(json.dumps(cases,indent=2)+'\n')
