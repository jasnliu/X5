import json,copy
from pathlib import Path
from drum_planner.planning import scene,backend
D=Path('/tmp/x5_repeat_stick_diag').read_text().strip(); root=Path(D)
r=json.loads(Path('planner_runs/20261019T044701Z-current-plan-a7617e/plan/backend_request.json').read_text());r['check_states']=[r['start']]
for f in sorted((root/'depth_series').glob('window*')):
 q=copy.deepcopy(r);q['scene']=scene(f/'scene.json');o=backend(q,f/'check');print(f.name,o['status'],o.get('valid'),o.get('details'),o.get('checks'),flush=True)
