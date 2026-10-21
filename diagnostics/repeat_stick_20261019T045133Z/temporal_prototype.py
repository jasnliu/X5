import json,copy
from pathlib import Path
import numpy as np
from drum_planner.depth import deproject,occupied_voxels
from drum_planner.planning import backend
D=Path('/tmp/x5_repeat_stick_diag').read_text().strip();root=Path(D);r=json.loads(Path('planner_runs/20261019T044701Z-current-plan-a7617e/plan/backend_request.json').read_text());r['check_states']=[r['start'],r['goal']]
folders=[Path('planner_runs/20261019T044701Z-current-plan-a7617e/camera')]+sorted((root/'depth_series').glob('window*'))
for i,f in enumerate(folders):
 s=json.loads((f/'scene.json').read_text());raw=np.load(f/'raw_depth.npz');z=raw['depth']*raw['scales_m'][:,None,None];good=(raw['depth']>0)&(raw['depth']<65535)&(z>=.15)&(z<=5.)
 # retain nearest reading which belongs to a mutually depth-consistent three-frame cluster
 z=np.where(good,z,np.inf); ordered=np.sort(z,axis=0); valid=np.zeros(z.shape[1:],bool); out=np.zeros(z.shape[1:])
 for j in range(2,-1,-1):
  ok=np.isfinite(ordered[j+2])&(ordered[j+2]<=ordered[j]+.02);out=np.where(ok,ordered[j],out);valid|=ok
 q=copy.deepcopy(r);p=root/f'prototype_{i:02}';p.mkdir(exist_ok=True);cloud=occupied_voxels(deproject(out,s['camera'],s['T_base_depth']));cloud.astype('<f4').tofile(p/'points.f32');s['points_file']=str(p/'points.f32');q['scene']=s
 res=backend(q,p);print(f.name,[(c['valid'],c.get('reason')) for c in res['checks']], 'pixels',np.count_nonzero(out),'old',np.count_nonzero(good.any(axis=0)),flush=True)
