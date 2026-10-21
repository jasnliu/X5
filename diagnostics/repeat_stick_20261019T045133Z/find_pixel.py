import json
from pathlib import Path
import numpy as np
from drum_planner.depth import deproject,depth_window
f=Path('planner_runs/20261019T044701Z-current-plan-a7617e/camera'); s=json.loads((f/'scene.json').read_text()); z=np.fromfile(f/'depth_m.bin',dtype='<f4').reshape(800,1280); raw=np.load(f/'raw_depth.npz'); ds=raw['depth']*raw['scales_m'][:,None,None]
p=deproject(z,s['camera'],np.array(s['T_base_depth'])); cells=np.floor(p/.01).astype(int); target=np.array([25,-14,-7]); ids=np.where(np.all(cells==target,axis=1))[0]; vs,us=np.where(z>0)
print('deproject count',len(p),'indices',len(us),'target points',p[ids]);
for i in ids:
 u,v=us[i],vs[i]; print('pixel',u,v,'depths',ds[:,v,u]); print('neighborhoods',ds[:,v-2:v+3,u-2:u+3])
