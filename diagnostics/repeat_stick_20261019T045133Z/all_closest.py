import json
from pathlib import Path
import numpy as np
from drum_planner.depth import deproject
from safe_zone.geometry import Model
from drum_planner.common import JOINTS
D=Path('/tmp/x5_repeat_stick_diag').read_text().strip();r=json.loads(Path('planner_runs/20261019T044701Z-current-plan-a7617e/plan/backend_request.json').read_text());T=Model(r['urdf']).transforms(dict(zip(JOINTS,r['start'])))['right_drumstick']; cam=np.array(r['scene']['T_base_depth']); c=(cam[:3,3]-T[:3,3])@T[:3,:3]
for f in sorted((Path(D)/'depth_series').glob('window*')):
 p=np.fromfile(f/'points_base.f32',dtype='<f4').reshape(-1,3);l=(p-T[:3,3])@T[:3,:3];d=l-c;A=np.sum(d[:,:2]**2,axis=1);B=2*(d[:,:2]@c[:2]);C=c[:2]@c[:2]-.0074**2;Dsc=B*B-4*A*C;root=np.sqrt(np.maximum(Dsc,0));tz=np.sort([(-.2047-c[2])/d[:,2],(.2047-c[2])/d[:,2]],axis=0);lo=np.maximum((-B-root)/(2*A),tz[0]);hi=np.minimum((-B+root)/(2*A),tz[1]);shadow=(Dsc>=0)&(hi>=lo)&(hi>0)&(lo<1);rad=np.linalg.norm(l[:,:2],axis=1);mask=(rad<.0364)&(abs(l[:,2])<.2337);dist=np.hypot(np.maximum(rad-.0074,0),np.maximum(abs(l[:,2])-.2047,0));ind=np.where(~mask&~shadow&(dist<.05))[0]
 print(f.name,[(p[i].round(4).tolist(),round(dist[i],4)) for i in ind])
