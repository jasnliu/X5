import json,sys
from pathlib import Path
import numpy as np
from safe_zone.geometry import Model
from drum_planner.common import JOINTS
folder=Path(sys.argv[1]); r=json.loads((folder/'plan/backend_request.json').read_text()); s=r['scene']
T=Model(r['urdf']).transforms(dict(zip(JOINTS,r['start'])))['right_drumstick']
p=np.fromfile(s['points_file'],dtype='<f4').reshape(-1,3)
l=(p-T[:3,3])@T[:3,:3]; radial=np.linalg.norm(l[:,:2],axis=1)
c=(np.array(s['T_base_depth'])[:3,3]-T[:3,3])@T[:3,:3]; d=l-c
A=np.sum(d[:,:2]**2,axis=1); B=2*(d[:,:2]@c[:2]); C=c[:2]@c[:2]-.0074**2
D=B*B-4*A*C; disc=np.sqrt(np.maximum(D,0)); t1=(-B-disc)/(2*A); t2=(-B+disc)/(2*A)
tz=np.sort(np.array([(-.2047-c[2])/d[:,2],(.2047-c[2])/d[:,2]]),axis=0)
lo=np.maximum(t1,tz[0]); hi=np.minimum(t2,tz[1]); shadow=(D>=0)&(hi>=lo)&(hi>0)&(lo<1)
mask=(radial<.0074+.029)&(abs(l[:,2])<.2047+.029)
distance=np.hypot(np.maximum(radial-.0074,0),np.maximum(abs(l[:,2])-.2047,0))
retained=np.where(~mask & ~shadow)[0]; inds=retained[np.argsort(distance[retained])[:18]]
Tc=np.array(s['T_base_depth']); pc=(p-Tc[:3,3])@Tc[:3,:3]; K=np.array(s['camera']['depth']['K']); uv=(pc@K.T)[:,:2]/pc[:,2:]
raw=np.load(folder/'camera/raw_depth.npz'); depth=raw['depth']*raw['scales_m'][:,None,None]
print('FOLDER',folder,'pose',r['start'],'mask',sum(mask),'shadow',sum(shadow&~mask))
for i in inds:
 u,v=uv[i].round().astype(int)
 print('p',p[i].round(4),'local',l[i].round(4),'distance',round(distance[i],4),'uv',uv[i].round(2),'z',round(pc[i,2],4),'frames_pixel',depth[:,v,u].round(4).tolist() if 0<=u<1280 and 0<=v<800 else 'outside')
np.savez(Path('/tmp/x5_repeat_stick_diag').read_text().strip()+'/'+folder.name+'_analysis.npz',points=p,local=l,mask=mask,shadow=shadow,uv=uv,distance=distance)
