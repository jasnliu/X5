"""Read-only diagnosis; training-only cross-validation, no held-out validation use."""
import diagnose as d
import numpy as np,json,time,copy
from pathlib import Path
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import cv2
OUT=d.OUT; initial=json.loads((OUT/'hypotheses.json').read_text()); results={}
# Verify images and saved FK/observations, without opening any devices.
replay=[]
for s in d.training:
 image=cv2.imread(str(d.SESSION/s['image']))
 obs=d.board.detect(image,d.K,d.D)
 assert obs['corner_ids']==s['corner_ids']
 replay.append(dict(id=s['id'],pixel_max_delta=float(np.max(abs(np.array(obs['corners_px'])-np.array(s['corners_px'])))),
                    pose_max_delta=float(np.max(abs(np.array(obs['T_color_board'])-np.array(s['T_color_board']))))))
(OUT/'image_replay.json').write_text(json.dumps(replay,indent=2)+'\n')
print('IMAGE_REPLAY_MAX',max(x['pixel_max_delta'] for x in replay),max(x['pose_max_delta'] for x in replay),flush=True)

qt=[]; qv=[]; center=[]
for s in d.training:
 batches=s['stationary_evidence']['encoder_batches']
 qt.append(np.array([(q['start']+q['end'])/2 for q in batches])); qv.append(np.array([[q['joints'][k] for k in d.core.JOINTS] for q in batches])); center.append(s['frame_header']['received_monotonic'])
low=max(t[0]-c for t,c in zip(qt,center)); high=min(t[-1]-c for t,c in zip(qt,center))
print('COMMON_TIME_OFFSET_BOUNDS',low,high,flush=True)

def q_at_lag(lag):
 return np.array([[np.interp(c+lag,t,v[:,j]) for j in range(7)] for c,t,v in zip(center,qt,qv)])

def lag_fun(p):
 aa=d.fk(q_at_lag(p[-1])); c,x=d.core._unpack(p[:6]),d.core._unpack(p[6:12]); return d.rigid_residual(c,x,aa,d.B)
f=least_squares(lag_fun,np.r_[d.seed,0.],bounds=(np.r_[np.full(12,-np.inf),low],np.r_[np.full(12,np.inf),high]),loss='soft_l1',max_nfev=300)
c,x=d.core._unpack(f.x[:6]),d.core._unpack(f.x[6:12]); m,_=d.summary(d.fk(q_at_lag(f.x[-1])),d.B,c,x)
results['best_uniform_timing_offset']=dict(offset_seconds=float(f.x[-1]),bounds=[low,high],metrics=m)
print('TIMING',json.dumps(results['best_uniform_timing_offset']),flush=True)

# Pixel-space comparisons hold the SAME observed corners fixed for all models.
# Keeping global calibration C/X in every variant makes this a matched comparison.
# Only middle joint zeros are observable: J1/J7 constant offsets can be absorbed
# into the free base-camera and hand-board transforms, so they are excluded.
def fit_pixels(model, ids):
 ids=np.array(ids); q=d.Q[ids]; aa0=d.A[ids]
 if model=='baseline': n=0
 elif model=='joint_offsets': n=5
 elif model=='camera_K': n=4
 elif model=='board_scale': n=1
 elif model=='board_anisotropic': n=2
 elif model=='board_bow': n=3
 elif model=='mount_gravity_flex': n=3
 elif model=='joint_offsets_and_bow': n=8
 elif model=='joint_offsets_and_K': n=9
 elif model=='joint_offsets_and_anisotropic': n=7
 else: raise ValueError(model)
 train=[d.training[i] for i in ids]
 seed_fit=d.core.solve(train,d.K,d.D,d.board)
 p0=np.r_[d.core._pack(np.array(seed_fit['T_base_color'])),d.core._pack(np.array(seed_fit['T_hand_board'])),np.zeros(n)]
 low=np.full(12+n,-np.inf); high=np.full(12+n,np.inf)
 if model.startswith('joint_offsets'):
  low[12:17]=-np.radians(10); high[12:17]=np.radians(10)
  p0[12:17]=np.radians(list(initial['joints_2_to_6_offsets']['joint_offsets_deg'].values())) if len(ids)==25 else 0.
 if model in ('board_scale','board_anisotropic'):
  low[12:]=np.log(.9);high[12:]=np.log(1.1)
 if model=='joint_offsets_and_anisotropic': low[17:]=np.log(.9);high[17:]=np.log(1.1)
 if model in ('camera_K','joint_offsets_and_K'):
  ix=17 if model.startswith('joint_offsets') else 12
  low[ix:]=[np.log(.8),np.log(.8),-2.5,-2.5]; high[ix:]=[np.log(1.2),np.log(1.2),2.5,2.5]
 if model in ('board_bow','joint_offsets_and_bow'):
  low[-3:]=-10;high[-3:]=10
 if model=='mount_gravity_flex': low[12:]=-np.radians(10);high[12:]=np.radians(10)
 def predict(p, ids):
  qq=d.Q[ids].copy(); aa=d.A[ids]
  c,x=d.core._unpack(p[:6]),d.core._unpack(p[6:12]); kk=d.K.copy(); obj=d.board.points.copy()
  if model.startswith('joint_offsets'):
   qq[:,1:6]+=p[12:17]; aa=d.fk(qq)
  if model in ('camera_K','joint_offsets_and_K'):
   ix=17 if model.startswith('joint_offsets') else 12
   kk[0,0]*=np.exp(p[ix]);kk[1,1]*=np.exp(p[ix+1]);kk[0,2]+=p[ix+2]*20;kk[1,2]+=p[ix+3]*20
  if model=='board_scale': obj[:,:2]*=np.exp(p[12])
  if model in ('board_anisotropic','joint_offsets_and_anisotropic'):
   ix=17 if model.startswith('joint_offsets') else 12
   obj[:,0]*=np.exp(p[ix]);obj[:,1]*=np.exp(p[ix+1])
  if model in ('board_bow','joint_offsets_and_bow'):
   xx=(obj[:,0]-.12)/.12; yy=(obj[:,1]-.08)/.08
   obj[:,2]=.001*(p[-3]*xx*xx+p[-2]*xx*yy+p[-1]*yy*yy)
  mounts=np.tile(x,(len(ids),1,1))
  if model=='mount_gravity_flex':
   gh=aa[:,:3,:3].transpose(0,2,1)@np.array([0.,0.,-1.])
   torque=np.cross(x[:3,3]/np.linalg.norm(x[:3,3]),gh)
   bend=np.tile(np.eye(4),(len(ids),1,1));bend[:,:3,:3]=Rotation.from_rotvec(torque*p[12:]).as_matrix(); mounts=bend@mounts
  pred=d.core.inverse(c)@aa@mounts
  pixels=[]
  for i,t in zip(ids,pred):
   image=cv2.projectPoints(obj[np.array(d.training[i]['corner_ids'])],Rotation.from_matrix(t[:3,:3]).as_rotvec(),t[:3,3],kk,d.D)[0].reshape(-1,2)
   pixels.append(image)
  return pixels
 def residual(p): return np.concatenate([(x-d.uv[i]).ravel() for x,i in zip(predict(p,ids),ids)])
 t=time.monotonic(); f=least_squares(residual,p0,bounds=(low,high),loss='soft_l1',max_nfev=200,ftol=1e-9,xtol=1e-9)
 per=np.array([np.sqrt(np.mean(np.sum((x-d.uv[i])**2,axis=1))) for i,x in enumerate(predict(f.x,np.arange(len(d.A))))])
 return dict(parameters=f.x.tolist(),success=bool(f.success),all_pose_pixel_rms=per.tolist(),seconds=time.monotonic()-t,fit_indices=ids.tolist())

models=['baseline','joint_offsets','camera_K','board_scale','board_anisotropic','board_bow','mount_gravity_flex','joint_offsets_and_bow','joint_offsets_and_K','joint_offsets_and_anisotropic']
allidx=np.arange(25)
for model in models:
 full=fit_pixels(model,allidx)
 cv=np.empty(25);folds=[]
 for fold in range(5):
  test=allidx[allidx%5==fold]; ids=allidx[allidx%5!=fold]
  fit=fit_pixels(model,ids);cv[test]=np.array(fit['all_pose_pixel_rms'])[test];folds.append(fit)
 results[model]=dict(full=full,full_rms=float(np.sqrt(np.mean(np.array(full['all_pose_pixel_rms'])**2))),
                     crossvalidated_rms=float(np.sqrt(np.mean(cv**2))),crossvalidated_per_pose=cv.tolist(),folds=folds)
 print('PIXEL_MODEL',model,'FULL',results[model]['full_rms'],'CV',results[model]['crossvalidated_rms'],'EXTRA',full['parameters'][12:],flush=True)
 (OUT/'comparisons.json').write_text(json.dumps(results,indent=2)+'\n')
print('DONE',flush=True)
