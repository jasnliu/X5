"""OFFLINE HYPOTHESIS TESTS ONLY. Writes solely to this diagnostic directory.
Does NOT modify or export any production calibration. Reserved validation unused.
"""
import sys,json,copy,time,socket,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path('/home/jason/Proyectos3/X5'); sys.path.insert(0,str(ROOT))
from camera_calibration import core
from safe_zone.geometry import Model

def forbidden(*args,**kwargs): raise RuntimeError('Hardware/process access forbidden in offline diagnosis')
socket.socket=forbidden; subprocess.Popen=forbidden
OUT=Path(__file__).resolve().parent
SESSION=ROOT/'calibration_sessions/20261017T211949Z-762f9908'
S=json.loads((SESSION/'session.json').read_text())
training=[s for s in S['samples'] if s['phase']=='training']
report=json.loads((SESSION/'training_report.json').read_text())
board=core.Board(SESSION/'target.json'); K,D=core.camera_parameters(S['camera'])
Q=np.array([[s['joints_rad'][k] for k in core.JOINTS] for s in training])
A=np.array([s['T_base_hand'] for s in training]); B=np.array([s['T_color_board'] for s in training])
points=[board.points[np.array(s['corner_ids'])] for s in training]
uv=[np.array(s['corners_px']) for s in training]
seed=np.r_[core._pack(np.array(report['T_base_color'])),core._pack(np.array(report['T_hand_board']))]
xml=ET.fromstring((SESSION/'model.urdf').read_text())
parents={j.find('child').get('link'):j for j in xml.findall('joint')}
chain=[]; link=core.HAND
while link!='world':
 j=parents[link]; origin=j.find('origin'); t=np.eye(4)
 if origin is not None:
  t[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
  t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
 name=j.get('name'); ix=core.JOINTS.index(name) if name in core.JOINTS else None
 axis=np.fromstring(j.find('axis').get('xyz'),sep=' ') if ix is not None else None
 chain.insert(0,(t,ix,axis)); link=j.find('parent').get('link')

def fk(q):
 out=np.tile(np.eye(4),(len(q),1,1))
 for origin,ix,axis in chain:
  out=out@origin
  if ix is not None:
   m=np.tile(np.eye(4),(len(q),1,1)); m[:,:3,:3]=Rotation.from_rotvec(q[:,ix,None]*axis).as_matrix(); out=out@m
 return out
assert np.max(abs(fk(Q)-A))<1e-12
model=Model(SESSION/'model.urdf')
assert max(np.max(abs(model.transforms(s['joints_rad'])[core.HAND]-a)) for s,a in zip(training,A))<1e-12

def rigid_residual(C,X,aa,bb):
 lhs=aa@X; rhs=C@bb
 pos=(lhs[:,:3,3]-rhs[:,:3,3])/.005
 rot=Rotation.from_matrix(lhs[:,:3,:3].transpose(0,2,1)@rhs[:,:3,:3]).as_rotvec()/np.radians(1)
 return np.c_[pos,rot].ravel()

def summary(aa,bb,C,X,kk=K,dd=D,scale=1.,indices=None):
 ids=np.arange(len(aa)) if indices is None else np.array(indices)
 samples=[]
 for i,a,b in zip(ids,aa,bb):
  s=copy.copy(training[i]); s['T_base_hand']=a.tolist(); s['T_color_board']=b.tolist(); samples.append(s)
 m=core.metrics(samples,C,X,kk,dd,board.points*scale)
 return {key:m[key] for key in ('translation_mm','rotation_deg','reprojection_px')},m['samples']

results={}
def fit_variant(name, bb=B, kk=K, dd=D, joints=(), board_scale=False, pixel_objective=False, indices=None):
 ids=np.arange(len(A)) if indices is None else np.array(indices); q=Q[ids]; b=bb[ids]
 p0=np.r_[seed,np.zeros(len(joints)),0.] if board_scale else np.r_[seed,np.zeros(len(joints))]
 def get(p):
  c,x=core._unpack(p[:6]),core._unpack(p[6:12]); qq=q.copy()
  for k,j in enumerate(joints): qq[:,j]+=p[12+k]
  aa=fk(qq) if joints else A[ids]
  scale=np.exp(p[-1]) if board_scale else 1.
  b2=b.copy(); b2[:,:3,3]*=scale
  return c,x,aa,b2,scale
 def fun(p):
  c,x,aa,b2,scale=get(p)
  if not pixel_objective: return rigid_residual(c,x,aa,b2)
  pred=core.inverse(c)@aa@x; res=[]
  for i,t in zip(ids,pred):
   image=cv2.projectPoints(points[i]*scale,Rotation.from_matrix(t[:3,:3]).as_rotvec(),t[:3,3],kk,dd)[0].reshape(-1,2)
   res.extend((image-uv[i]).ravel())
  return np.array(res)
 low=np.r_[np.full(12,-np.inf),np.full(len(joints),-np.radians(15))]
 high=np.r_[np.full(12,np.inf),np.full(len(joints),np.radians(15))]
 if board_scale: low=np.r_[low,np.log(.8)]; high=np.r_[high,np.log(1.2)]
 t=time.monotonic(); fit=least_squares(fun,p0,loss='soft_l1',bounds=(low,high),max_nfev=350,xtol=1e-10,ftol=1e-10,gtol=1e-10)
 c,x,aa,b2,scale=get(fit.x); metrics,rows=summary(aa,b2,c,x,kk,dd,scale,ids)
 result=dict(success=bool(fit.success),seconds=time.monotonic()-t,metrics=metrics,samples=rows,
             joint_offsets_deg={str(j+1):float(np.degrees(fit.x[12+k])) for k,j in enumerate(joints)},
             board_scale=scale,T_base_color=c.tolist(),T_hand_board=x.tolist(),parameters=fit.x.tolist(),
             input_B=bb.tolist(),K=kk.tolist(),D=dd.tolist(),training_indices=ids.tolist())
 results[name]=result
 print(name,json.dumps(dict(rms={k:round(v['rms'],5) for k,v in metrics.items()},offsets=result['joint_offsets_deg'],scale=scale,seconds=round(result['seconds'],2))),flush=True)
 (OUT/'hypotheses.json').write_text(json.dumps(results,indent=2)+'\n')
 return result

if __name__=='__main__':
 fit_variant('baseline')
 # PnP optimization only: unchanged camera intrinsics, robot data and target.
 refined=[]
 for i,b in enumerate(B):
  rv=Rotation.from_matrix(b[:3,:3]).as_rotvec().reshape(3,1); tv=b[:3,3].copy().reshape(3,1)
  rv,tv=cv2.solvePnPRefineLM(points[i],uv[i],K,D,rv,tv)
  refined.append(core.transform(cv2.Rodrigues(rv)[0],tv))
 refined=np.array(refined); np.save(OUT/'refined_B.npy',refined)
 fit_variant('pnp_refine_only',bb=refined)
 fit_variant('pixel_objective_only',pixel_objective=True)
 fit_variant('board_scale_only',board_scale=True)
 for j in range(1,6): fit_variant('joint_%d_offset'%(j+1),joints=(j,))
 fit_variant('joints_2_to_6_offsets',joints=tuple(range(1,6)))
 fit_variant('joints_2_to_6_plus_scale',joints=tuple(range(1,6)),board_scale=True)
 # Camera-only intrinsic fits do not use encoder/FK observations.
 objects=[p.astype(np.float32) for p in points]; pixels=[p.astype(np.float32) for p in uv]
 frozen_flags=cv2.CALIB_USE_INTRINSIC_GUESS|cv2.CALIB_RATIONAL_MODEL|cv2.CALIB_FIX_K1|cv2.CALIB_FIX_K2|cv2.CALIB_FIX_K3|cv2.CALIB_FIX_K4|cv2.CALIB_FIX_K5|cv2.CALIB_FIX_K6|cv2.CALIB_FIX_TANGENT_DIST
 for label,flags,k0,d0 in [('intrinsics_K_only',frozen_flags,K.copy(),D.copy()),
                           ('intrinsics_Brown5',cv2.CALIB_USE_INTRINSIC_GUESS,K.copy(),np.zeros(5))]:
  err,kk,dd,rv,tv=cv2.calibrateCamera(objects,pixels,(1280,720),k0,d0,flags=flags,
                                   criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,100,1e-10))
  bb=np.array([core.transform(cv2.Rodrigues(r)[0],t) for r,t in zip(rv,tv)])
  print('CAMERA_FIT',label,'image_rms',err,'K',kk.tolist(),'D',dd.ravel().tolist(),flush=True)
  result=fit_variant(label,bb=bb,kk=kk,dd=dd.ravel())
  result['independent_camera_fit_rms']=err
  (OUT/'hypotheses.json').write_text(json.dumps(results,indent=2)+'\n')
 print('DONE',flush=True)
