import diagnose as d
import numpy as np,json,copy
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import cv2
OUT=d.OUT
# Avoid overwriting the primary hypothesis report when reusing its fitter.
d.OUT=OUT/'internal_crossvalidation';d.OUT.mkdir(exist_ok=True)
res={};fullidx=np.arange(25)
def crossval(joints):
 rows=[];offsets=[]
 for fold in range(5):
  test=fullidx[fullidx%5==fold];train=fullidx[fullidx%5!=fold]
  sf=d.core.solve([d.training[i] for i in train],d.K,d.D,d.board)
  d.seed=np.r_[d.core._pack(np.array(sf['T_base_color'])),d.core._pack(np.array(sf['T_hand_board']))]
  f=d.fit_variant(('zero' if joints else 'base')+'_fold_'+str(fold),joints=joints,indices=train)
  q=d.Q[test].copy()
  for key,value in f['joint_offsets_deg'].items():q[:,int(key)-1]+=np.radians(value)
  c,x=np.array(f['T_base_color']),np.array(f['T_hand_board'])
  _,r=d.summary(d.fk(q),d.B[test],c,x,indices=test);rows.extend(r);offsets.append(f['joint_offsets_deg'])
 metrics={k:dict(rms=float(np.sqrt(np.mean([r[k]**2 for r in rows]))),maximum=max(r[k] for r in rows)) for k in ('translation_mm','rotation_deg','reprojection_px')}
 return dict(metrics=metrics,offsets=offsets,samples=sorted(rows,key=lambda r:r['sample']))
res['baseline_5fold']=crossval(())
res['joint_offsets_5fold']=crossval(tuple(range(1,6)))
for name,train,test in [('early_to_late',fullidx[:15],fullidx[15:]),('late_to_early',fullidx[10:],fullidx[:10])]:
 results={}
 for variant,joints in [('baseline',()),('joint_offsets',tuple(range(1,6)))]:
  sf=d.core.solve([d.training[i] for i in train],d.K,d.D,d.board);d.seed=np.r_[d.core._pack(np.array(sf['T_base_color'])),d.core._pack(np.array(sf['T_hand_board']))]
  f=d.fit_variant(name+'_'+variant,joints=joints,indices=train);q=d.Q[test].copy()
  for key,v in f['joint_offsets_deg'].items(): q[:,int(key)-1]+=np.radians(v)
  c,x=np.array(f['T_base_color']),np.array(f['T_hand_board']);m,r=d.summary(d.fk(q),d.B[test],c,x,indices=test)
  results[variant]=dict(metrics=m,offsets=f['joint_offsets_deg'])
 res[name]=results
# Rotation-only fit: no board scale/link lengths/translation evidence involved.
init=json.loads((OUT/'hypotheses.json').read_text())['baseline']
rc=Rotation.from_matrix(np.array(init['T_base_color'])[:3,:3]).as_rotvec();rx=Rotation.from_matrix(np.array(init['T_hand_board'])[:3,:3]).as_rotvec()
def angular(p):
 q=d.Q.copy();q[:,1:6]+=p[6:];aa=d.fk(q)
 lhs=aa[:,:3,:3]@Rotation.from_rotvec(p[3:6]).as_matrix();rhs=Rotation.from_rotvec(p[:3]).as_matrix()@d.B[:,:3,:3]
 return (Rotation.from_matrix(lhs.transpose(0,2,1)@rhs).as_rotvec()/np.radians(1)).ravel()
f=least_squares(angular,np.r_[rc,rx,np.zeros(5)],loss='soft_l1',max_nfev=200)
res['rotation_only']=dict(joint_offsets_deg=np.degrees(f.x[6:]).tolist(),angular_rms_deg=float(np.sqrt(np.mean(np.sum(angular(f.x).reshape(-1,3)**2,axis=1)))))
# Approximate board_scale or link-length corrections cannot change these angular invariants.
# Compare no-distortion hypothesis; do not substitute it into any saved dataset.
bb=[]
for p,uv in zip(d.points,d.uv):
 _,rv,tv=cv2.solvePnP(p,uv,d.K,np.zeros(8),flags=cv2.SOLVEPNP_ITERATIVE)
 bb.append(d.core.transform(cv2.Rodrigues(rv)[0],tv))
d.seed=np.r_[d.core._pack(np.array(init['T_base_color'])),d.core._pack(np.array(init['T_hand_board']))]
f=d.fit_variant('no_camera_distortion',bb=np.array(bb),dd=np.zeros(8));res['no_camera_distortion']=f['metrics']
(OUT/'robustness.json').write_text(json.dumps(res,indent=2)+'\n');print('ROBUSTNESS',json.dumps(res,indent=2),flush=True)
