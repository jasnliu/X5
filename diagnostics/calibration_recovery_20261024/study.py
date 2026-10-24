"""Saved-input calibration study. Never opens devices or edits source captures.

Select only using all training poses and the existing deterministic five folds.
Reserved validation is a separate explicit final step, never a tuning input.
"""
import argparse,copy,hashlib,json,time
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from camera_calibration import core,kinematics as kin

SESSION=core.ROOT/'calibration_sessions/20261024T211629Z-1227b7b1'
OUTPUT=Path(__file__).resolve().parent
REAL_LS=kin.least_squares
VARIANTS=[dict(name=f'{prefix}_pose_w{w}',lm=lm,weight=w,objective='pose',loss='linear')
          for prefix,lm in [('raw',False),('lm',True)] for w in (1.,2.,4.)]
VARIANTS += [dict(name='lm_image_bundle',lm=True,weight=1.,objective='image',loss='linear'),
             dict(name='lm_mixed_bundle',lm=True,weight=1.,objective='mixed',loss='linear')]


def read():
    data=json.loads((SESSION/'session.json').read_text())
    K,D=core.camera_parameters(data['camera'])
    return data,K,D,core.Board(SESSION/'target.json'),kin.HandKinematics(SESSION/'model.urdf')


def prepare(rows,K,D,board,lm):
    rows=copy.deepcopy(rows)
    if not lm:return rows
    for x in rows:
        obj=np.ascontiguousarray(board.points[np.asarray(x['corner_ids'],int)])
        uv=np.ascontiguousarray(x['corners_px'],dtype=float);B=np.asarray(x['T_color_board'])
        r=Rotation.from_matrix(B[:3,:3]).as_rotvec().reshape(3,1)
        t=B[:3,3].reshape(3,1).copy()
        r,t=cv2.solvePnPRefineLM(obj,uv,K,D,r,t,
            criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_COUNT,100,1e-12))
        T=core.transform(Rotation.from_rotvec(r.ravel()).as_matrix(),t.ravel())
        image=cv2.projectPoints(obj,r,t,K,D)[0].reshape(-1,2)
        rms=float(np.sqrt(np.mean(np.sum((image-uv)**2,axis=1))))
        if rms>x['reprojection_rms_px']+1e-6:raise ValueError('Per-image refinement worsened pixel fit')
        if np.min((T[:3,:3]@obj.T+T[:3,3:4])[2])<=0:raise ValueError('Board behind camera')
        x['original_T_color_board']=x['T_color_board']
        x['original_reprojection_rms_px']=x['reprojection_rms_px']
        x.update(T_color_board=T.tolist(),reprojection_rms_px=rms)
    return rows


def fit(rows,K,D,board,model,v):
    baseline=core.solve(rows,K,D,board);q=model.measured(rows)
    objects=[np.ascontiguousarray(board.points[np.asarray(x['corner_ids'],int)]) for x in rows]
    observations=[np.asarray(x['corners_px']) for x in rows]
    def ls(fun,seed,**kwargs):
        kwargs['loss']=v['loss']
        def residual(p):
            pose=fun(p).reshape(-1,6);pose[:,3:]*=v['weight']
            if v['objective']=='pose':return pose.ravel()
            C,X=core._unpack(p[:6]),core._unpack(p[6:12])
            A=model.fk(q+np.r_[0.,p[12:],0.]);pred=core.inverse(C)@A@X
            rv=Rotation.from_matrix(pred[:,:3,:3]).as_rotvec()
            errors=[]
            for i,(obj,uv) in enumerate(zip(objects,observations)):
                image=cv2.projectPoints(obj,rv[i],pred[i,:3,3],K,D)[0].reshape(-1,2)
                errors.append((image-uv).ravel())
            pixels=np.concatenate(errors)
            return pixels if v['objective']=='image' else np.r_[pixels,pose.ravel()]
        return REAL_LS(residual,seed,**kwargs)
    kin.least_squares=ls
    try:result=kin._refine(rows,K,D,board,model,baseline)
    finally:kin.least_squares=REAL_LS
    return result,baseline


def evaluate(rows,f,K,D,board):
    corrected=kin.corrected_samples(rows,f['kinematic_correction'],SESSION/'model.urdf')
    return core.metrics(corrected,np.asarray(f['T_base_color']),np.asarray(f['T_hand_board']),K,D,board.points)


def score(m):
    return sum((m[k]['rms']/v)**2 for k,v in [('translation_mm',5),('rotation_deg',1),('reprojection_px',4)])


def brief(m):return {k:v for k,v in m.items() if k!='samples'}


def run(v):
    data,K,D,board,model=read();start=time.monotonic()
    rows=prepare([x for x in data['samples'] if x['phase']=='training'],K,D,board,v['lm'])
    result,baseline=fit(rows,K,D,board,model,v)
    cvrows=[];oldrows=[];folds=[]
    for fold in range(5):
        train=[x for i,x in enumerate(rows) if i%5!=fold]
        test=[x for i,x in enumerate(rows) if i%5==fold]
        f,b=fit(train,K,D,board,model,v)
        cvrows.extend(evaluate(test,f,K,D,board)['samples'])
        oldrows.extend(core.metrics(test,np.asarray(b['T_base_color']),np.asarray(b['T_hand_board']),K,D,board.points)['samples'])
        folds.append(dict(training_ids=[x['id'] for x in train],test_ids=[x['id'] for x in test]))
    cv,old=core.aggregate_metrics(cvrows),core.aggregate_metrics(oldrows);ratio=score(cv)/score(old)
    cvpass=core.passes(cv) and ratio<.8
    report=dict(variant=v,training_count=len(rows),training_ids=[x['id'] for x in rows],
        fit=result,training_passed=result['training_passed'],
        crossvalidation=dict(metrics=cv,baseline=old,ratio=ratio,passed=cvpass,folds=folds),
        all_training_gates_passed=bool(result['training_passed'] and cvpass),
        reserved_validation_used=False,elapsed_s=time.monotonic()-start,
        session_sha256=core.digest(SESSION/'session.json'))
    core.atomic_json(OUTPUT/(v['name']+'.json'),report) if hasattr(core,'atomic_json') else (OUTPUT/(v['name']+'.json')).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(variant=v['name'],training=brief(result['training_metrics']),cv=brief(cv),
        ratio=ratio,passed=report['all_training_gates_passed'],elapsed_s=report['elapsed_s'])),flush=True)
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=[v['name'] for v in VARIANTS]);args=parser.parse_args()
    before={str(p):core.digest(p) for p in SESSION.rglob('*') if p.is_file()}
    for v in VARIANTS:
        if args.variant and v['name']!=args.variant:continue
        try:run(v)
        except Exception as exc:print(v['name'],'FAILED',repr(exc),flush=True)
    assert before=={str(p):core.digest(p) for p in SESSION.rglob('*') if p.is_file()},'Original session changed'


if __name__=='__main__':main()
