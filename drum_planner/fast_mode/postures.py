"""Bounded coupled task IK: reduce proximal travel, never copy a recording route."""
import time
import numpy as np
from scipy.optimize import minimize
from .kinematics import target,error,task_proof


class BudgetExpired(Exception):pass


def solve(kin,nominal,*,start=None,kind='full-pose',seconds=8.,seed=19):
    nominal=np.asarray(nominal,float);n=len(nominal)
    if n<1 or n>3 or nominal.shape!=(n,7):raise ValueError('Expected one to three destination poses')
    if np.any(nominal<kin.lower) or np.any(nominal>kin.upper):raise ValueError('Captured destination exceeds joint limits')
    if n==1 and start is None:raise ValueError('Single-target optimization needs a fixed start')
    goals=[target(kin,q) for q in nominal];deadline=time.monotonic()+seconds
    rows=[dict(poses=nominal.tolist(),source='captured_postures',weight=0.,cost=None,
               proofs=[task_proof(kin,q,g,kind) for q,g in zip(nominal,goals)])];attempts=[]
    rng=np.random.default_rng(seed);bounds=list(zip(np.tile(kin.lower,n),np.tile(kin.upper,n)))
    rest=nominal[0] if start is None else np.asarray(start,float)
    def constraints(x):
        return np.concatenate([error(kin,q,g,kind) for q,g in zip(x.reshape(n,7),goals)])
    def jacobian(x):
        blocks=[error(kin,q,g,kind,True)[1] for q,g in zip(x.reshape(n,7),goals)]
        m=len(blocks[0]);J=np.zeros((m*n,7*n))
        for i,b in enumerate(blocks):J[i*m:(i+1)*m,i*7:(i+1)*7]=b
        return J
    original_delta=np.diff(nominal,axis=0) if n>1 else nominal-rest
    proximal_cap=float(np.sqrt(original_delta[:,:4]**2+1e-12).sum())
    def proximal_constraint(x):
        q=x.reshape(n,7);d=np.diff(q,axis=0) if n>1 else q-rest
        return proximal_cap-float(np.sqrt(d[:,:4]**2+1e-12).sum())
    def proximal_jacobian(x):
        q=x.reshape(n,7);d=np.diff(q,axis=0) if n>1 else q-rest
        v=np.zeros_like(d);v[:,:4]=d[:,:4]/np.sqrt(d[:,:4]**2+1e-12)
        grad=np.zeros_like(q)
        if n==1:grad-=v
        else:grad[:-1]+=v;grad[1:]-=v
        return grad.ravel()
    for weight in [1.,2.,4.,12.]:
        w=np.array([weight]*4+[1.,1.,1.])
        for trial in range(3):
            if time.monotonic()>deadline:break
            initial=nominal.copy()
            if trial:
                # Shared-body seeds, never historical motion interiors.
                center=nominal.mean(0) if n>1 else rest
                initial[:,:4]=center[:4]+rng.normal(0,.12,(n,4))
                initial[:,4:]+=rng.normal(0,.15,(n,3))
                initial=np.clip(initial,kin.lower+1e-8,kin.upper-1e-8)
            def fun(x):
                if time.monotonic()>deadline:raise BudgetExpired()
                q=x.reshape(n,7);d=np.diff(q,axis=0) if n>1 else q-rest
                # Tiny posture preference resolves redundant near-ties; it is
                # not a route template or an additional physical limit.
                value=float(np.sum((d*w)**2)+.0005*np.sum((q-nominal)**2))
                grad=.001*(q-nominal)
                if n==1:grad+=2*d*w*w
                else:
                    grad[:-1]-=2*d*w*w;grad[1:]+=2*d*w*w
                return value,grad.ravel()
            began=time.monotonic()
            try:
                result=minimize(fun,initial.ravel(),jac=True,method='SLSQP',bounds=bounds,
                    constraints=[dict(type='eq',fun=constraints,jac=jacobian),
                                 dict(type='ineq',fun=proximal_constraint,jac=proximal_jacobian)],
                    options=dict(maxiter=160,ftol=1e-11))
            except BudgetExpired:
                attempts.append(dict(weight=weight,trial=trial,reason='posture budget exhausted'));break
            q=result.x.reshape(n,7);proofs=[task_proof(kin,a,g,kind) for a,g in zip(q,goals)]
            valid=(all(p['valid'] for p in proofs) and np.all(q>=kin.lower) and np.all(q<=kin.upper)
                   and proximal_constraint(result.x)>=-1e-8)
            attempts.append(dict(weight=weight,trial=trial,converged=bool(result.success),
                message=str(result.message),iterations=int(result.nit),elapsed_s=time.monotonic()-began,valid=bool(valid)))
            if valid and not any(np.max(abs(q-np.array(r['poses'])))<1e-4 for r in rows):
                rows.append(dict(poses=q.tolist(),source='coupled_task_ik',weight=weight,
                                 cost=float(result.fun),proofs=proofs))
    for row in rows:
        q=np.array(row['poses']);d=np.diff(q,axis=0) if n>1 else q-rest
        row.update(proximal_travel_rad=float(abs(d[:,:4]).sum()),distal_travel_rad=float(abs(d[:,4:]).sum()),
                   max_joint_transfer_rad=float(np.max(abs(d))),task_constraint=kind)
    rows.sort(key=lambda r:(r['max_joint_transfer_rad'],r['proximal_travel_rad']))
    return rows,dict(attempts=attempts,task_constraint=kind,seed=seed,returned=len(rows),
                     proximal_travel_not_increased=True,proximal_net_travel_cap_rad=proximal_cap,
                     endpoint_targets=[{k:v.tolist() for k,v in g.items()} for g in goals])
