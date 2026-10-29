"""Bounded, non-exportable IK failure explanation; never relax a constraint."""
import time
import numpy as np
from scipy.optimize import minimize
from .config import MAX_INWARD_YAW_DEG


def nearest_inward_diagnostic(kin,point,normal,inward,initial,lower,upper):
    """Explain an aiming conflict without exporting or accepting a relaxed goal.

    Keep exact tip, exact J7 roll, shaft-above and every supplied joint reserve.
    Minimize absolute yaw only for diagnosis; this is NOT a global reachability
    proof and the results must never enter the production candidate list.
    """
    from .solver import Task,geometry
    from .config import POSITION_TOL_M,ROLL_TOL_DEG
    task=Task(kin,point,normal,None,inward);indices=[0,1,2,4]
    tangent=np.cross(normal,inward);found=[]
    def yaw(q):
        f=kin.fk(q,jac=True);x=f['axis']@inward;y=f['axis']@tangent
        angle=np.arctan2(y,x)
        jac=(x*(tangent@f['Jaxis'])-y*(inward@f['Jaxis']))/max(x*x+y*y,1e-20)
        return angle,jac
    for seed in initial[:18]:
        fit=minimize(lambda q:yaw(q)[0]**2,seed,jac=lambda q:2*yaw(q)[0]*yaw(q)[1],
            method='SLSQP',bounds=list(zip(lower,upper)),constraints=[
                dict(type='eq',fun=lambda q:task.fun(q)[indices],jac=lambda q:task.jac(q)[indices]),
                dict(type='ineq',fun=task.above,jac=task.above_jac)],options=dict(maxiter=110,ftol=1e-10))
        if not fit.success or np.any(fit.x<lower-1e-10) or np.any(fit.x>upper+1e-10):continue
        p=geometry(kin,fit.x,point,normal,inward)
        if (not p['aim_defined'] or p['inward_cosine']<=0 or p['tip_error_m']>POSITION_TOL_M
                or p['j7_plane_error_deg']>ROLL_TOL_DEG or p['stick_tilt_deg']< -1e-6
                or p['negative_j7_toward_disc_cosine']<=0):continue
        near=np.minimum(fit.x-lower,upper-fit.x)
        found.append(dict(geometry=p,active_joint_bounds=[
            dict(joint=f'J{i+1}',side='lower' if fit.x[i]-lower[i]<upper[i]-fit.x[i] else 'upper')
            for i in range(7) if near[i]<1e-4]))
    if not found:return None
    return dict(min(found,key=lambda r:abs(r['geometry']['inward_yaw_deg'])),
        diagnostic_only=True,not_a_destination=True,collision_validated=False,
        global_impossibility_proven=False,relaxed_for_diagnosis_only='inward_yaw',
        requested_maximum_inward_yaw_deg=MAX_INWARD_YAW_DEG)


def explain(kin, start, point, normal, inward, initial,*,joint_bounds=None):
    from .solver import Task, geometry,acceptable
    began=time.monotonic();task=Task(kin,point,normal,0.,inward)
    # Exact tip + tangent J7 axis; find a near-parallel fit on the approved
    # shaft-above side. This is a numerical diagnostic, NOT a global proof.
    indices=[0,1,2,4]
    eq=lambda q:task.fun(q)[indices]
    jac=lambda q:task.jac(q)[indices]
    lower,upper=(kin.lower,kin.upper) if joint_bounds is None else joint_bounds
    bounds=list(zip(lower,upper));found=[]
    for seed in initial[:18]:
        fit=minimize(lambda q:-kin.fk(q)['axis']@normal,seed,
            jac=lambda q:-normal@kin.fk(q,jac=True)['Jaxis'],method='SLSQP',bounds=bounds,
            constraints=[dict(type='eq',fun=eq,jac=jac),dict(type='ineq',fun=task.above,jac=task.above_jac),
                         dict(type='ineq',fun=task.aim,jac=task.aim_jac)],
            options=dict(maxiter=110,ftol=1e-10))
        if not fit.success or np.linalg.norm(eq(fit.x))>1e-7:continue
        proof=geometry(kin,fit.x,point,normal,inward)
        if not acceptable(proof):continue
        near=np.minimum(fit.x-lower,upper-fit.x)
        found.append(dict(geometry=proof,active_joint_bounds=[
            dict(joint=f'J{i+1}',side='lower' if fit.x[i]-lower[i]<upper[i]-fit.x[i] else 'upper',
                 value_rad=float(fit.x[i])) for i in range(7) if near[i]<1e-4]))
    out=dict(elapsed_s=time.monotonic()-began,global_impossibility_proven=False,
        diagnostic_only=True,not_a_destination=True,limits_unchanged=True,collision_validated=False,
        bounds_scope='original_model' if joint_bounds is None else 'generated_goal_reserve')
    if found:
        best=min(found,key=lambda row:row['geometry']['stick_tilt_deg']);out['best_exact_tip_found']=best
        tilt=best['geometry']['stick_tilt_deg'];bound=', '.join(x['joint']+' '+x['side'] for x in best['active_joint_bounds'])
        out['summary']=f'Exact-tip/inward-aim diagnostic: closest shaft-above elevation found {tilt:+.2f} degrees; active bounds: {bound or "none"}. '
        out['summary']+='Diagnostic only, not collision-validated or exported; not an impossibility proof.'
    else:
        out['summary']=f'No exact-tip, J7-aligned, inward ±{MAX_INWARD_YAW_DEG:g}° diagnostic fit found within the bounded search.'
        nearest=nearest_inward_diagnostic(kin,point,normal,inward,initial,lower,upper)
        if nearest:
            out['nearest_inward_diagnostic']=nearest
            yaw=nearest['geometry']['inward_yaw_deg']
            active=', '.join(x['joint']+' '+x['side'] for x in nearest['active_joint_bounds']) or 'none'
            out['summary']+=f' Best exact-tip diagnostic found yaw {yaw:+.2f}°; active goal bounds: {active}. '
            out['summary']+='No target/limit changes and no goal exported; bounded diagnostic, not an impossibility proof.'
    out['elapsed_s']=time.monotonic()-began
    return out
