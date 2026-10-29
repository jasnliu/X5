"""Tip-constrained, near-parallel posture IK with actual J7-axis alignment.

Only endpoint joint positions are returned. Approach curves are evaluated and
then discarded: no recording, strike, trajectory playback or actuator control.
"""
import time
import numpy as np
from scipy.optimize import least_squares,minimize
from scipy.stats import qmc
from drum_planner.fast_mode.kinematics import Kinematics
from drum_planner.experiment import fast
from drum_planner.experiment.config import settings
from drum_planner.experiment.algorithms import dense_check
from drum_planner.timing import core
from .config import POSITION_TOL_M,ROLL_TOL_DEG,IK_TILT_MODES,ANGLE_CONVENTION,APPROACH_SHORTLIST,MAX_FEASIBLE_APPROACHES
from .config import AIM_CONVENTION,MAX_INWARD_YAW_DEG,EDGE_PREFERENCE_FRACTION
from . import aim,goal_limits


def geometry(kin,q,point,normal,inward):
    f=kin.fk(q,jac=True);a=f['Jrotation'][:,6];velocity=f['Jtip'][:,6]
    down=-velocity/np.linalg.norm(velocity) if np.linalg.norm(velocity)>1e-10 else np.zeros(3)
    # The actual modeled stick axis remains grip -> tip. Positive elevation
    # means the SHAFT back toward the gripper lies above a disc-parallel plane
    # through the tip (the user's R1 reference). Do not rotate/flip any geometry.
    tilt=float(np.degrees(np.arcsin(np.clip(-f['axis']@normal,-1,1))))
    roll=float(np.degrees(np.arcsin(np.clip(abs(a@normal),0,1))))
    return dict(tip_world_m=f['tip'].tolist(),stick_direction=f['axis'].tolist(),j7_axis_world=a.tolist(),
        shaft_direction_tip_to_grip=(-f['axis']).tolist(),angle_convention=ANGLE_CONVENTION,
        tip_error_m=float(np.linalg.norm(f['tip']-point)),stick_tilt_deg=tilt,j7_plane_error_deg=roll,
        negative_j7_tip_direction=down.tolist(),negative_j7_toward_disc_cosine=float(down@(-normal)),
        negative_j7_tip_speed_m_per_rad=float(np.linalg.norm(velocity)),
        kinematic_direction_only=True,strike_generated=False,**aim.measure(f['axis'],normal,inward))


def acceptable(proof):
    # The tiny angular roundoff tolerance only covers equality-solver noise.
    # 90 degrees is the mathematical end of the signed line/plane angle, not
    # an added operating cap. Near-parallel is a preference, NOT an 8-degree gate.
    if proof.get('aim_convention')!=AIM_CONVENTION or proof.get('aim_defined') is not True:return False
    values=[proof[k] for k in ('tip_error_m','stick_tilt_deg','j7_plane_error_deg','negative_j7_toward_disc_cosine',
                              'inward_yaw_deg','inward_cosine','stick_plane_projection_norm')]
    return (bool(np.isfinite(values).all()) and proof.get('angle_convention')==ANGLE_CONVENTION and
        0.<=proof['tip_error_m']<=POSITION_TOL_M and
        -1e-6<=proof['stick_tilt_deg']<=90.+1e-6 and
        proof['j7_plane_error_deg']<=ROLL_TOL_DEG and
        proof['negative_j7_toward_disc_cosine']>0. and
        abs(proof['inward_yaw_deg'])<=MAX_INWARD_YAW_DEG+1e-6 and
        proof['inward_cosine']>0. and proof['stick_plane_projection_norm']>aim.PROJECTION_EPS)


class Task:
    def __init__(self,kin,point,normal,tilt,inward):
        self.kin=kin;self.point=np.asarray(point);self.normal=np.asarray(normal)
        self.sine=None if tilt is None else np.sin(np.radians(tilt))
        self.cone=aim.cone(normal,inward)
        self.last_q=None
    def compute(self,q):
        if self.last_q is not None and np.array_equal(q,self.last_q):return self.residual,self.jacobian
        f=self.kin.fk(q,jac=True);axes=f['Jrotation'];a=axes[:,6]
        ja=np.cross(axes.T,a).T;ja[:,6]=0.
        self.shaft_sine=-f['axis']@self.normal;self.shaft_jac=-self.normal@f['Jaxis']
        # None searches ALL allowed elevations continuously. Penalize only the
        # forbidden side, instead of sampling a few fixed angles up to a cap.
        angular=(min(0.,self.shaft_sine) if self.sine is None else self.shaft_sine-self.sine)
        angular_jac=(np.zeros(7) if self.sine is None and self.shaft_sine>=0 else self.shaft_jac)
        self.aim_margins=self.cone@f['axis'];self.aim_jacobian=self.cone@f['Jaxis']
        self.residual=np.r_[(f['tip']-self.point)/.3,angular,a@self.normal,np.minimum(0.,self.aim_margins)]
        self.jacobian=np.vstack((f['Jtip']/.3,angular_jac,self.normal@ja,
                                self.aim_jacobian*(self.aim_margins<0)[:,None]))
        self.last_q=q.copy();return self.residual,self.jacobian
    def fun(self,q):return self.compute(q)[0]
    def jac(self,q):return self.compute(q)[1]
    def above(self,q):self.compute(q);return self.shaft_sine
    def above_jac(self,q):self.compute(q);return self.shaft_jac
    def aim(self,q):self.compute(q);return self.aim_margins
    def aim_jac(self,q):self.compute(q);return self.aim_jacobian


def refine(kin,start,point,normal,inward,q):
    """Prefer the closest parallel elevation, then improve redundant posture.

    Keep the original accurate fit too: a polished endpoint may collide even
    though another, more elevated posture is clear. Every result is checked.
    """
    task=Task(kin,point,normal,None,inward);indices=[0,1,2,4]
    bounds=list(zip(*goal_limits.bounds(kin.lower,kin.upper)))
    constraints=[dict(type='eq',fun=lambda x:task.fun(x)[indices],jac=lambda x:task.jac(x)[indices]),
                 dict(type='ineq',fun=task.above,jac=task.above_jac),
                 dict(type='ineq',fun=task.aim,jac=task.aim_jac)]
    fit=minimize(task.above,q,jac=task.above_jac,method='SLSQP',bounds=bounds,
        constraints=constraints,options=dict(maxiter=60,ftol=1e-10))
    alternatives=[q]
    if fit.success and goal_limits.valid(fit.x,kin.lower,kin.upper) and acceptable(geometry(kin,fit.x,point,normal,inward)):
        q=fit.x;alternatives.insert(0,q)
    tilt=max(0.,geometry(kin,q,point,normal,inward)['stick_tilt_deg'])
    fixed=Task(kin,point,normal,tilt,inward)
    fit=minimize(lambda x:posture(x,start,kin.lower,kin.upper),q,method='SLSQP',bounds=bounds,
        constraints=[dict(type='eq',fun=lambda x:fixed.fun(x)[:5],jac=lambda x:fixed.jac(x)[:5]),
                     dict(type='ineq',fun=fixed.aim,jac=fixed.aim_jac)],options=dict(maxiter=45,ftol=1e-9))
    if fit.success and goal_limits.valid(fit.x,kin.lower,kin.upper) and acceptable(geometry(kin,fit.x,point,normal,inward)):
        alternatives.insert(0,fit.x)
    return alternatives


def rank(row,score='pose_score'):
    return row[score],-abs(row['geometry']['inward_yaw_deg']),max(0.,row['geometry']['stick_tilt_deg'])


def preference_score(primary,proof):
    # Route/posture quality dominates sideways edge preference, never overrides
    # a hard constraint. Near-parallel remains a soft preference, not an angle cap.
    edge=max(0.,1.-abs(proof['inward_yaw_deg'])/MAX_INWARD_YAW_DEG)
    return primary*(1.+EDGE_PREFERENCE_FRACTION*edge)+max(0.,proof['stick_tilt_deg'])/50.


def posture(q,start,lower,upper):
    w=np.array([1.2,1.2,1.2,1.,.35,.35,.35]);d=q-start
    span=upper-lower;near=np.minimum(q-lower,upper-q)/span
    penalty=np.maximum(0.,.1-near)
    return float(np.sum(w*d*d)+4*np.sum(penalty**2))


def seeds(kin,start):
    unit=qmc.Sobol(7,scramble=False).random_base2(5)
    lower,upper=goal_limits.bounds(kin.lower,kin.upper)
    # Only generated optimizer seeds are brought into the interior; never
    # alter measured poses, targets, physical bounds, or rejected solutions.
    generic=lower+.05*(upper-lower)+unit*.9*(upper-lower)
    seed_start=np.clip(start,lower,upper)  # optimizer seed ONLY; captured start remains exact
    return [seed_start,(lower+upper)/2,*[.6*seed_start+.4*q for q in generic[:16]],*generic[16:]]


def candidates(kin,oracle,start,point,normal,inward,log):
    began=time.monotonic();rows=[];attempts=[];initial=seeds(kin,start);best=None
    inner_bounds=goal_limits.bounds(kin.lower,kin.upper)
    for tilt in IK_TILT_MODES:
        task=Task(kin,point,normal,tilt,inward);found=[]
        for seed in initial:
            fit=least_squares(task.fun,seed,jac=task.jac,bounds=inner_bounds,max_nfev=100,
                              ftol=1e-10,xtol=1e-10,gtol=1e-10)
            q=fit.x;proof=geometry(kin,q,point,normal,inward)
            if best is None or fit.cost<best['cost']:
                best=dict(cost=float(fit.cost),geometry=proof,seed=q.tolist())
            if not acceptable(proof) or not goal_limits.valid(q,kin.lower,kin.upper):continue
            alternatives=refine(kin,start,point,normal,inward,q)
            # Keep the accurate unpolished posture too: the closest-to-start
            # polish can collide even when another redundant posture is clear.
            for q in alternatives:
                if not goal_limits.valid(q,kin.lower,kin.upper):continue
                if any(np.max(abs(q-r['q']))<.03 for r in found):continue
                proof=geometry(kin,q,point,normal,inward)
                if not acceptable(proof):continue
                row=dict(q=q,geometry=proof,posture_cost=posture(q,start,kin.lower,kin.upper),requested_tilt_deg=tilt)
                row['pose_score']=preference_score(.3*row['posture_cost'],proof)
                found.append(row)
        checks=oracle.check([r['q'].tolist() for r in found]) if found else dict(valid=[])
        if len(checks['valid'])!=len(found):raise ValueError('Incomplete endpoint collision reply')
        accepted=0
        for row,valid in zip(found,checks['valid']):
            attempts.append(dict(tilt_deg=tilt,collision_zone_valid=valid,geometry=row['geometry']))
            if valid:
                accepted+=1
                if not any(np.max(abs(row['q']-r['q']))<.03 for r in rows):rows.append(row)
        mode='continuous above-parallel (no angle cap)' if tilt is None else f'{tilt:g}° shaft elevation'
        log(f'IK {mode}: {len(found)} distinct accurate poses; {accepted} collision/zone-valid')
    rows.sort(key=rank)
    report=dict(elapsed_s=time.monotonic()-began,attempts=attempts,endpoint_valid_candidates=len(rows),
                    seeds_per_mode=len(initial),target_m=point.tolist(),disc_normal=normal.tolist(),
                    angle_convention=ANGLE_CONVENTION,maximum_elevation_cap_deg=None,
                    continuous_elevation_search=True,near_parallel_preference=True,
                    goal_joint_reserve_policy=goal_limits.policy())
    report.update(aim_convention=AIM_CONVENTION,inward_reference_world=inward.tolist(),maximum_inward_yaw_deg=MAX_INWARD_YAW_DEG,
        edge_preference_fraction=EDGE_PREFERENCE_FRACTION,ranking='shorter/easier checked approach; soft near-parallel and edge preferences')
    if not attempts:
        from .diagnostic import explain
        diagnostic=explain(kin,start,point,normal,inward,[np.array(best['seed']),*initial],joint_bounds=inner_bounds)
        report['ik_diagnostic']=diagnostic;log(diagnostic['summary'])
    return rows,report


def approach(oracle,request,q,log):
    req=dict(request,goal=q.tolist());report={};began=time.monotonic()
    for row in fast.proposal_stages(oracle,req,settings(),12.,log,report):
        from drum_planner.fast_mode.planner import continuous_joint_bounds
        continuous_joint_bounds(dict(timing=row['timing']),oracle.lower,oracle.upper)
        path=core.original(dict(timing=row['timing']))
        if np.max(abs(path(0)-request['start']))>1e-8 or np.max(abs(path(1)-q))>1e-8:
            raise ValueError('Approach curve does not match the requested endpoints')
        grid=np.unique(np.r_[np.linspace(0,1,2001),path.x]);grid=grid[(grid>=0)&(grid<=1)]
        ok,proof=dense_check(oracle,path(grid))
        if ok:
            travel=float(np.sum(np.abs(np.diff(path(grid),axis=0))*np.array([1.2,1.2,1.2,1.,.35,.35,.35])))
            return dict(valid=True,elapsed_s=time.monotonic()-began,geometry_score=row['score'],
                weighted_joint_travel_rad=travel,
                source=row['source'],metrics=row['workspace'],collision_zone=proof,
                final_endpoint_error_rad=float(np.max(abs(path(1)-q))),
                original_camera_mask=request['scene_robot_start'],timing_computed=False,
                trajectory_discarded=True,recording_generated=False)
    return dict(valid=False,elapsed_s=time.monotonic()-began,reason='No validated approach in bounded workspace search')


def solve(oracle,request,point,normal,inward,log=print):
    began=time.monotonic();kin=Kinematics(request['urdf'],'right');start=np.asarray(request['start'])
    point=np.asarray(point,float);normal=np.asarray(normal,float)
    if (point.shape!=(3,) or normal.shape!=(3,) or not np.isfinite(point).all() or
        not np.isfinite(normal).all() or abs(np.linalg.norm(normal)-1)>1e-8 or normal[2]<0):
        raise ValueError('Expected finite target and unit upward disc normal')
    if start.shape!=(7,) or not np.isfinite(start).all() or np.any(start<kin.lower) or np.any(start>kin.upper):
        raise ValueError('Captured right-arm pose exceeds this model\'s joint limits; not clipped. Recalculate from the actual measured rest.')
    normal,inward=aim.validate(normal,inward)
    log(f'Right J2 operating minimum: 0° in encoder/display coordinates; J2 remains free above it. '
        'Exact acquired-rest transitions are checked separately. '
        'Generated goal reserve: 3° at other bounds; J2 goal minimum +1° regardless of rest.')
    kin.verify_native(oracle,np.array([start,(kin.lower+kin.upper)/2]))
    check=oracle.check(np.array([start]))
    if len(check['valid'])!=1 or not check['valid'][0]:
        return None,dict(reason='Captured start is collision/zone invalid',details=check)
    rows,report=candidates(kin,oracle,start,point,normal,inward,log)
    finalists=[];trials=[]
    for i,row in enumerate(rows[:APPROACH_SHORTLIST]):
        log(f'Approach check {i+1}/{min(len(rows),APPROACH_SHORTLIST)}: tilt {row["geometry"]["stick_tilt_deg"]:.3f}°; inward yaw {row["geometry"]["inward_yaw_deg"]:+.3f}°')
        proof=approach(oracle,request,row['q'],log);trials.append(dict(pose_score=row['pose_score'],**proof))
        if proof['valid']:
            row['approach']=proof
            row['score']=preference_score(proof['weighted_joint_travel_rad']+.25*proof['geometry_score'],row['geometry'])
            finalists.append(row)
            if len(finalists)>=MAX_FEASIBLE_APPROACHES:break
    report.update(approach_trials=trials,elapsed_s=time.monotonic()-began,feasible_finalists=len(finalists),
        finalist_scores=[dict(score=r['score'],geometry=r['geometry'],approach=r['approach']) for r in finalists])
    if not finalists:
        report['reason']=('No IK pose found with the exact tip, orientation and generated-goal joint reserve' if not report['attempts'] else
            'Aligned endpoint candidates fail collision/zone checks') if not rows else 'No checked approach to shortlisted endpoints'
        if report.get('ik_diagnostic'):report['reason']+='; '+report['ik_diagnostic']['summary']
        return None,report
    winner=min(finalists,key=lambda r:rank(r,'score'));winner=dict(winner,q=winner['q'].tolist())
    # Independent final endpoint check after ranking, with native FK equality.
    kin.verify_native(oracle,np.array(winner['q']))
    final=oracle.check([winner['q']])
    if len(final['valid'])!=1 or not final['valid'][0] or not acceptable(geometry(kin,winner['q'],point,normal,inward)):
        raise ValueError('Selected endpoint failed final verification')
    winner['goal_joint_reserve']=goal_limits.require(winner['q'],kin.lower,kin.upper)
    report['selected']=winner
    return winner,report
