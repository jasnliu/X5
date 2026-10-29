"""Interior generated goals; leave real model/start/path/controller bounds alone."""
import numpy as np
from .config import GOAL_JOINT_RESERVE_DEG,GOAL_J2_LOWER_RESERVE_DEG

# Numerical optimizer roundoff only, many orders below encoder resolution.
ROUND_OFF_RAD=1e-10


def policy():
    return dict(v2_policy(),schema='x5-generated-goal-joint-reserve-v3',j2_operating_floor_deg=0.)


def v2_policy():
    lower=[GOAL_JOINT_RESERVE_DEG]*7;lower[1]=GOAL_J2_LOWER_RESERVE_DEG
    return dict(schema='x5-generated-goal-joint-reserve-v2',scope='generated_endpoint_only',
                lower_deg=lower,upper_deg=[GOAL_JOINT_RESERVE_DEG]*7,
                minimum_deg=GOAL_J2_LOWER_RESERVE_DEG)


def legacy_policy():
    return dict(schema='x5-generated-goal-joint-reserve-v1',scope='generated_endpoint_only',minimum_deg=3.)


def bounds(lower,upper,*,legacy=False):
    lower=np.asarray(lower,float);upper=np.asarray(upper,float)
    p=policy();lo=np.full(7,np.radians(3.)) if legacy else np.radians(p['lower_deg'])
    hi=np.full(7,np.radians(3.)) if legacy else np.radians(p['upper_deg'])
    if (lower.shape!=(7,) or upper.shape!=(7,) or not np.isfinite([lower,upper]).all()
            or np.any(upper-lower<=lo+hi)):
        raise ValueError('Invalid or too narrow model bounds for generated-goal joint reserve')
    result=lower+lo
    if not legacy:result[1]=max(lower[1],0.)+lo[1]
    if np.any(result>=upper-hi):raise ValueError('Model interval has no room for the generated-goal operating reserve')
    return result,upper-hi


def valid(q,lower,upper,*,legacy=False):
    q=np.asarray(q,float);lo,hi=bounds(lower,upper,legacy=legacy)
    return (q.shape==(7,) and bool(np.isfinite(q).all()) and
            bool(np.all(q>=lo-ROUND_OFF_RAD) and np.all(q<=hi+ROUND_OFF_RAD)))


def require(q,lower,upper,*,legacy=False):
    """Never clip an endpoint. Re-solving IK is the only way to change it."""
    if not valid(q,lower,upper,legacy=legacy):
        raise ValueError('Generated goal lacks the required joint-position reserve (3 degrees, J2 goal at least +1 degree); recalculate, do not clip')
    q=np.asarray(q,float);lower=np.asarray(lower,float);upper=np.asarray(upper,float)
    distances=np.minimum(q-lower,upper-q)
    return dict(policy=legacy_policy() if legacy else policy(),model_lower_rad=lower.tolist(),model_upper_rad=upper.tolist(),
        nearest_bound_distance_deg=np.degrees(distances).tolist(),
        minimum_distance_deg=float(np.degrees(distances.min())))


def verify(q,proof,joint_bounds=None,*,allow_legacy=False):
    """Bind saved evidence to the actual goal and, when loading, the real model."""
    legacy=isinstance(proof,dict) and allow_legacy and proof.get('policy')==legacy_policy()
    v2=isinstance(proof,dict) and allow_legacy and proof.get('policy')==v2_policy()
    if not isinstance(proof,dict) or (proof.get('policy')!=policy() and not legacy and not v2):
        raise ValueError('Generated goal joint-position reserve proof is missing/obsolete; recalculate')
    lower=proof.get('model_lower_rad');upper=proof.get('model_upper_rad')
    if joint_bounds is not None and (not np.array_equal(lower,joint_bounds[0]) or
                                    not np.array_equal(upper,joint_bounds[1])):
        raise ValueError('Generated goal joint-position reserve model bounds changed')
    actual=require(q,lower,upper,legacy=legacy)
    if v2:
        if float(lower[1])<0:raise ValueError('Legacy v2 reserve requires its original nonnegative J2 model')
        actual['policy']=v2_policy()
    if proof!=actual:raise ValueError('Generated goal joint-position reserve proof changed')
    return actual
