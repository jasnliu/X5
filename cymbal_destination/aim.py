"""Directed inward aim in the fitted disc plane, independent of camera view.

The measured red target and fitted center are not generally on the same plane.
Project their difference; never substitute world-XY or an unsigned line angle.
"""
import numpy as np
from .config import MAX_INWARD_YAW_DEG,AIM_CONVENTION

PROJECTION_EPS=1e-10  # Numerical undefined-direction guard, not a camera tolerance.


def validate(normal,inward):
    n=np.asarray(normal,float);u=np.asarray(inward,float)
    if (n.shape!=(3,) or u.shape!=(3,) or not np.isfinite(n).all() or not np.isfinite(u).all() or
        abs(np.linalg.norm(n)-1)>1e-8 or abs(np.linalg.norm(u)-1)>1e-8 or abs(n@u)>1e-8):
        raise ValueError('Expected unit disc normal and unit in-plane inward reference')
    return n,u


def reference(target,normal):
    n=np.asarray(normal,float)
    red=np.asarray(target['target']['world_m'],float)
    center=np.asarray(target['flat_disc']['center_world_m'],float)
    if any(v.shape!=(3,) or not np.isfinite(v).all() for v in (n,red,center)) or abs(np.linalg.norm(n)-1)>1e-8:
        raise ValueError('Invalid inward-aim reference geometry')
    delta=center-red;delta-=n*(delta@n);length=np.linalg.norm(delta)
    if length<=PROJECTION_EPS:
        raise ValueError('Inward aim is undefined: surface target projects onto the disc center. No arbitrary direction substituted.')
    return delta/length


def cone(normal,inward):
    n,u=validate(normal,inward);v=np.cross(n,u);t=np.tan(np.radians(MAX_INWARD_YAW_DEG))
    # Both >=0 defines the INWARD wedge, not the reverse wedge of an undirected line.
    return np.array([t*u-v,t*u+v])


def measure(axis,normal,inward):
    n,u=validate(normal,inward);d=np.asarray(axis,float);x=float(d@u);y=float(d@np.cross(n,u))
    length=float(np.hypot(x,y));defined=bool(np.isfinite(length) and length>PROJECTION_EPS)
    return dict(aim_convention=AIM_CONVENTION,aim_defined=defined,
        inward_reference_world=u.tolist(),inward_yaw_deg=float(np.degrees(np.arctan2(y,x))) if defined else 0.,
        inward_cosine=x/length if defined else 0.,stick_plane_projection_norm=length,
        maximum_inward_yaw_deg=MAX_INWARD_YAW_DEG)
