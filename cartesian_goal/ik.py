"""Position-only inverse kinematics for a selected gripper TCP."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import least_squares,minimize
from safe_zone.geometry import TCP,MEMBERSHIP_BUFFER_M

IK_TOLERANCE_M=.002
PATH_PERIOD_S=.02

@dataclass(frozen=True)
class IKResult:
    joints:np.ndarray
    target:np.ndarray
    error_m:float

def parse_xyz(values):
    try:offset=np.asarray([float(v) for v in values],dtype=float)
    except (TypeError,ValueError):raise ValueError('Enter numeric X, Y and Z coordinates in meters')
    if offset.shape!=(3,) or not np.isfinite(offset).all():raise ValueError('Enter finite X, Y and Z coordinates')
    return offset

class CartesianIK:
    def __init__(self,model,zone,lower,upper,speed,side='left',tcp=TCP,home=None,origin=None):
        if side not in ('left','right'):raise ValueError('IK side must be left or right')
        self.model=model;self.zone=zone;self.side=side;self.tcp=tcp
        self.lower=np.asarray(lower,dtype=float);self.upper=np.asarray(upper,dtype=float)
        self.speed=float(speed)
        self.home=np.zeros(7) if home is None else np.asarray(home,dtype=float)
        self.origin=np.zeros(7) if origin is None else np.asarray(origin,dtype=float)
        self.center=self.home  # Compatibility with the original left-only implementation.
        self.origin_tcp=self.position(self.origin)
        self.center_tcp=self.origin_tcp  # Cartesian coordinates remain relative to the zero-joint origin.
        if self.lower.shape!=(7,) or self.upper.shape!=(7,) or self.speed<=0:raise ValueError('Invalid IK configuration')
        if self.home.shape!=(7,) or self.origin.shape!=(7,) or np.any(self.home<self.lower) or np.any(self.home>self.upper):
            raise ValueError('Invalid IK home/origin')

    def position(self,q):
        values={f'openarmx_{self.side}_joint{i+1}':float(q[i]) for i in range(7)}
        return self.model.transforms(values)[self.tcp][:3,3]

    def path_inside(self,goal):
        """Check the nominal firmware-speed path out and back at the GUI period."""
        for start,end in ((self.home,goal),(goal,self.home)):
            delta=np.asarray(end)-start
            duration=float(np.max(np.abs(delta))/self.speed)
            steps=max(1,int(math.ceil(duration/PATH_PERIOD_S)))
            for n in range(steps+1):
                travel=min(duration,n*duration/steps)*self.speed
                q=start+np.sign(delta)*np.minimum(np.abs(delta),travel)
                if not self.zone.contains(self.position(q),MEMBERSHIP_BUFFER_M):return False
        return True

    def solve(self,offset):
        offset=parse_xyz(offset);target=self.origin_tcp+offset
        if not self.zone.contains(target,MEMBERSHIP_BUFFER_M):
            raise ValueError('Requested coordinate is outside zone1')
        if np.linalg.norm(offset)<1e-10 and self.path_inside(self.origin):return IKResult(self.origin.copy(),target,0.)

        # Deterministic nearby seeds avoid the straight-arm singularity while
        # preferring solutions that do not make unnecessarily large movements.
        seed_degrees=(
            (0,0,0,0,0,0,0),(30,0,0,30,0,0,-30),(-30,0,0,30,0,0,-30),
            (0,-25,0,30,0,15,-30),(30,-20,20,50,-20,15,-40),
            (-30,-20,-20,50,20,-15,-40),(20,-40,40,60,-40,20,20),
            (-20,-40,-40,60,40,-20,20))
        candidates=[];unsafe_path=False
        for seed in (self.home,self.origin,*[np.radians(d) for d in seed_degrees]):
            seed=np.clip(seed,self.lower,self.upper)
            fit=least_squares(lambda q:self.position(q)-target,seed,bounds=(self.lower,self.upper),
                              max_nfev=600,ftol=1e-11,xtol=1e-11,gtol=1e-11)
            error=float(np.linalg.norm(self.position(fit.x)-target))
            if error<=IK_TOLERANCE_M:
                if self.path_inside(fit.x):candidates.append((float(np.linalg.norm(fit.x-self.home)),fit.x.copy(),error))
                else:unsafe_path=True
        if not candidates:
            if unsafe_path:raise ValueError('A joint solution was found, but its path leaves zone1')
            raise ValueError('Requested coordinate is not reachable within 2 mm')

        _,best,_=min(candidates,key=lambda item:item[0])
        # Refine the chosen solution toward the smallest joint displacement while
        # keeping the requested Cartesian point as an equality constraint.
        refined=minimize(lambda q:.5*float((q-self.home)@(q-self.home)),best,method='SLSQP',bounds=list(zip(self.lower,self.upper)),
                         constraints={'type':'eq','fun':lambda q:self.position(q)-target},
                         options={'maxiter':500,'ftol':1e-12})
        if refined.success:
            error=float(np.linalg.norm(self.position(refined.x)-target))
            if error<=IK_TOLERANCE_M and self.path_inside(refined.x):best=refined.x.copy()
        error=float(np.linalg.norm(self.position(best)-target))
        return IKResult(best,target,error)
