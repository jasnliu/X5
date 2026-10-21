"""Position-only inverse kinematics for a selected gripper TCP."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import least_squares,minimize
from safe_zone.geometry import TCP,MEMBERSHIP_BUFFER_M

IK_TOLERANCE_M=.002
PATH_PERIOD_S=.02
IK_MAX_NFEV=250
IK_FIT_TOLERANCE=1e-9
# Checking a complete center-return trajectory from every 20 ms point makes a
# long update quadratic in travel time.  Seven evenly spaced anchors include
# both endpoints and keep the preflight bounded while still checking the full
# update trajectory and a complete center return from representative points.
UPDATE_RETURN_ANCHORS=7

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
    def __init__(self,model,zone,lower,upper,speed,side='left',tcp=TCP,home=None,origin=None,
                 fixed_joints=None):
        if side not in ('left','right'):raise ValueError('IK side must be left or right')
        self.model=model;self.zone=zone;self.side=side;self.tcp=tcp
        self.lower=np.asarray(lower,dtype=float);self.upper=np.asarray(upper,dtype=float)
        self.speed=float(speed)
        self.home=np.zeros(7) if home is None else np.asarray(home,dtype=float)
        self.origin=np.zeros(7) if origin is None else np.asarray(origin,dtype=float)
        self.fixed_joints={} if fixed_joints is None else {
            int(index):float(value) for index,value in fixed_joints.items()
        }
        self.center=self.home  # Compatibility with the original left-only implementation.
        self.origin_tcp=self.position(self.origin)
        self.center_tcp=self.origin_tcp  # Cartesian coordinates remain relative to the zero-joint origin.
        if self.lower.shape!=(7,) or self.upper.shape!=(7,) or self.speed<=0:raise ValueError('Invalid IK configuration')
        if self.home.shape!=(7,) or self.origin.shape!=(7,) or np.any(self.home<self.lower) or np.any(self.home>self.upper):
            raise ValueError('Invalid IK home/origin')
        if (any(index not in range(7) for index in self.fixed_joints)
                or any(not math.isfinite(value)
                       or value<self.lower[index] or value>self.upper[index]
                       for index,value in self.fixed_joints.items())):
            raise ValueError('Invalid fixed IK joint')
        self.free_joints=np.array(
            [index for index in range(7) if index not in self.fixed_joints],dtype=int)
        if len(self.free_joints)<3:raise ValueError('At least three IK joints must remain free')

    def position(self,q):
        values={f'openarmx_{self.side}_joint{i+1}':float(q[i]) for i in range(7)}
        return self.model.transforms(values)[self.tcp][:3,3]

    def path_inside(self,goal):
        """Check the nominal firmware-speed path out and back at the GUI period."""
        for start,end in ((self.home,goal),(goal,self.home)):
            for q in self.path_samples(start,end):
                if not self.zone.contains(self.position(q),MEMBERSHIP_BUFFER_M):return False
        return True

    def path_samples(self,start,end):
        start=np.asarray(start,dtype=float);end=np.asarray(end,dtype=float);delta=end-start
        duration=float(np.max(np.abs(delta))/self.speed)
        steps=max(1,int(math.ceil(duration/PATH_PERIOD_S)))
        for n in range(steps+1):
            travel=min(duration,n*duration/steps)*self.speed
            yield start+np.sign(delta)*np.minimum(np.abs(delta),travel)

    def path_between_inside(self,start,end):
        return all(self.zone.contains(self.position(q),MEMBERSHIP_BUFFER_M)
                   for q in self.path_samples(start,end))

    def update_transition_inside(self,start,end):
        """Validate the full update plus bounded immediate-center returns."""
        samples=list(self.path_samples(start,end))
        if not all(self.zone.contains(self.position(q),MEMBERSHIP_BUFFER_M) for q in samples):
            return False
        count=min(UPDATE_RETURN_ANCHORS,len(samples))
        evenly=np.unique(np.linspace(0,len(samples)-1,count,dtype=int)).tolist()
        # The new goal-to-center path is the most important new return and the
        # most common rejection.  Check it first instead of repeatedly checking
        # the already-held start-to-center path before discovering a bad goal.
        indices=[evenly[-1],*evenly[1:-1],evenly[0]] if len(evenly)>1 else evenly
        for index in indices:
            if not self.path_between_inside(samples[index],self.home):return False
        return True

    def _fit(self,target,seed):
        seed=np.clip(np.asarray(seed,dtype=float),self.lower,self.upper)
        if self.fixed_joints:
            for index,value in self.fixed_joints.items():seed[index]=value
            free=self.free_joints
            def expand(values):
                joints=seed.copy();joints[free]=values
                return joints
            fit=least_squares(lambda values:self.position(expand(values))-target,seed[free],
                              bounds=(self.lower[free],self.upper[free]),max_nfev=IK_MAX_NFEV,
                              ftol=IK_FIT_TOLERANCE,xtol=IK_FIT_TOLERANCE,
                              gtol=IK_FIT_TOLERANCE)
            joints=expand(fit.x)
            error=float(np.linalg.norm(self.position(joints)-target))
            return joints,error
        fit=least_squares(lambda q:self.position(q)-target,seed,
                          bounds=(self.lower,self.upper),max_nfev=IK_MAX_NFEV,
                          ftol=IK_FIT_TOLERANCE,xtol=IK_FIT_TOLERANCE,
                          gtol=IK_FIT_TOLERANCE)
        error=float(np.linalg.norm(self.position(fit.x)-target))
        return fit.x.copy(),error

    def _refine(self,target,reference,candidate):
        if self.fixed_joints:
            reference=np.asarray(reference,dtype=float);candidate=np.asarray(candidate,dtype=float).copy()
            for index,value in self.fixed_joints.items():candidate[index]=value
            free=self.free_joints
            def expand(values):
                joints=candidate.copy();joints[free]=values
                return joints
            refined=minimize(lambda values:.5*float((expand(values)-reference)@(expand(values)-reference)),
                             candidate[free],method='SLSQP',
                             bounds=list(zip(self.lower[free],self.upper[free])),
                             constraints={'type':'eq','fun':lambda values:self.position(expand(values))-target},
                             options={'maxiter':150,'ftol':1e-10})
            if not refined.success:return None
            joints=expand(refined.x);error=float(np.linalg.norm(self.position(joints)-target))
            return (joints,error) if error<=IK_TOLERANCE_M else None
        refined=minimize(lambda q:.5*float((q-reference)@(q-reference)),candidate,
                         method='SLSQP',bounds=list(zip(self.lower,self.upper)),
                         constraints={'type':'eq','fun':lambda q:self.position(q)-target},
                         options={'maxiter':150,'ftol':1e-10})
        if not refined.success:return None
        error=float(np.linalg.norm(self.position(refined.x)-target))
        return (refined.x.copy(),error) if error<=IK_TOLERANCE_M else None

    @staticmethod
    def _distinct_seeds(seeds):
        distinct=[]
        for seed in seeds:
            value=np.asarray(seed,dtype=float)
            if not any(np.allclose(value,old,rtol=0.,atol=1e-9) for old in distinct):
                distinct.append(value)
        return distinct

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
        candidates=[]
        seeds=self._distinct_seeds((self.home,self.origin,*[np.radians(d) for d in seed_degrees]))
        for seed in seeds:
            joints,error=self._fit(target,seed)
            if error<=IK_TOLERANCE_M:
                candidates.append((float(np.linalg.norm(joints-self.home)),joints,error))
        if not candidates:
            raise ValueError('Requested coordinate is not reachable within 2 mm')

        best=None
        for _,candidate,_ in sorted(candidates,key=lambda item:item[0]):
            if self.path_inside(candidate):
                best=candidate
                break
        if best is None:raise ValueError('A joint solution was found, but its path leaves zone1')
        # Refine the chosen solution toward the smallest joint displacement while
        # keeping the requested Cartesian point as an equality constraint.
        refined=self._refine(target,self.home,best)
        if refined is not None and self.path_inside(refined[0]):best=refined[0]
        error=float(np.linalg.norm(self.position(best)-target))
        return IKResult(best,target,error)

    def solve_update(self,offset,start):
        """Solve a changed goal near the held pose and preflight its live transition."""
        offset=parse_xyz(offset);start=np.asarray(start,dtype=float);target=self.origin_tcp+offset
        if (start.shape!=(7,) or not np.isfinite(start).all()
                or np.any(start<self.lower) or np.any(start>self.upper)):
            raise ValueError('Invalid current joint position for goal update')
        if not self.zone.contains(target,MEMBERSHIP_BUFFER_M):
            raise ValueError('Updated coordinate is outside zone1')
        if not self.zone.contains(self.position(start),MEMBERSHIP_BUFFER_M):
            raise ValueError('Current gripper position is outside zone1')

        seed_degrees=(
            (0,0,0,0,0,0,0),(30,0,0,30,0,0,-30),(-30,0,0,30,0,0,-30),
            (0,-25,0,30,0,15,-30),(30,-20,20,50,-20,15,-40),
            (-30,-20,-20,50,20,-15,-40),(20,-40,40,60,-40,20,20),
            (-20,-40,-40,60,40,-20,20))
        # The held pose is normally an excellent seed.  Try and validate it
        # first so ordinary coordinate edits do not pay for every fallback IK
        # branch.  Fallbacks are used only when the local branch is unreachable
        # or unsafe.
        seeds=self._distinct_seeds((start,self.home,self.origin,*[np.radians(d) for d in seed_degrees]))
        any_solution=False
        for seed in seeds:
            candidate,error=self._fit(target,seed)
            if error>IK_TOLERANCE_M:continue
            any_solution=True
            refined=self._refine(target,start,candidate)
            # Refinement preserves the Cartesian point while minimizing motion
            # from the held posture.  Do not repeat the expensive path check for
            # its nearly identical unrefined precursor.
            choice=refined[0] if refined is not None else candidate
            if self.update_transition_inside(start,choice):
                error=float(np.linalg.norm(self.position(choice)-target))
                return IKResult(choice,target,error)
        if any_solution:
            raise ValueError('An updated joint solution was found, but its update or center-return path leaves zone1')
        raise ValueError('Updated coordinate is not reachable within 2 mm')
