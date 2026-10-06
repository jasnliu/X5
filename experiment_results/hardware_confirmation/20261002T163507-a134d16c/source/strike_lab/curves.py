"""Analytic position/velocity/acceleration curves; depth is positive downward."""
import math
from .config import TARGET


def smooth(u):
    u = max(0., min(1., u))
    return u*u*u*(10+u*(-15+6*u))


class Quintic:
    def __init__(self, p0, v0, a0, p1, v1, a1, duration):
        self.duration = duration
        t = duration
        d = p1-p0
        self.c = (p0, v0*t, .5*a0*t*t,
                  10*d-6*v0*t-4*v1*t-1.5*a0*t*t+.5*a1*t*t,
                  -15*d+8*v0*t+7*v1*t+1.5*a0*t*t-a1*t*t,
                  6*d-3*(v0+v1)*t-.5*(a0-a1)*t*t)

    def at(self, elapsed):
        u = max(0., min(1., elapsed/self.duration))
        c = self.c
        p = sum(c[i]*u**i for i in range(6))
        v = sum(i*c[i]*u**(i-1) for i in range(1, 6))/self.duration
        a = sum(i*(i-1)*c[i]*u**(i-2) for i in range(2, 6))/self.duration**2
        return p, v, a


class Dip:
    """One bottom at exactly TARGET, nonzero upward acceleration through it."""
    def __init__(self, down=.22, up=.20, curvature=.8, knot=None, boost=1.):
        self.down, self.up = down, up
        self.duration = down+up
        a = -curvature*min(4*TARGET/down**2, 4*TARGET/up**2)
        self.descent = Quintic(0, 0, 0, TARGET, 0, a, down)
        self.ascent = Quintic(TARGET, 0, a, 0, 0, 0, up)
        self.knot = knot
        if knot is not None:
            self.knot_at = down*knot
            p, v, acc = self.descent.at(self.knot_at)
            self.first = Quintic(0, 0, 0, p, v*boost, acc, self.knot_at)
            self.second = Quintic(p, v*boost, acc, TARGET, 0, a, down-self.knot_at)

    def at(self, t):
        if t >= self.duration:
            return 0., 0., 0.
        if t >= self.down:
            return self.ascent.at(t-self.down)
        if self.knot is not None:
            return self.first.at(t) if t < self.knot_at else self.second.at(t-self.knot_at)
        return self.descent.at(t)


class CosineDip:
    """Sin^4 burst: zero endpoint v/a, finite acceleration at its single bottom."""
    def __init__(self, duration=.48):
        self.duration = duration

    def at(self, t):
        if t >= self.duration:
            return 0., 0., 0.
        z = math.pi*max(0., t)/self.duration
        s, c = math.sin(z), math.cos(z)
        w = math.pi/self.duration
        return TARGET*s**4, 4*TARGET*w*s**3*c, 4*TARGET*w*w*(3*s*s*c*c-s**4)


class Catch:
    """Velocity-shaped catch and continuous-acceleration return. No contact sensor."""
    def __init__(self, depth, speed, brake=40., return_time=.20, join_shape=4.):
        self.speed = max(0., speed)
        self.catch_time = max(.004, 1.5*self.speed/brake)
        t = self.catch_time
        base = depth+.5*self.speed*t
        self.join = min(1.5*self.speed/t, join_shape*max(0., base)/return_time**2)
        self.low = base+self.join*t*t/12
        self.depth = depth
        self.duration = t+return_time
        self.ascent = Quintic(self.low, 0, -self.join, 0, 0, 0, return_time)

    @classmethod
    def to_target(cls,depth,speed,brake,return_time,join_shape=4.):
        """Fit the sampled catch to ten degrees, not the threshold overshoot.

        If already past the target, retain an honest recovery trajectory; scoring
        will reject its measured depth rather than pretending it reached ten.
        """
        nominal=cls(depth,speed,brake,return_time,join_shape)
        if depth>=TARGET or nominal.low<TARGET:return nominal
        lo,hi=brake,max(brake,1.5*max(0.,speed)/.004)
        for _ in range(24):
            mid=(lo+hi)/2
            if cls(depth,speed,mid,return_time,join_shape).low>TARGET:lo=mid
            else:hi=mid
        return cls(depth,speed,hi,return_time,join_shape)

    def at(self, t):
        if t >= self.duration:
            return 0., 0., 0.
        if t >= self.catch_time:
            return self.ascent.at(t-self.catch_time)
        T = self.catch_time
        u = max(0., t/T)
        v0, a0 = self.speed, self.join
        return (self.depth+v0*T*(u-u**3+.5*u**4)-a0*T*T*(.25*u**4-u**3/3),
                v0*(1-3*u*u+2*u**3)-a0*T*(u**3-u*u),
                v0/T*(-6*u+6*u*u)-a0*(3*u*u-2*u))
