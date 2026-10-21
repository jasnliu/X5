"""Small measured-error correction and lenient stall test."""
from collections import deque
import math
import numpy as np

TOLERANCE = math.radians(3)
HOLD_SECONDS = 2.0
MAX_CORRECTION = math.radians(12)
STALL_ERROR = math.radians(8)
STALL_MOVEMENT = math.radians(1)
STALL_SECONDS = 2.5
CENTER = np.zeros(7)
GOAL = np.array([math.radians(45),0.,0.,math.radians(80),0.,0.,math.radians(-80)])

class GoalControl:
    def __init__(self, desired, lower, upper, now):
        self.desired=np.asarray(desired,dtype=float)
        self.lower=np.asarray(lower,dtype=float);self.upper=np.asarray(upper,dtype=float)
        self.history=deque();self.in_tolerance_since=None;self.started=now
        if self.desired.shape!=(7,) or np.any(self.desired<self.lower) or np.any(self.desired>self.upper):
            raise ValueError('Invalid joint goal')

    def update(self, actual, now):
        actual=np.asarray(actual,dtype=float)
        if actual.shape!=(7,) or not np.isfinite(actual).all():raise RuntimeError('Invalid encoder feedback')
        error=self.desired-actual
        # Simple correction: ask slightly farther in the direction of residual error.
        correction=np.clip(.5*error,-MAX_CORRECTION,MAX_CORRECTION)
        correction[np.abs(error)<=TOLERANCE]=0.
        command=np.clip(self.desired+correction,self.lower,self.upper)
        self.history.append((now,actual.copy()))
        while self.history and now-self.history[0][0]>STALL_SECONDS:self.history.popleft()
        if self.history and now-self.history[0][0]>=STALL_SECONDS-.05:
            span=np.ptp(np.array([q for _,q in self.history]),axis=0)
            stalled=(np.abs(error)>STALL_ERROR)&(span<STALL_MOVEMENT)
            if np.any(stalled):raise RuntimeError('STALL: motor '+str(int(np.flatnonzero(stalled)[0])+1))
        if np.max(np.abs(error))<=TOLERANCE:
            if self.in_tolerance_since is None:self.in_tolerance_since=now
        else:self.in_tolerance_since=None
        reached=self.in_tolerance_since is not None and now-self.in_tolerance_since>=.6
        if now-self.started>30:raise RuntimeError('Position timeout')
        return command,error,reached
