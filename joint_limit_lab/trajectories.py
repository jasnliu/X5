"""Analytic C3 probes. No differentiation of noisy positions to set limits."""
import numpy as np
from numpy.polynomial import Polynomial
from scipy.interpolate import PPoly

from drum_planner.timing import core
from .schema import array, identity

BLEND=Polynomial([0,0,0,0,35,-84,70,-20])


def unit_peak(order):
    p=BLEND.deriv(order);roots=p.deriv().roots()
    r=[x.real for x in roots if abs(x.imag)<1e-8 and 0<x.real<1]
    return float(np.max(abs(p(np.r_[0.,r,1.]))))


PEAKS=np.array([unit_peak(k) for k in (1,2,3)])


def probe(anchor,joint,direction,displacement,limits,settle=.4):
    q=array(anchor,(7,),'anchor');caps=array(limits,(3,7),'limits',positive=True)
    if type(joint) is not int or not 1<=joint<=7 or direction not in (-1,1):raise ValueError('Invalid probe joint/direction')
    if not np.isfinite(displacement) or not 0<displacement<=1.:raise ValueError('Invalid displacement')
    duration=float(np.max((PEAKS*displacement/caps[:,joint-1])**(1/np.arange(1,4))))*1.001
    if not np.isfinite(duration) or duration<=0 or 2*(duration+settle)>60:
        raise ValueError('Probe exceeds the bounded 60-second trace window or has an invalid duration')
    delta=np.zeros(7);delta[joint-1]=direction*displacement
    intervals=[]
    for start,change,T in ((q,delta,duration),(q+delta,np.zeros(7),settle),
                           (q+delta,-delta,duration),(q,np.zeros(7),settle)):
        c=np.zeros((8,7));c[-1]=start
        for power,k in enumerate(BLEND.coef):c[7-power]+=change*k/T**power
        intervals.append(c)
    pp=PPoly(np.stack(intervals,axis=1),[0,duration,duration+settle,2*duration+settle,2*(duration+settle)],extrapolate=False)
    peaks,jumps=core.polynomial_peaks(pp)
    if np.any(peaks>caps*(1+1e-8)) or np.any(jumps>[1e-8,1e-7,1e-6]):raise ValueError('Invalid probe derivatives')
    return dict(kind='joint_polynomial',schema='x5-fixed-path-timing-v1',curve=core.encode(pp),
        duration_s=float(pp.x[-1]),peaks=peaks.tolist(),limits=caps.tolist(),
        outward_s=duration,hold_s=settle,joint=joint,direction=direction,
        displacement_rad=displacement,anchor=q.tolist(),analytic=True)


def evaluate(law,t):
    return core.evaluate(law,t)


def verify(law):
    p=core.decode(law['curve']);peaks,jumps=core.polynomial_peaks(p)
    if p.c.shape[-1]!=7 or not np.isfinite(p.c).all() or np.any(np.diff(p.x)<=0):raise ValueError('Invalid probe polynomial')
    if np.any(peaks>np.array(law['limits'])*(1+1e-8)):raise ValueError('Probe exceeds its declared limits')
    np.testing.assert_allclose(peaks,law['peaks'],atol=1e-8,rtol=1e-8)
    if np.any(jumps>[1e-8,1e-7,1e-6]):raise ValueError('Probe is discontinuous')
    if not np.allclose(p(0),p(p.x[-1]),atol=1e-10):raise ValueError('Probe does not return to anchor')
    for d in (1,2,3):
        if np.max(abs(p([0,p.x[-1]],nu=d)))>1e-6:raise ValueError('Probe is not at rest at endpoints')
    return identity(law)
