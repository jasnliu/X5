"""Preserved timing-constrained arc-length experiment (offline by default)."""
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.interpolate import PchipInterpolator,make_interp_spline,PPoly
from .trajectory import build,extrema


def fit(base,T=4.6,ramp=.6,knots=65):
    s=np.linspace(0,base.source_duration,10001)
    q=np.array([p(s) for p in base.curves]).T
    v=np.array([p.derivative()(s) for p in base.curves]).T
    length=cumulative_trapezoid(np.linalg.norm(v,axis=1),s,initial=0)
    u=length/length[-1]
    keep=np.r_[True,np.diff(u)>1e-12]
    inverse=PchipInterpolator(u[keep],s[keep])
    times=np.linspace(0,T,10001)
    # Raised-cosine velocity ramps: zero velocity AND acceleration at rest.
    speed=np.ones_like(times)/(T-ramp)
    mask=times<ramp
    speed[mask]*=.5*(1-np.cos(np.pi*times[mask]/ramp))
    mask=times>T-ramp
    speed[mask]*=.5*(1-np.cos(np.pi*(T-times[mask])/ramp))
    progress=cumulative_trapezoid(speed,times,initial=0);progress/=progress[-1]
    source=np.asarray(inverse(progress))
    # Uniform geometric knots, plus near-end knots to resolve smooth ramping.
    uk=np.unique(np.r_[np.linspace(0,1,knots),.0001,.0005,.002,.005,.995,.998,.9995,.9999])
    tk=np.interp(uk,progress,times)
    sk=inverse(uk)
    qk=np.array([p(sk) for p in base.curves]).T
    curves=[PPoly.from_spline(make_interp_spline(tk,qk[:,j],k=5,bc_type=([(1,0.),(2,0.)],[(1,0.),(2,0.)]))) for j in range(7)]
    return curves,times,source

if __name__=='__main__':
    g,ms=build();base=ms['smooth']
    for T in [3.84,4.2,4.6,5.0]:
      for ramp in [.4,.6,.8]:
        curves,times,source=fit(base,T,ramp)
        peaks=[max(extrema(p,d) for p in curves) for d in (1,2,3)]
        print(T,ramp,peaks,flush=True)
