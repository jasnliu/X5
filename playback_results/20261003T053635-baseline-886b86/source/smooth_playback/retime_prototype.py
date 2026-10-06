"""Preserved offline path-dependent acceleration-limited timing experiment."""
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.interpolate import PchipInterpolator,make_interp_spline,PPoly
from scipy.ndimage import gaussian_filter1d
from scipy.optimize import linprog
from scipy.sparse import lil_matrix
from .trajectory import build,extrema


def fit(base,amax=1.5,vmax=.70,smoothing=5,edge=.04):
    s=np.linspace(0,base.source_duration,10001)
    v=np.array([p.derivative()(s) for p in base.curves]).T
    arc=cumulative_trapezoid(np.linalg.norm(v,axis=1),s,initial=0)
    keep=np.r_[True,np.diff(arc)>1e-12]
    inverse=PchipInterpolator(arc[keep]/arc[-1],s[keep])
    n=401;u=np.linspace(0,1,n);du=u[1]-u[0];source=inverse(u)
    qs=np.array([p(source) for p in base.curves]).T
    shape=make_interp_spline(u,qs,k=5)
    mid=(u[:-1]+u[1:])/2
    q1=shape(mid,1);q2=shape(mid,2)
    A=lil_matrix((14*(n-1),n));b=np.full(14*(n-1),amax)
    row=0
    for i in range(n-1):
      for j in range(7):
        aa=q2[i,j]/2-q1[i,j]/(2*du);bb=q2[i,j]/2+q1[i,j]/(2*du)
        A[row,i]=aa;A[row,i+1]=bb;A[row+1,i]=-aa;A[row+1,i+1]=-bb;row+=2
    speeds=np.abs(shape(u,1))
    speed_bounds=np.min((np.asarray(vmax)/np.maximum(speeds,1e-6))**2,axis=1)
    bounds=[(0.,float(v)) for v in speed_bounds];bounds[0]=bounds[-1]=(0.,0.)
    solution=linprog(-np.ones(n),A_ub=A.tocsr(),b_ub=b,bounds=bounds,method='highs')
    if not solution.success:raise ValueError(solution.message)
    x=solution.x
    if smoothing:x=gaussian_filter1d(x,smoothing,mode='nearest')
    x=np.maximum(0,x);x[0]=x[-1]=0
    if edge:
        z=np.minimum(np.minimum(u,1-u)/edge,1.)
        x*=np.cbrt(z)*(4.-z)/3.
    # dt = 2 du / (udot_i + udot_i+1) for constant acceleration.
    dt=2*du/(np.sqrt(x[:-1])+np.sqrt(x[1:]))
    times=np.r_[0.,np.cumsum(dt)]
    curves=[PPoly.from_spline(make_interp_spline(times,qs[:,j],k=5,bc_type=([(1,0.),(2,0.)],[(1,0.),(2,0.)]))) for j in range(7)]
    return curves,times,source

if __name__=='__main__':
    _,ms=build();base=ms['smooth']
    for a in [1.,1.5,2.]:
      for sm in [2,5,10]:
        curves,times,source=fit(base,a,smoothing=sm)
        peaks=[max(extrema(p,d) for p in curves) for d in (1,2,3)]
        print(a,sm,'duration',times[-1],'peaks',peaks,flush=True)
