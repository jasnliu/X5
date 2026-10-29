"""Trial vectorized form of core.density_peaks; identical polynomial equations."""
import numpy as np

def local(pp,bounds):
    lo,hi=bounds[:-1],bounds[1:]
    idx=np.clip(np.searchsorted(pp.x,(lo+hi)/2,side='right')-1,0,len(pp.x)-2)
    c=pp.c[:,idx]
    if c.ndim==2:c=c[...,None]
    a=(lo-pp.x[idx])[:,None,None];b=(hi-lo)[:,None,None]
    out=c[0][...,None]
    for coefficient in c[1:]:
        z=np.zeros((*out.shape[:-1],out.shape[-1]+1))
        z[...,:-1]+=out*a;z[...,1:]+=out*b;z[...,0]+=coefficient
        out=z
    return out

def mul(a,b):
    shape=np.broadcast_shapes(a.shape[:-1],b.shape[:-1]);out=np.zeros((*shape,a.shape[-1]+b.shape[-1]-1))
    for i in range(a.shape[-1]):out[...,i:i+b.shape[-1]]+=a[...,i,None]*b
    return out

def add(a,b):
    n=max(a.shape[-1],b.shape[-1]);out=np.zeros((*np.broadcast_shapes(a.shape[:-1],b.shape[:-1]),n))
    out[...,:a.shape[-1]]+=a;out[...,:b.shape[-1]]+=b;return out

def deriv(a):return a[...,1:]*np.arange(1,a.shape[-1]) if a.shape[-1]>1 else np.zeros_like(a)

def at(a,x):
    out=np.zeros(np.broadcast_shapes(a.shape[:-1],x.shape[:-1])+(x.shape[-1],))
    for coefficient in np.moveaxis(a,-1,0)[::-1]:out=out*x+coefficient[...,None]
    return out

def roots(a):
    if not np.isfinite(a).all():raise ValueError('Nonfinite stationary polynomial')
    n=a.shape[-1]-1;flat=a.reshape(-1,n+1)
    degree=np.max(np.where(abs(flat)>1e-15,np.arange(n+1),0),axis=1)
    result=np.full((len(flat),n+2),np.nan);result[:,0]=0.;result[:,1]=1.
    for d in np.unique(degree):
        if d==0:continue
        ids=np.flatnonzero(degree==d);pol=flat[ids,:d+1]
        companion=np.zeros((len(ids),d,d))
        if d>1:companion[:,np.arange(1,d),np.arange(d-1)]=1.
        companion[:,:,-1]=-pol[:,:-1]/pol[:,-1,None]
        r=np.linalg.eigvals(companion)
        good=(abs(r.imag)<1e-7)&(r.real>0)&(r.real<1)
        result[ids,2:d+2]=np.where(good,r.real,np.nan)
    return result.reshape(*a.shape[:-1],n+2)

def peaks(path,h):
    bounds=np.unique(np.r_[path.x,h.x]);bounds=bounds[(bounds>=0)&(bounds<=1)]
    H,H1,H2=[local(h.derivative(d),bounds) for d in range(3)]
    q1,q2,q3=[local(path.derivative(d),bounds) for d in (1,2,3)]
    if np.nanmin(at(H,roots(deriv(H))))<=0:raise ValueError('Nonmonotone density')
    nums=[q1,add(mul(q2,H),-mul(q1,H1)),
          add(add(mul(mul(q3,H),H),-3*mul(mul(q2,H),H1)),
              add(-mul(mul(q1,H),H2),3*mul(mul(q1,H1),H1)))]
    out=np.zeros((3,7));where=out.copy()
    for k,(N,power) in enumerate(zip(nums,(1,3,5))):
        r=roots(add(mul(deriv(N),H),-power*mul(N,deriv(H))))
        vals=abs(at(N,r)/at(H,r)**power)
        for j in range(7):
            i,a=np.unravel_index(np.nanargmax(vals[:,j]),vals[:,j].shape)
            out[k,j]=vals[i,j,a];where[k,j]=bounds[i]+np.diff(bounds)[i]*r[i,j,a]
    return out,where
