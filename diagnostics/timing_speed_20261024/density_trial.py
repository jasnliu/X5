"""Offline bounded timing candidates. Never imported by a hardware program."""
import time
import numpy as np
from scipy.interpolate import BSpline,PPoly
from scipy.sparse import csc_matrix
from drum_planner.timing import core
from drum_planner.timing.vulkan.fast import cpu_solve
from drum_planner.timing.vulkan.device import Device


def basis_for(n,continuity=2,degree=3,extra=(),warp=False):
    multiplicity=degree-continuity
    intervals=max(1,(n-degree-1)//multiplicity+1)
    inside=np.unique(np.r_[np.linspace(0,1,intervals+1)[1:-1],extra]);inside=inside[(inside>0)&(inside<1)]
    if warp:inside=(1-np.cos(np.pi*inside))/2
    knots=np.r_[np.zeros(degree+1),np.repeat(inside,multiplicity),np.ones(degree+1)]
    n=len(knots)-degree-1
    return knots,BSpline(knots,np.eye(n),degree,extrapolate=False)


def ipopt(B,q,w,x,*,reduced=True,maxiter=150,tol=1e-7,screen=False,polynomial=False):
    import casadi as ca
    n=len(x);z=ca.MX.sym('clock',n)
    H,H1,H2=[b@z for b in map(lambda a:ca.DM(csc_matrix(a)),B)]
    q1,q2,q3=[ca.DM(a) for a in q]
    h,h1,h2=[ca.repmat(v,1,7) for v in (H,H1,H2)]
    acc=q2/h**2-q1*h1/h**3
    jerk=q3/h**3-3*q2*h1/h**4-q1*h2/h**4+3*q1*h1**2/h**5
    vals=[acc/core.LIMITS[1],jerk/core.LIMITS[2]]
    if screen:
        hv,hv1,hv2=[(b@x)[:,None] for b in B];p1,p2,p3=q
        estimates=[abs(p2/hv**2-p1*hv1/hv**3),abs(p3/hv**3-3*p2*hv1/hv**4-p1*hv2/hv**4+3*p1*hv1**2/hv**5)]
        vals=[ca.vec(v)[np.flatnonzero((e>=np.max(e,axis=1)[:,None]*.75).ravel(order='F'))] for v,e in zip(vals,estimates)]
    if polynomial:
        ref=ca.DM(np.maximum(B[0]@x,.1)[:,None]);ref=ca.repmat(ref,1,7)
        av=q2*h-q1*h1;jv=q3*h*h-3*q2*h*h1-q1*h*h2+3*q1*h1*h1
        vals=[(av/core.LIMITS[1]-h**3)/ref**3,(-av/core.LIMITS[1]-h**3)/ref**3,
              (jv/core.LIMITS[2]-h**5)/ref**5,(-jv/core.LIMITS[2]-h**5)/ref**5]
    g=ca.vertcat(H-np.max(abs(q[0]),axis=1)/core.LIMITS[0],*[ca.vec(v) for v in vals])
    m=len(q[0]);nother=sum(int(v.numel()) for v in vals);lbg=np.r_[np.zeros(m),-np.ones(nother)];ubg=np.r_[np.full(m,np.inf),np.ones(nother)]
    if polynomial:lbg[m:]=-np.inf;ubg[m:]=0.
    opts={'print_time':False,'ipopt.print_level':0,'ipopt.sb':'yes','ipopt.tol':tol,'ipopt.max_iter':maxiter,
          'ipopt.acceptable_iter':0,'ipopt.mu_strategy':'adaptive','ipopt.bound_relax_factor':0.,'ipopt.linear_solver':'mumps'}
    solver=ca.nlpsol('trial','ipopt',{'x':z,'f':ca.dot(w,z),'g':g},opts)
    r=solver(x0=x,lbx=.005,ubx=5,lbg=lbg,ubg=ubg)
    return np.asarray(r['x']).ravel(),dict(iterations=solver.stats()['iter_count'],status=solver.stats()['return_status'])


def solve(source,method='al_cpu',n=48,continuity=2,degree=3,refine=1,gridfactor=4,warm=False,pathknots=False,screen=False,polynomial=False,lobatto=False,warp=False,seed_method="al"):
    began=time.monotonic();p=core.original(source);base=source['timing']['time_scale']/core.RESERVE
    knots,basis=basis_for(n,continuity,degree,warp=warp,extra=np.unique(p.x) if pathknots else ());n=basis.c.shape[0]
    cuts=np.unique(knots)
    grid=np.unique(np.r_[np.linspace(0,1,max(129,n*gridfactor+1)),p.x,cuts,np.nextafter(cuts[1:-1],-np.inf)])
    if lobatto:grid=np.unique(np.r_[(cuts[:-1,None]+np.diff(cuts)[:,None]*np.linspace(0,1,gridfactor+1)).ravel(),p.x,np.nextafter(p.x[1:-1],-np.inf)])
    grid=grid[(grid>=0)&(grid<=1)];x=np.ones(n);history=[]
    if warm:
        from drum_planner.timing.vulkan.optimizer import arrays
        k,b,_,bvals,qp,w=arrays(p,base,24)
        v,_=ipopt(bvals,qp,w,np.ones(24),maxiter=90) if seed_method=="ipopt" else cpu_solve(bvals,qp,w,np.ones(24))
        xx=np.linspace(0,1,max(201,n*4));x=np.clip(np.linalg.lstsq(basis(xx),b(xx)@v,rcond=None)[0],.005,5.)
    weights=basis.antiderivative()(1)-basis.antiderivative()(0)
    for r in range(refine):
        B=np.array([basis(grid,nu=d)*base for d in range(3)]);q=np.array([p(grid,nu=d) for d in (1,2,3)])
        start=time.monotonic()
        if method=='al_cpu':x,it=cpu_solve(B,q,weights,x);info=dict(iterations=it)
        elif method=='ipopt':x,info=ipopt(B,q,weights,x,screen=screen,polynomial=polynomial)
        elif method=='al_gpu':
            if continuity!=2 or degree!=3:raise ValueError('Existing shader expects standard cubic')
            with Device(B,q,weights,x) as gpu:
                rho=1;it=0
                for outer in range(7):
                    for block in range(64 if outer==0 else 32):
                        x,state=gpu.step();it+=32
                        if state[7]>=64 and state[4]<1e-6:break
                    if outer>=3 and state[8]<1e-5:break
                    rho*=5;gpu.dual(rho)
                info=dict(iterations=it,device=gpu.name)
        else:raise ValueError(method)
        compute=time.monotonic()-start;h=PPoly.from_spline(BSpline(knots,x*base,degree));peaks,where=core.density_peaks(p,h)
        scale=max(1.,core.scale_needed(peaks))*core.RESERVE
        history.append(dict(solve_s=compute,scale=scale,grid=len(grid),**info))
        if scale<1.00105:break
        grid=np.unique(np.r_[grid,where.ravel(),np.nextafter(where.ravel(),-np.inf)])
        grid=grid[(grid>=0)&(grid<=1)]
    h.c*=scale;peaks/=scale**np.arange(1,4)[:,None]
    duration=float(h.integrate(0,1))
    return dict(schema='x5-fixed-path-timing-v1',method=method,kind='time_density',duration_s=duration,
        path=core.encode(p),density=core.encode(h),source_path_sha256=core.identity(source),limits=core.LIMITS.tolist(),
        peaks=peaks.tolist(),extrema_s=where.tolist(),geometry_preserved=True,
        details=dict(n=n,continuity=continuity,degree=degree,history=history,elapsed_s=time.monotonic()-began))
