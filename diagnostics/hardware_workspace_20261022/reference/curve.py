"""Device-free exact playback of current retimed laws and legacy saved plans.

Never refit sampled positions: hardware, preview, and prefix recovery evaluate
the same coefficients and monotone clock. Legacy recovery remains supported.
"""
import numpy as np
from ..execution import Curve as LegacyCurve
from ..timing import core


class Curve(LegacyCurve):
    def __init__(self,plan):
        self.retimed=None;self._peaks=None
        timing=plan['timing']
        if timing.get('schema')!='x5-fixed-path-timing-v1':
            super().__init__(plan);return
        self.duration=float(timing['duration_s'])
        if not np.isfinite(self.duration) or not 0<self.duration<=120:
            raise ValueError('Invalid exact retimed trajectory duration')
        self.retimed=core.Evaluator(timing)
        # Validate the actual representation, not its cached "peaks" metadata.
        if timing['kind']=='time_density':
            p,h=self.retimed.path,self.retimed.h
            if abs(float(self.retimed.H(1)-self.retimed.H(0))-self.duration)>1e-9:
                raise ValueError('Timing integral/duration mismatch')
            self._validate_polynomial(p,0.,1.,7)
            self._validate_polynomial(h,0.,1.,None)
            _,jumps=core.polynomial_peaks(p)
            if np.any(jumps>[1e-9,1e-8,1e-7]):raise ValueError('Discontinuous retimed path')
            for knot in np.unique(h.x)[1:-1]:
                if 0<knot<1:
                    for k in range(3):
                        if abs(h(knot,nu=k)-h(np.nextafter(knot,-np.inf),nu=k))>1e-7:
                            raise ValueError('Discontinuous time density')
            self._peaks=core.density_peaks(p,h)[0]
        else:
            self._validate_polynomial(self.retimed.p,0.,self.duration,7)
            self._peaks,jumps=core.polynomial_peaks(self.retimed.p)
            if np.any(jumps>[1e-9,1e-8,1e-7]):raise ValueError('Discontinuous timed polynomial')
        if not np.isfinite(self._peaks).all():raise ValueError('Nonfinite motion extrema')
        times=np.array([p['time_s'] for p in plan['outward']],float)
        q=np.asarray([p['positions_rad'] for p in plan['outward']],float)
        if (len(times)<2 or not np.isfinite(times).all() or times[0]!=0 or
                abs(times[-1]-self.duration)>1e-9 or np.any(np.diff(times)<=0)):
            raise ValueError('Invalid retimed outward timestamps')
        if q.shape!=(len(times),7) or not np.allclose(self.at_many(times),q,atol=1e-9,rtol=0):
            raise ValueError('Retimed law differs from collision-checked samples')

    @staticmethod
    def _validate_polynomial(p,start,end,dimension):
        if (not np.isfinite(p.c).all() or not np.isfinite(p.x).all() or np.any(np.diff(p.x)<0)
                or p.x[0]>start or p.x[-1]<end
                or (dimension is not None and (p.c.ndim!=3 or p.c.shape[-1]!=dimension))
                or (dimension is None and p.c.ndim!=2)):
            raise ValueError('Invalid retimed polynomial representation')

    def at_many(self,times):
        times=np.asarray(times,float)
        if not np.isfinite(times).all() or np.any(times<0) or np.any(times>self.duration):
            raise ValueError('Trajectory time out of range')
        if self.retimed is None:return np.array([super(Curve,self).at(float(t)) for t in times])
        q=self.retimed(times)[0]
        if not np.isfinite(q).all():raise ValueError('Invalid trajectory target')
        return q

    def at(self,t):
        if self.retimed is None:return super().at(t)
        return self.at_many(t)
