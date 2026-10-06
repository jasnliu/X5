"""Identical readiness, command limiting, capture and scoring for all strategies."""
from collections import deque
from dataclasses import replace
import math
import time
from camera_playback.mit_strike import Command
from .config import TARGET, TARGET_DEG, Rules, Limits
from .methods import METHODS
from .scoring import score


class StableWindow:
    def __init__(self, anchor, rules):
        self.anchor,self.rules=anchor,rules;self.samples=deque()

    def add(self,s):
        h=self.samples;r=self.rules
        if h and s.at<=h[-1].at:return
        if h and s.at-h[-1].at>r.settle_seconds:h.clear()
        h.append(s)
        while len(h)>2 and h[1].at<s.at-r.settle_seconds:h.popleft()

    @property
    def ready(self):
        h=self.samples;r=self.rules
        return (len(h)>=3 and h[-1].at-h[0].at>=r.settle_seconds-1e-9
                and max(abs(s.position-self.anchor) for s in h)<=math.radians(r.return_tolerance_deg)
                and max(s.position for s in h)-min(s.position for s in h)<=math.radians(r.settle_range_deg))


def validate_curve(method,limits):
    """Reject invalid reference shapes BEFORE physical execution, not after it."""
    curve=getattr(method,'curve',None)
    if curve is None:return
    last=None;turns=0;sign=1
    for i in range(1001):
        t=curve.duration*i/1000;x,v,a=curve.at(t)
        if not all(math.isfinite(z) for z in (x,v,a)) or not -1e-9<=x<=TARGET+1e-9:
            raise ValueError('Reference leaves the fixed 0-to-10-degree corridor')
        if abs(v)>limits.velocity or abs(a)>limits.acceleration:
            raise ValueError('Reference exceeds common speed/acceleration limits')
        if last and abs((a-last[1])/(t-last[0]))>limits.jerk:
            raise ValueError('Reference exceeds common jerk limit')
        if abs(v)>1e-6:
            new=1 if v>0 else -1
            if new!=sign:turns+=1;sign=new
        last=t,a
    if turns!=1:raise ValueError('Reference must have one down/up reversal')


class Engine:
    def __init__(self,backend,store,stop,publish=lambda _:None,rules=Rules(),limits=Limits()):
        self.backend,self.store,self.stop,self.publish=backend,store,stop,publish
        self.rules,self.limits=rules,limits
        self.preparing=True
        self.anchor,self.bias=backend.prepare(stop,publish)
        self.window=StableWindow(self.anchor,rules)
        self.last_send=None;self.last_torque=self.bias;self.last_publish=-1e9
        self.current_method=None
        self.validated_references=set()
        self.wait_ready()
        self.preparing=False

    def check(self,s,now,moving):
        if self.stop.is_set():raise InterruptedError('Stop requested')
        if s is None or not all(math.isfinite(v) for v in (s.position,s.velocity,s.torque,s.at)):
            raise RuntimeError('Invalid/missing encoder feedback')
        if s.state!=2:raise RuntimeError('J7 not running')
        if not -.001<=now-s.at<=(self.limits.feedback_timeout if moving else .25):
            raise RuntimeError('J7 feedback deadline missed')
        if moving and self.last_send is not None and now-self.last_send>self.limits.control_timeout:
            raise RuntimeError('J7 control deadline missed')
        depth=math.degrees(self.anchor-s.position)
        if not -self.limits.upper_excursion_deg<=depth<=self.limits.hard_depth_deg:
            raise RuntimeError(f'J7 outside experiment corridor ({depth:.3f} degrees)')
        speed_limit=self.limits.velocity*(2. if self.preparing else 1.25)
        if abs(s.velocity)>speed_limit:
            raise RuntimeError(f'J7 excessive measured velocity ({s.velocity:.3f} rad/s, depth={depth:.3f} deg, age={(now-s.at)*1000:.2f} ms)')
        self.store.check()

    def transmit(self,c,s,now):
        l=self.limits
        vals=(c.position,c.velocity,c.kp,c.kd,c.torque)
        if not all(math.isfinite(v) for v in vals):raise RuntimeError('Nonfinite command')
        if not 0<=c.kp<=500 or not 0<=c.kd<=5 or abs(c.velocity)>l.velocity:
            raise RuntimeError('Command exceeds shared bounds')
        if not self.anchor-math.radians(l.hard_depth_deg)<=c.position<=self.anchor+1e-8:
            # KP=0 position is irrelevant; encode an in-corridor reference anyway.
            if c.kp==0:c=replace(c,position=min(self.anchor,max(self.anchor-math.radians(l.hard_depth_deg),c.position)))
            else:raise RuntimeError('Commanded position outside experiment corridor')
        estimated=c.kp*(c.position-s.position)+c.kd*(c.velocity-s.velocity)+c.torque
        dt=1/l.hz if self.last_send is None else max(1e-6,now-self.last_send)
        bounded=max(-l.torque,min(l.torque,estimated))
        bounded=max(self.last_torque-l.torque_slew*dt,min(self.last_torque+l.torque_slew*dt,bounded))
        limited=abs(estimated-bounded)>1e-8
        c=replace(c,torque=c.torque+bounded-estimated)
        if abs(c.torque)>14:raise RuntimeError('Required torque compensation exceeds drive encoding')
        self.backend.send(c)
        self.last_send=self.backend.now();self.last_torque=bounded
        return c,bounded,limited

    def emit(self,phase,s,**extra):
        now=self.backend.now()
        if now-self.last_publish>=.04 or extra:
            self.publish(dict(phase=phase,positions=list(self.backend.positions),anchor=self.anchor,
                              depth_deg=math.degrees(self.anchor-s.position),velocity=s.velocity,
                              backend=self.backend.name,**extra))
            self.last_publish=now

    def idle(self,period=.01):
        s=self.backend.receive();now=self.backend.now();self.check(s,now,False)
        self.window.add(s)
        self.transmit(self.hold(s,period),s,now)
        self.emit('READY' if self.window.ready else 'SETTLING',s)
        self.backend.advance(period,self.stop)

    def hold(self,s,dt):
        # Correct static-friction/load offsets at rest, including the return.
        # The bias is frozen during the actual down/up stroke for repeatability.
        if abs(s.velocity)<.2 and abs(self.anchor-s.position)<.05:
            self.bias=max(-2.,min(2.,self.bias+60*(self.anchor-s.position)*min(.02,dt)))
        return Command(self.anchor,0,40,1.8,self.bias)

    def wait_ready(self):
        start=self.backend.now()
        while not self.window.ready:
            if self.backend.now()-start>3:
                h=self.window.samples
                span=math.degrees(max(s.position for s in h)-min(s.position for s in h)) if h else float('nan')
                error=math.degrees(self.anchor-h[-1].position) if h else float('nan')
                raise RuntimeError(f'Anchor did not settle (error={error:.3f} deg, window range={span:.3f} deg, bias={self.bias:.3f} Nm)')
            self.idle()

    def trial(self,method_id,overrides=None,stage='manual',parent=None,schedule=None):
        definition=METHODS[method_id];p=definition.parameters(overrides)
        self.wait_ready()
        trial=self.store.begin(method_id,p,stage,self.anchor,self.bias,parent,schedule)
        rows=[];chunk=[];error=None;major=None;started=False
        try:
            method=definition.create(self.anchor,self.bias,p)
            reference_key=(method_id,tuple(sorted(p.items())))
            if reference_key not in self.validated_references:
                validate_curve(method,self.limits)
                self.validated_references.add(reference_key)
            self.current_method=method
            # Archive work is outside active timing. Refresh hold/feedback first.
            self.idle(1/self.limits.hz);self.wait_ready()
            # USB CAN can deliver the first hold reply late. Do not release a
            # stroke against an old sample just because it is inside the abort
            # timeout; actively prime until a recent hold reply is available.
            prime_at=self.backend.now()
            while self.backend.now()-self.backend.receive().at>min(.006,self.rules.maximum_feedback_gap/2):
                if self.backend.now()-prime_at>.25:raise RuntimeError('Fresh release feedback unavailable')
                self.idle(1/self.limits.hz)
            s=self.backend.receive();now=self.backend.now();self.check(s,now,True)
            if now-s.at>self.limits.feedback_timeout:raise RuntimeError('Strike feedback not fresh')
            method.start(now,list(self.window.samples))
            started=True
            start=now;self.window=StableWindow(self.anchor,self.rules)
            returned=False
            while True:
                s=self.backend.receive();now=self.backend.now();self.check(s,now,True)
                if now-start>self.limits.trial_timeout:raise RuntimeError('Trial timeout')
                c=method.update(s,now)
                if method.done:
                    if not returned:self.window=StableWindow(self.anchor,self.rules);returned=True
                    self.window.add(s)
                    c=self.hold(s,1/self.limits.hz)
                # Recheck after method computation; never send an obsolete tick.
                send_at=self.backend.now();self.check(s,send_at,True)
                c,tau,limited=self.transmit(c,s,send_at)
                row=dict(time=now-start,sample_at=s.at-start,depth_deg=math.degrees(self.anchor-s.position),
                         position=s.position,velocity=s.velocity,feedback_torque=s.torque,
                         command_position=c.position,command_velocity=c.velocity,kp=c.kp,kd=c.kd,
                         feedforward_torque=c.torque,estimated_torque=tau,limited=limited,phase=method.phase,
                         temperature_c=getattr(self.backend,'temperature',None))
                row.update({f'held_j{i+1}':self.backend.positions[i] for i in range(6)})
                rows.append(row);chunk.append(row)
                if len(chunk)>=50:self.store.rows(trial,chunk);chunk=[]
                self.emit(method.phase,s,method=method_id,trial=trial) if now-self.last_publish>=.04 else None
                if returned and self.window.ready:break
                # Schedule from successful send; no catch-up bursts after delays.
                self.backend.advance(max(0.,1/self.limits.hz-(self.backend.now()-send_at)),self.stop)
        except ValueError as exc:
            error='reference rejected: '+str(exc)
            if started:
                major=exc
                self.backend.relax()
        except (Exception,KeyboardInterrupt) as exc:
            error=str(exc) or type(exc).__name__;major=exc
            # Stop actuation BEFORE scoring or filesystem work on a fault.
            try:self.backend.relax()
            except Exception as relax_error:error+='; relaxation: '+str(relax_error)
        finally:
            self.store.rows(trial,chunk)
            result=score(rows,self.rules,self.limits,error)
            if started:
                result['release_at']=start
                if schedule:
                    late=max(0.,start-schedule['requested_start'])
                    result['release_lateness_ms']=late*1000
                    if late>self.rules.maximum_start_lateness:
                        result['passed']=False;result['reasons'].append('missed_start_interval')
            self.store.finish(trial,result)
            self.current_method=None
        self.publish(dict(phase='TRIAL COMPLETE',trial=trial,method=method_id,score=result,
                          positions=list(self.backend.positions)))
        if major:raise major
        return dict(id=trial,metadata=self.store.trials[-1]['metadata'],score=result)
