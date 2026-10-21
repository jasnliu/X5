"""Tick-driven supervised motion state machine; no GUI, sockets or auto-fault relax.

Ordinary faults latch a measured-pose CSP hold if healthy feedback permits it.
No fault automatically starts recovery. A new, validated return requires an
explicit operator command. A drive/CAN/power fault can make torque unavailable.
"""
from collections import deque
import time
import numpy as np
from ..execution import Curve,TRACK_L1,LEFT_MAX,REST_TOL
from .paths import speed,validate_timing,recovery_at

HOLD_PHASES={'HELD','FAULT_HOLD','CHECKING_RETURN'}
ACTIVE_PHASES={'ARMING','SETTLING_START','OUTWARD','SETTLING_GOAL','GOAL_HOLD','RETURN','RECOVERY','SETTLING_REST'}
TERMINAL={'RELAXED','REJECTED','SUPPORTED_EXIT'}


class Controller:
    def __init__(self,bus,obj,req,*,rate=1.,now=time.monotonic,event=lambda *_:None):
        self.bus=bus;self.obj=obj;self.req=req;self.curve=Curve(obj);validate_timing(self.curve)
        self.rate=speed(rate);self.now=now;self.sink=event;self.phase='READY';self.error=''
        self.tau=0.;self.anchor=self.curve.at(0);self.q=None;self.held=None;self.hold_confirmed=False
        self.started=False;self.disabled=False;self.arm_steps=None;self.previous=None;self.phase_at=now()
        self.history=deque(maxlen=200);self.events=deque(maxlen=1000);self.generation=0
        self.return_spec=None;self.closed=False;self.recovery_reason='';self.feedback_error=''

    def emit(self,phase,**kw):
        self.phase=phase;self.phase_at=self.now();self.history.clear()
        if phase not in HOLD_PHASES:self.hold_confirmed=False
        item=dict(monotonic=self.phase_at,phase=phase,tau=self.tau,**kw);self.events.append(item)
        try:self.sink(item)
        except Exception as e:
            # Logging is never an authorization to release torque.
            self.error='Event log failed: '+str(e)
            if self.started and phase not in HOLD_PHASES and phase not in TERMINAL:
                self.halt(self.error,fault=True,log=False)
            elif not self.started and phase not in TERMINAL:self.phase='REJECTED'

    def sample(self,mode='enabled',check_scene=True):
        self.bus.poll();now=self.now()
        if not self.bus.fresh() or len(self.bus.states)!=16 or any(now-v[2]>.12 for v in self.bus.states.values()):
            raise RuntimeError('Missing/stale motor feedback; holding torque cannot be confirmed')
        if any(self.bus.states['left',i][1]!=0 for i in range(1,9)):raise RuntimeError('Left arm is not relaxed')
        q=np.array([self.bus.states['right',i][0] for i in range(1,8)])
        if not np.isfinite(q).all():raise RuntimeError('Non-finite feedback')
        self.q=q.copy()
        if mode=='enabled' and any(self.bus.states['right',i][1]!=2 for i in range(1,9)):
            raise RuntimeError('Right drive not running; cannot guarantee hold or reverse')
        if mode=='disabled' and any(v[1]!=0 for v in self.bus.states.values()):raise RuntimeError('Both arms must initially be relaxed')
        from centering.motors import RIGHT_GRIPPER_CLOSED
        if abs(self.bus.states['right',8][0]-RIGHT_GRIPPER_CLOSED)>np.radians(1.):
            raise RuntimeError('Custom gripper is not at its closed position; opening/forcing it is forbidden')
        bounds=np.asarray(self.req['execution_joint_limits'])
        if np.any(q<bounds[:,0]) or np.any(q>bounds[:,1]):raise RuntimeError('Right joint outside model bounds; no clipping')
        if check_scene:
            if getattr(self.bus,'audit_error',''):raise RuntimeError('CAN audit failed: '+self.bus.audit_error)
            left=np.array([self.bus.states['left',i][0] for i in range(1,8)])
            if np.max(abs(left-self.req['left']))>LEFT_MAX:raise RuntimeError('Static left arm moved; return needs original scene restored')
            opening=np.clip(.044*self.bus.states['left',8][0]/1.0472,0,.044)
            if abs(opening-self.req['left_opening_m'])>.001:raise RuntimeError('Static left gripper changed')
        self.feedback_error='';return q

    def start(self):
        if self.phase!='READY':raise ValueError('This session cannot be started again; make a new measured plan')
        self.previous=self.now();self.emit('PREFLIGHT')

    def send(self,target):
        target=np.asarray(target,dtype=float)
        if self.q is None or np.sum(abs(self.q-self.anchor))>TRACK_L1:
            raise RuntimeError('Tracking deviation / possible contact: stopped; no relaxation')
        if np.sum(abs(target-self.anchor))>.025:raise RuntimeError('Target discontinuity rejected')
        self.bus.set_positions(target);self.anchor=target.copy()

    def halt(self,reason='Operator emergency hold',*,fault=False,log=True):
        if self.phase in TERMINAL:return
        self.generation+=1;self.return_spec=None
        if self.arm_steps is not None:
            self.arm_steps.close();self.arm_steps=None
        self.started=self.started or bool(getattr(self.bus,'enable_attempted',False))
        if not self.started:
            self.error=reason;self.emit('REJECTED',error=reason);return
        already_held=self.phase in HOLD_PHASES
        self.error=reason;self.hold_confirmed=False
        # Do not continually chase drift under gravity. Freeze ONCE, rather
        # than moving the hold anchor each tick or after every recurring fault.
        if not already_held or self.held is None:
            try:
                q=self.sample(check_scene=False)
                self.bus.set_positions(q);self.anchor=q.copy();self.held=q.copy()
                self.hold_confirmed=True
            except Exception as e:self.feedback_error=str(e)
        else:
            try:
                q=self.sample(check_scene=False)
                self.hold_confirmed=np.sum(abs(q-self.held))<=TRACK_L1
            except Exception as e:self.feedback_error=str(e)
        phase='FAULT_HOLD' if fault else 'HELD'
        if log:self.emit(phase,error=reason,hold_confirmed=bool(self.hold_confirmed),feedback_error=self.feedback_error)
        else:self.phase=phase;self.phase_at=self.now();self.history.clear()

    def request_return(self,scene_confirmed):
        if self.phase not in {'HELD','FAULT_HOLD'}:raise ValueError('Hold first; return is only available from a held state')
        if scene_confirmed is not True:raise ValueError('Confirm obstruction removed, return corridor clear, and original support unchanged')
        q=self.sample()
        if self.held is None:
            # A transient CAN failure may have prevented the initial hold
            # request. Relatch only on a new explicit operator action.
            self.bus.set_positions(q);self.anchor=q.copy();self.held=q.copy();self.history.clear()
            raise ValueError('Fresh hold established; wait for stationary feedback and press Reverse again')
        if not self.stationary(q,self.held,.5):raise ValueError('Wait for at least 0.5 s of healthy stationary hold feedback')
        if np.sum(abs(q-self.held))>TRACK_L1:raise ValueError('Arm drifted from the hold; no unchecked reverse')
        self.hold_confirmed=True;self.generation+=1
        self.emit('CHECKING_RETURN')
        return dict(held=q.tolist(),tau=self.tau,rate=self.rate,generation=self.generation)

    def retry_hold(self):
        if self.phase not in {'HELD','FAULT_HOLD'}:raise ValueError('Recheck hold is only available after a stop')
        q=self.sample(check_scene=False)
        if self.held is not None:
            if np.sum(abs(q-self.held))>TRACK_L1:raise ValueError('Arm drifted from its held pose; will not chase drift or relax')
        else:
            self.bus.set_positions(q);self.anchor=q.copy();self.held=q.copy()
        self.hold_confirmed=True;self.error='';self.emit('HELD',reason='Operator rechecked healthy feedback; measured pose held')

    def accept_return(self,spec,generation):
        if self.phase!='CHECKING_RETURN' or generation!=self.generation:return False
        q=self.sample()
        if not spec.get('proof',{}).get('all_valid'):raise ValueError('Recovery was not collision/zone checked')
        if np.sum(abs(q-np.array(spec['held'])))>TRACK_L1:raise ValueError('Arm moved while checking return; hold and retry')
        if not self.stationary(q,self.held,.5):raise ValueError('Held position is not settled')
        if not 0<=spec['tau']<=self.tau+1e-9:raise ValueError('Recovery cannot advance beyond traveled progress')
        self.anchor=np.array(spec['held']);self.return_spec=spec
        self.error=''
        self.emit('RECOVERY',connector_l1_rad=spec['connector_l1_rad'])
        self.previous=self.now();return self.phase=='RECOVERY'

    def stationary(self,q,target,seconds=.6):
        now=self.now()
        if not self.history or self.history[-1][0]!=now:self.history.append((now,np.asarray(q).copy()))
        while self.history and now-self.history[0][0]>seconds+.035:self.history.popleft()
        values=np.array([r[1] for r in self.history])
        return bool(self.history and now-self.history[0][0]>=seconds and
                    np.max(abs(values-target))<=REST_TOL and np.max(np.ptp(values,axis=0))<=np.radians(.12))

    def step(self):
        now=self.now()
        if self.phase=='READY' or self.phase in TERMINAL:return
        try:
            if self.phase=='PREFLIGHT':
                self.bus.poll()
                if not self.bus.fresh():
                    if now-self.phase_at>3:raise RuntimeError('No fresh initial encoders')
                    return
                q=self.sample(mode='disabled')
                if np.sum(abs(q-self.req['start']))>TRACK_L1:raise RuntimeError('Arm moved since preparation; replan before enable')
                self.arm_steps=self.bus.begin_arm(self.req['start']);self.emit('ARMING');return
            if self.phase=='ARMING':
                self.sample(mode='any')
                if np.sum(abs(self.q-self.req['start']))>TRACK_L1:raise RuntimeError('Arm shifted during enable; no relax')
                if now-self.phase_at>5:raise RuntimeError('Enable timeout; no retry/disable')
                try:next(self.arm_steps)
                except StopIteration:
                    self.sample();self.started=True;self.anchor=self.curve.at(0);self.emit('SETTLING_START')
                self.started=self.started or bool(getattr(self.bus,'enable_attempted',False));return
            if self.phase in HOLD_PHASES:
                try:
                    q=self.sample();self.stationary(q,self.held if self.held is not None else q,.5)
                    self.hold_confirmed=self.held is not None and np.sum(abs(q-self.held))<=TRACK_L1
                    if not self.hold_confirmed:self.feedback_error='Arm is not at the held target; no automatic motion or release'
                except Exception as e:
                    self.feedback_error=str(e);self.hold_confirmed=False;self.history.clear()
                return
            if self.previous is not None and now-self.previous>.08:
                raise RuntimeError('Control scheduler missed 80 ms; trajectory stopped, no catch-up')
            q=self.sample()
            if np.sum(abs(q-self.anchor))>TRACK_L1:raise RuntimeError('Tracking deviation / possible contact; no relaxation')
            elapsed=now-self.phase_at
            if self.phase.startswith('SETTLING_'):
                target=self.curve.at(self.curve.duration if self.phase=='SETTLING_GOAL' else 0)
                self.send(target)
                if self.stationary(q,target):
                    if self.phase=='SETTLING_START':self.emit('OUTWARD')
                    elif self.phase=='SETTLING_GOAL':self.emit('GOAL_HOLD')
                    else:
                        # Verify return/settling, not physical support. The
                        # original resting pose must be supported by the setup;
                        # stationary encoders cannot establish that fact.
                        self.emit('VERIFIED_START_POSITION')
                        if self.phase!='VERIFIED_START_POSITION':return
                        self.bus.release_at_verified_rest(self.req['start']);self.disabled=True
                        self.emit('RELAXED')
                elif elapsed>8:raise RuntimeError('Endpoint did not settle; holding, no release')
            elif self.phase=='OUTWARD':
                tau=min(self.curve.duration,elapsed*self.rate);self.send(self.curve.at(tau));self.tau=tau
                if tau>=self.curve.duration:self.emit('SETTLING_GOAL')
            elif self.phase=='GOAL_HOLD':
                self.send(self.curve.at(self.curve.duration))
                if elapsed>=2.:self.emit('RETURN')
            elif self.phase=='RETURN':
                tau=max(0.,self.curve.duration-elapsed*self.rate);self.send(self.curve.at(tau));self.tau=tau
                if tau<=0:self.emit('SETTLING_REST')
            elif self.phase=='RECOVERY':
                target,tau,_=recovery_at(self.curve,self.return_spec,elapsed)
                self.send(target);self.tau=min(self.tau,tau)
                if elapsed>=self.return_spec['bridge_s']+self.return_spec['return_s']:
                    self.tau=0.;self.emit('SETTLING_REST')
        except BaseException as e:
            self.halt(str(e) or type(e).__name__,fault=True)
        finally:self.previous=now

    def supported_exit(self,confirmation):
        if self.phase not in {'HELD','FAULT_HOLD'} or confirmation!='ARM PHYSICALLY SUPPORTED':
            raise ValueError('Physically support the arm first, then type ARM PHYSICALLY SUPPORTED; no relax is sent')
        self.emit('SUPPORTED_EXIT',disable_sent=False);self.closed=True

    def status(self):
        return dict(phase=self.phase,error=self.error,feedback_error=self.feedback_error,tau=self.tau,
            duration=self.curve.duration,rate=self.rate,started=self.started,disabled=self.disabled,
            hold_confirmed=bool(self.hold_confirmed),right_rad=None if self.q is None else self.q.tolist(),
            can_retry_hold=self.phase in {'HELD','FAULT_HOLD'},
            can_return=self.phase in {'HELD','FAULT_HOLD'} and bool(self.hold_confirmed),
            safe_to_close=self.phase in TERMINAL or (not self.started and self.phase=='READY'))
