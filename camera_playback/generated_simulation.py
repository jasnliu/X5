"""Explicit ideal motors for offline generated workflow tests; never devices."""
from collections import deque
from types import SimpleNamespace
import time
import multiprocessing as mp
import numpy as np
from .generated_transport import GeneratedMotors
from .left_hold import LEFT_GRIPPER_TARGET
from centering.motors import RIGHT_GRIPPER_CLOSED


class SimulatedGeneratedMotors:
    def __init__(self,rest,clock=time.monotonic,*,streaming=False):
        self.rest={s:np.asarray(q,float).copy() for s,q in rest.items()}
        self.q={s:q.copy() for s,q in self.rest.items()};self.target={s:q.copy() for s,q in self.rest.items()}
        self.streaming_enabled=streaming;self.path_session=None
        if streaming:
            self.shared=mp.get_context('spawn').Array('d',np.concatenate([self.q[s] for s in ('left','right')]),lock=False)
            values=np.frombuffer(self.shared).reshape(2,7)
            self.q={s:values[i] for i,s in enumerate(('left','right'))}
        self.active=False;self.clock=clock;self.states={};self.generated_history=deque(maxlen=100)
        self.control_side='right';self.control_gripper=True;self.center_disabled={};self.left_monitor=None
        self.right_joint7_session=self.left_joint6_session=None;self.closed=False;self.calls=[]
        self.motor8_feedback={};self.handoff_goal=None;self.poll()
    def poll(self):
        now=self.clock()
        self.states={(s,i):(float(q[i-1]) if i<8 else (LEFT_GRIPPER_TARGET if s=='left' else RIGHT_GRIPPER_CLOSED),2 if self.active else 0,now)
                     for s,q in self.q.items() for i in range(1,9)}
        self.generated_history.append((now,{s:q.copy() for s,q in self.q.items()}))
        while self.generated_history and now-self.generated_history[0][0]>.65:self.generated_history.popleft()
    def fresh(self):return not self.closed and len(self.states)==16
    def positions_for(self,s):return self.q[s].copy()
    def positions(self):
        return {**{f'openarmx_{s}_joint{i+1}':float(q[i]) for s,q in self.q.items() for i in range(7)},
                'openarmx_left_finger_joint1':0.,'openarmx_right_finger_joint1':0.}
    def arm_at_rest(self):
        self.calls.append(('enable_at_rest',));self.active=True;self.poll();yield
    def command_arm(self,s,target,*,hold=False):
        if not self.active:raise RuntimeError('Simulated motors disabled')
        if self.path_session is not None:raise RuntimeError('Stop simulated path writer before GUI commands')
        self.q[s][:]=target;self.target[s]=self.q[s].copy();self.calls.append(('hold' if hold else 'path',s,self.q[s].tolist()))
    def start_stream(self,curve,side,step):
        from .generated_stream import StreamSession
        return StreamSession(self,curve,side,step,geometries=self.stream_geometry,
            directory=self.stream_directory,factory=SimulatedStreamMotors,shared=self.shared)
    def settled(self,targets):
        h=self.generated_history
        if len(h)<2 or h[-1][0]-h[0][0]<.6:return False
        return all(np.max(abs(np.array([v[s] for _,v in h])-goal))<=np.radians(.2)
                   and np.max(np.ptp(np.array([v[s] for _,v in h]),axis=0))<=np.radians(.12) for s,goal in targets.items())
    def release_rest(self):
        if not self.settled(self.rest):raise RuntimeError('Simulated rest not settled')
        self.calls.append(('release_at_rest',));self.active=False
    def emergency_release(self):
        if self.path_session is not None:
            self.path_session.stop();self.path_session.detach()
        self.calls.append(('emergency_release',));self.active=False
        self.right_joint7_session=self.left_joint6_session=None
        self.poll()
    def close(self):
        if self.path_session is not None:
            self.path_session.stop();self.path_session.detach()
        if self.active:raise RuntimeError('Simulation detected close of powered owner')
        self.closed=True


class SimulatedStreamMotors:
    """Spawn-compatible fake transport exercising the REAL 200 Hz worker loop."""
    def __init__(self,directory,handles,side,special,shared):
        # Audit even subprocesses: a simulation must never open real CAN.
        import sys,socket
        def no_can(event,args):
            if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:
                raise AssertionError('Physical CAN forbidden in generated simulation')
        sys.addaudithook(no_can)
        self.side=side;self.shared=shared;self.states={};self.poll()
    def poll(self):
        values=np.asarray(self.shared[:]).reshape(2,7)
        now=time.monotonic()
        self.states={(s,i):(float(values[j,i-1]) if i<8 else (LEFT_GRIPPER_TARGET if s=='left' else RIGHT_GRIPPER_CLOSED),2,now)
                     for j,s in enumerate(('left','right')) for i in range(1,9)}
    def prime(self,check):check();self.poll()
    def fresh(self):return True
    def command(self,q):
        offset=0 if self.side=='left' else 7
        self.shared[offset:offset+7]=q
    def close(self):pass
