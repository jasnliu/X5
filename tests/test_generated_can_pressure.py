"""Real generated worker branches and CAN scheduling, fake devices only."""
import errno
import pickle
import queue
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch
from dataclasses import replace
import numpy as np
import pytest
from safe_zone.encoder import FRAME, request_frame
from camera_playback.mit_strike import Sample


@pytest.mark.parametrize('worker',['right','left','recovery-right','recovery-left'])
@pytest.mark.parametrize('fault',['brief','persistent','network','stale'])
def test_actual_hold_workers_under_queue_pressure(worker,fault):
    from camera_playback.generated_worker_hold import recover_worker
    from camera_playback.hybrid_strike import _run_hybrid,load_tuning
    from camera_playback.snare_beat import _run
    from tests.test_snare_beat import SnareBeatTests
    left=worker.endswith('left');motor=6 if left else 7;sign=-1. if left else 1.
    t=[0.];stop=threading.Event();hold=threading.Event();hold.set()
    heartbeat=SimpleNamespace(value=0.);goal=SimpleNamespace(value=sign*.3)
    states=[];frames=[];refused=[];accepted_after_pressure=[]
    class Channel:
        def __init__(self,*args,**kwargs):self.sample=Sample(.3,0.,0.,2,0.);self.mode=0
        def receive(self):
            t[0]+=.0005;heartbeat.value=t[0]
            self.sample=replace(self.sample,at=t[0]-(.1 if fault=='stale' and t[0]>.06 else 0.))
            if t[0]>.065:goal.value=sign*.2
            if t[0]>.20:stop.set()
        def wait(self,dt):t[0]+=max(0.,min(dt,.001))
        def send(self,frame):
            cid,_,data=FRAME.unpack(frame);kind=(cid>>24)&31
            assert cid&255==motor and kind in (1,2,17)
            if kind!=1:return
            if .06<t[0]<.068 or fault in ('persistent','network') and t[0]>.06:
                refused.append(t[0])
                raise OSError(errno.ENETDOWN if fault=='network' else
                              errno.ENOBUFS if len(refused)%2 else errno.EAGAIN,'injected TX pressure')
            q=-sign*(int.from_bytes(data[:2],'big')/65535*25.14-12.57)
            self.sample=replace(self.sample,position=q)
            frames.append((t[0],frame))
            if t[0]>.068:accepted_after_pressure.append(t[0])
        def close(self):pass
    def prepare(helper,channel):helper.last_command_at=t[0];helper.controller.bias=0.
    sender=SimpleNamespace(send=lambda b:states.append(pickle.loads(b)),close=lambda:None)
    with patch('time.monotonic',side_effect=lambda:t[0]),\
         patch('time.sleep',side_effect=lambda dt:t.__setitem__(0,t[0]+dt)),patch('signal.signal'),\
         patch('camera_playback.mit_strike.Joint7Worker._prepare',prepare),\
         patch('snare_lab.runner._LeftJoint6Channel',side_effect=Channel):
        if worker.startswith('recovery'):
            recover_worker('fake','left' if left else 'right',-1.,1.,goal,stop,heartbeat,sender,Channel)
        elif left:
            SnareBeatTests.setUpClass();f=SnareBeatTests
            _run('fake',f.template,f.rules,f.limits,-1.,1.,goal,[0.,.6,0.,0.,0.],stop,heartbeat,sender,path_hold=hold)
        else:
            p,r,l=load_tuning()
            _run_hybrid('fake',.3,-1.,1.,p,r,l,queue.Queue(),stop,heartbeat,sender,Channel,
                        path_hold=hold,path_goal=goal,recovery_bounds=(-1.,1.))
    errors=[s.error for s in states if s.error]
    if fault=='brief':
        assert refused and accepted_after_pressure and not errors, errors
        assert max(np.diff([at for at,_ in frames]))<.025
    elif fault in ('persistent','network'):
        assert errors and 'transmit' in errors[-1].lower(),errors
        assert t[0]<.10  # Bounded failure, never unlimited retry or blind advance.
    else:assert errors and 'feedback' in errors[-1].lower()


def test_rejected_hold_command_rolls_back_integrator_and_recomputes_new_goal():
    from camera_playback.generated_worker_hold import PoweredPathHold
    sample=Sample(.2,0.,.1,2,0.)
    held=PoweredPathHold(sample,0.);before=held.controller.__dict__.copy()
    send=Mock(side_effect=OSError(errno.ENOBUFS,'full'))
    assert not held.send(send,replace(sample,at=.005),.005,.4,-1.,1.,7,1.)
    assert held.controller.__dict__==before
    fresh=replace(sample,position=.199,at=.006)
    expected=PoweredPathHold(sample,0.).update(fresh,.006,.1,-1.,1.).frame(7,1.)
    sent=[]
    assert held.send(sent.append,fresh,.006,.1,-1.,1.,7,1.)
    assert sent==[expected] and held.pressure.retries==1


def test_queue_deadline_does_not_allow_late_success_to_hide_outage():
    from camera_playback.generated_can import QueuePressure,TransmitError
    pressure=QueuePressure();send=Mock(side_effect=OSError(errno.ENOBUFS,'full'))
    assert not pressure.send(send,b'frame',1.)
    send.side_effect=None
    with pytest.raises(TransmitError,match='20 ms'):pressure.send(send,b'new frame',1.021)
    assert send.call_count==1


@pytest.mark.parametrize('side',['left','right'])
@pytest.mark.parametrize('case',['fault','temperature','receive_error'])
def test_real_receive_fault_still_latches_even_when_queries_also_fail(side,case):
    from camera_playback.generated_transport import GeneratedMotors
    from tests.test_left_center_hold import fake_bus
    from safe_zone.encoder import EFF
    bus=GeneratedMotors.from_owner(fake_bus(),{s:np.zeros(7) for s in ('left','right')})
    if case=='receive_error':bus.sockets[side].recv=Mock(side_effect=OSError(errno.ENETDOWN,'RX failed'))
    else:
        cid=EFF|(21 if case=='fault' else 2)<<24|3<<8
        bus.sockets[side].recv=Mock(side_effect=[FRAME.pack(cid,8,b'\x80\x00\x80\x00\x80\x00\x02\x8a')])
    for sock in bus.sockets.values():sock.send=Mock(side_effect=OSError(errno.ENOBUFS,'TX full'))
    with pytest.raises((OSError,RuntimeError),match={'fault':'fault','temperature':'too hot','receive_error':'RX failed'}[case]):bus.poll()
    assert bus.feedback_failed_at is not None and not bus.fresh()


@pytest.mark.parametrize('sign,motor,kp,kd',[(1.,7,40.,1.8),(-1.,6,160.,4.)])
def test_healthy_hold_command_bytes_unchanged(sign,motor,kp,kd):
    from camera_playback.generated_worker_hold import PoweredPathHold
    sample=Sample(.2,0.,.1,2,0.)
    held=PoweredPathHold(sample,0.,kp=kp,kd=kd)
    old=PoweredPathHold(sample,0.,kp=kp,kd=kd);sent=[]
    for tick in range(1,101):
        now=tick*.005;sample=replace(sample,at=now)
        frame=old.update(sample,now,.3,-1.,1.).frame(motor,sign)
        assert held.send(sent.append,sample,now,.3,-1.,1.,motor,sign)
        assert sent[-1]==frame


def test_query_transmit_error_is_not_latched_as_lost_encoder_feedback():
    from camera_playback.generated_transport import GeneratedMotors
    from camera_playback.generated_can import TransmitError
    from tests.test_left_center_hold import fake_bus
    bus=GeneratedMotors.from_owner(fake_bus(),{s:np.zeros(7) for s in ('left','right')})
    bus.states={k:(q,state,time-.05) for k,(q,state,time) in bus.states.items()}
    for sock in bus.sockets.values():sock.send=Mock(side_effect=OSError(errno.ENOBUFS,'full'))
    with pytest.raises(TransmitError,match='query CAN transmit failed'):bus.poll()
    assert bus.feedback_failed_at is None and bus.fresh()


@pytest.mark.parametrize('period,selected,sweep',[(.005,'right',.020),(.020,None,.080)])
def test_queries_bounded_fair_and_skip_fresh_mit_replies(period,selected,sweep):
    from camera_playback.generated_can import FeedbackQueries
    t=[1.];socks={s:Mock(send=Mock(return_value=16)) for s in ('right','left')}
    bus=SimpleNamespace(sockets=socks,states={('right',7):(.1,2,t[0])})
    schedule=FeedbackQueries(period,selected)
    with patch('time.monotonic',side_effect=lambda:t[0]):
        for tick in range(4):
            bus.states['right',7]=(.1,2,t[0])
            before={s:len(sock.send.call_args_list) for s,sock in socks.items()}
            schedule.poll(bus);schedule.poll(bus)
            for s,sock in socks.items():assert len(sock.send.call_args_list)-before[s]<=2
            t[0]+=period
    assert request_frame(7) not in [c.args[0] for c in socks['right'].send.call_args_list]
    for side in socks:
        seen={c.args[0] for c in socks[side].send.call_args_list}
        assert seen=={request_frame(i) for i in range(1,9) if not(side=='right' and i==7)}


@pytest.mark.parametrize('ready',[True,False])
def test_parent_queries_follow_received_freshness_not_child_liveness(ready):
    from camera_playback.generated_transport import GeneratedMotors
    from tests.test_left_center_hold import fake_bus
    bus=GeneratedMotors.from_owner(fake_bus(),{s:np.zeros(7) for s in ('left','right')})
    bus.states={k:(q,state,at-.01) for k,(q,state,at) in bus.states.items()}
    session=SimpleNamespace(queries_ready=ready,owns_feedback=ready)
    bus.playback_session=session
    bus.poll()
    assert all(not sock.sock.sock.send.called for sock in bus.sockets.values())
    bus.states={k:(q,state,at-.06) for k,(q,state,at) in bus.states.items()}
    bus.feedback_queries.last_tick=-float('inf')
    bus.poll()
    assert all(sock.sock.sock.send.call_count==8 for sock in bus.sockets.values())


@pytest.mark.parametrize('side',['left','right'])
def test_recovery_startup_queries_tolerate_brief_pressure_without_arming(side):
    from camera_playback.generated_worker_hold import recover_worker
    t=[0.];stop=threading.Event();heartbeat=SimpleNamespace(value=0.)
    goal=SimpleNamespace(value=float('nan'));states=[];failures=set();accepted=[]
    class Channel:
        def __init__(self,*args,**kwargs):self.sample=None;self.mode=None
        def receive(self):
            t[0]+=.001;heartbeat.value=t[0]
            if self.sample is not None:self.sample=replace(self.sample,at=t[0])
            if t[0]>.05:stop.set()
        def send(self,frame):
            kind=(FRAME.unpack(frame)[0]>>24)&31
            assert kind in (1,2,17)
            if kind in (2,17) and kind not in failures:
                failures.add(kind);raise OSError(errno.ENOBUFS,'query refused once')
            accepted.append(kind)
            if kind==2:self.sample=Sample(0.,0.,0.,2,t[0])
            if kind==17:self.mode=0
            if kind==1:assert self.mode==0 and self.sample is not None
        def close(self):pass
    sender=SimpleNamespace(send=lambda b:states.append(pickle.loads(b)),close=lambda:None)
    with patch('time.monotonic',side_effect=lambda:t[0]),patch('signal.signal'),\
         patch('time.sleep',side_effect=lambda dt:t.__setitem__(0,t[0]+dt)):
        recover_worker('fake',side,-1.,1.,goal,stop,heartbeat,sender,Channel)
    assert failures=={2,17} and 1 in accepted and not [s.error for s in states if s.error]


@pytest.mark.parametrize('side',['right','left'])
def test_actual_controller_and_both_mit_holds_share_finite_can_queues(side):
    """Discrete CAN model, not a physical timing/performance certification.

    Actual parent/child decoders, 200 Hz Controller, CSP send gate, and both MIT
    Hold controllers; independent receive copies; ten-frame transmit queues.
    Burst interference is injected during an otherwise healthy computed route.
    """
    import io,struct
    from collections import deque
    from camera_playback.generated_stream import StreamMotors,run_controller
    from camera_playback.generated_controller import GeneratedController
    from camera_playback.generated_transport import GeneratedMotors
    from camera_playback.generated_worker_hold import PoweredPathHold
    from camera_playback.generated_can import FeedbackQueries
    from camera_playback.playback_transport import RetrySocket
    from tests.test_left_center_hold import fake_bus
    from tests.test_generated_stream import make_curve
    from camera_playback.left_hold import LEFT_GRIPPER_TARGET
    from centering.motors import RIGHT_GRIPPER_CLOSED
    from safe_zone.encoder import EFF,joint_to_motor
    t=[1.];queues={s:deque() for s in ('right','left')};listeners={s:[] for s in queues}
    positions={s:np.zeros(8) for s in queues}
    positions['right'][7]=RIGHT_GRIPPER_CLOSED;positions['left'][7]=LEFT_GRIPPER_TARGET
    maximum={s:0 for s in queues};overflows={s:0 for s in queues};frames=[]
    def feedback_frame(s,i):
        raw=joint_to_motor(s,i,positions[s][i-1]) if i<8 else -positions[s][7]
        code=int(round((raw+12.57)/25.14*65535))
        data=code.to_bytes(2,'big')+b'\x80\x00\x80\x00\x00\xc8'
        return FRAME.pack(EFF|2<<24|2<<22|i<<8,8,data)
    def deliver(s,frame):
        if frame is None:return
        cid,_,data=FRAME.unpack(frame);kind=(cid>>24)&31;i=cid&255
        if kind==18:
            raw=struct.unpack('<f',data[4:])[0]
            from safe_zone.encoder import encoder_to_joint
            positions[s][i-1]=encoder_to_joint(s,i,raw)
        elif kind==1:
            raw=int.from_bytes(data[:2],'big')/65535*25.14-12.57
            positions[s][i-1]=-raw
        assert kind in (1,2,18)  # No mode, enable, disable, speed or gain writes.
        response=feedback_frame(s,i)
        for listener in listeners[s]:listener.rx.append(response)
    def advance(dt):
        t[0]+=dt
        for s,q in queues.items():
            while q and q[0][0]<=t[0]:deliver(s,q.popleft()[1])
    class Socket:
        def __init__(self,s):self.side=s;self.rx=deque();listeners[s].append(self)
        def send(self,frame):
            s=self.side;q=queues[s]
            if len(q)>=10:
                overflows[s]+=1;raise OSError(errno.ENOBUFS,'simulated ten-frame TX queue full')
            q.append((max(t[0],q[-1][0] if q else t[0])+.00025,frame))
            maximum[s]=max(maximum[s],len(q));frames.append((t[0],s,frame));return 16
        def recv(self,n):
            if not self.rx:raise BlockingIOError
            return self.rx.popleft()
    with patch('time.monotonic',side_effect=lambda:t[0]),patch('time.sleep',side_effect=advance):
        parent=GeneratedMotors.from_owner(fake_bus(),{s:np.zeros(7) for s in queues})
        parent.sockets={s:RetrySocket(Socket(s)) for s in queues}
        parent.active=True
        parent.states={(s,i):(positions[s][i-1],2,t[0]) for s in queues for i in range(1,9)}
        child=StreamMotors.__new__(StreamMotors);child.__dict__.update(parent.__dict__)
        child.sockets={s:RetrySocket(Socket(s)) for s in queues};child.states=parent.states.copy()
        child.control_side=side;child.command_file=io.StringIO();child.feedback_queries=FeedbackQueries(.005,side)
        holds={};goals={s:SimpleNamespace(value=0.) for s in queues};last={s:0. for s in queues}
        sockets={s:Socket(s) for s in queues}
        for s in queues:holds[s]=PoweredPathHold(Sample(0.,0.,0.,2,t[0]),t[0],
            kp=160. if s=='left' else 40.,kd=4. if s=='left' else 1.8)
        held=threading.Event();held.set()
        child.special=dict(motor=7 if side=='right' else 6,goal=goals[side],hold=held)
        parent.playback_session=SimpleNamespace(queries_ready=True,owns_feedback=True)
        next_parent=[1.];injected={s:False for s in queues};ages=[];accepted={s:[] for s in queues}
        def check():
            advance(0.)
            for s in queues:
                period=.002 if s=='left' else .010
                if t[0]-last[s]<period:continue
                if t[0]>=1.25 and not injected[s]:
                    # Interference overlaps each owner's next command. Dummy
                    # queue slots generate no made-up encoder responses.
                    q=queues[s]
                    while len(q)<10:q.append((max(t[0],q[-1][0] if q else t[0])+.00025,None))
                    injected[s]=True
                i=6 if s=='left' else 7;sign=-1. if s=='left' else 1.
                # Independent socket copies include the same MIT feedback.
                while sockets[s].rx:sockets[s].recv(16)
                sample=Sample(sign*positions[s][i-1],0.,0.,2,
                              child.states[s,i][2])
                if holds[s].send(sockets[s].send,sample,t[0],sign*goals[s].value,-1.,1.,i,sign):
                    last[s]=t[0];accepted[s].append(t[0])
            if t[0]>=next_parent[0]:parent.poll();next_parent[0]=t[0]+.020
        def read(bus,now):
            ages.append(max(now-v[2] for v in bus.states.values()))
            assert bus.fresh() and ages[-1]<.120
            return {s:np.array([bus.states[s,i][0] for i in range(1,8)]) for s in queues}
        curve=make_curve(side,delta=np.linspace(.001,.008,7));rows=[]
        engine=GeneratedController(child,curve,side,None,{s:np.zeros(7) for s in queues},read,now=lambda:t[0])
        run_controller(engine,check,lambda *row:rows.append(row),clock=lambda:t[0],wait=advance)
        assert engine.complete and all(injected.values()) and all(v>0 for v in overflows.values())
        assert all(h.pressure.retries>0 for h in holds.values())
        assert max(ages)<.120 and all(max(np.diff(a))<.020 for a in accepted.values())
        for elapsed,tau,q,_ in rows:np.testing.assert_array_equal(q,curve.at(elapsed))
        for _,s,frame in frames:
            cid=FRAME.unpack(frame)[0];kind=(cid>>24)&31
            if kind==18:assert s==side and cid&255!=child.special['motor']
