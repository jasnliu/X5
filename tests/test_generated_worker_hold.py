"""Run the actual MIT worker loops on fake channels; never open a CAN socket."""
import pickle
import queue
import threading
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from camera_playback.hybrid_strike import _run_hybrid,load_tuning
from camera_playback.mit_strike import Sample
from safe_zone.encoder import FRAME


def test_real_hybrid_loop_hold_return_fault_never_jumps_back_to_strike_corridor():
    t=[0.];stop=threading.Event();hold=threading.Event();goal=SimpleNamespace(value=float('nan'))
    heartbeat=SimpleNamespace(value=0.);states=[];frames=[];events=[]
    class Channel:
        def __init__(self,interface):self.sample=Sample(.6,0.,0.,2,0.)
        def receive(self):
            t[0]+=.002;heartbeat.value=t[0]
            self.sample=Sample(self.sample.position,self.sample.velocity,0.,2,t[0])
            if t[0]>.02 and not hold.is_set():hold.set();events.append('hold')
            if t[0]>.1:goal.value=.2  # checked path return leaves the STRIKE corridor
            if t[0]>2.3:raise RuntimeError('injected worker error after return')
        def wait(self,seconds):t[0]+=max(0.,min(.002,seconds))
        def send(self,frame):
            cid,_,data=FRAME.unpack(frame);assert (cid>>24)&31==1 and cid&255==7
            # right motor raw position is opposite the displayed joint
            q=-(int.from_bytes(data[:2],'big')/65535*25.14-12.57)
            v=-(int.from_bytes(data[2:4],'big')/65535*66.-33.)
            frames.append((t[0],q));self.sample=Sample(q,v,0.,2,t[0])
        def close(self):events.append('closed')
    def prepare(helper,channel):helper.last_command_at=t[0];helper.controller.bias=0.
    sender=SimpleNamespace(send=lambda x:states.append(pickle.loads(x)),close=lambda:None)
    p,r,l=load_tuning()
    with patch('camera_playback.hybrid_strike.time.monotonic',side_effect=lambda:t[0]),\
         patch('camera_playback.hybrid_strike.Joint7Worker._prepare',prepare),patch('signal.signal') as handler:
        _run_hybrid('fake',.6,.4,.8,p,r,l,queue.Queue(),stop,heartbeat,sender,Channel,
                    path_hold=hold,path_goal=goal,recovery_bounds=(-1.,1.),parent_handles_interrupt=True)
    import signal
    handler.assert_called_once_with(signal.SIGINT,signal.SIG_IGN)
    assert states[-1].error=='injected worker error after return'
    assert any(s.phase=='generated powered hold/return' for s in states)
    assert frames[-1][1]<.3  # NOT clipped back to original lower=.4
    assert abs(frames[-1][1]-frames[-2][1])<.001
    assert events==['hold','closed']
    assert all(abs(q-.6)<.001 for time,q in frames if time<.1)


def test_actual_snare_loop_latched_hold_keeps_sign_and_cancels_schedule():
    from camera_playback.snare_beat import _run
    from tests.test_snare_beat import SnareBeatTests
    # Build the same unchanged powered stroke fixture as the existing beat tests.
    SnareBeatTests.setUpClass();f=SnareBeatTests;t=[0.];stop=threading.Event();hold=threading.Event()
    goal=SimpleNamespace(value=-f.anchor);heartbeat=SimpleNamespace(value=0.)
    clock=[0.,.6,0.,0.,0.];states=[];frames=[]
    class Channel:
        def __init__(self):self.sample=Sample(f.anchor,0.,0.,2,0.)
        def receive(self):
            t[0]+=.002;heartbeat.value=t[0]
            self.sample=Sample(self.sample.position,self.sample.velocity,0.,2,t[0])
            if t[0]>.02 and not hold.is_set():goal.value=float('nan');hold.set()
            if t[0]>.1:goal.value=.2
            if t[0]>1.5:stop.set()
        def wait(self,s):t[0]+=max(0.,min(s,.002))
        def send(self,frame):
            cid,_,data=FRAME.unpack(frame);assert (cid>>24)&31==1 and cid&255==6
            q=int.from_bytes(data[:2],'big')/65535*25.14-12.57
            v=int.from_bytes(data[2:4],'big')/65535*66.-33.
            self.sample=Sample(q,v,0.,2,t[0]);frames.append(q)
        def close(self):pass
    channel=Channel();sender=SimpleNamespace(send=lambda b:states.append(pickle.loads(b)),close=lambda:None)
    with patch('time.monotonic',side_effect=lambda:t[0]),patch('camera_playback.snare_beat.signal.signal'),\
         patch('snare_lab.runner._LeftJoint6Channel',return_value=channel),patch('camera_playback.snare_beat.Joint7Worker._prepare'):
        _run('fake',f.template,f.rules,f.limits,-.75,.75,goal,clock,stop,heartbeat,sender,path_hold=hold)
    assert not [s.error for s in states if s.error]
    assert any(s.phase=='generated powered hold/return' for s in states)
    assert abs(frames[-1]+.2)<.02
    assert not states[-1].moving and not states[-1].scheduled
