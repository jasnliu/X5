"""Synthetic plant and explicitly opt-in physical transport. No import-time I/O."""
from dataclasses import asdict
import importlib.util
import math
import os
import queue
import threading
import time
from types import SimpleNamespace
from camera_playback.mit_strike import Command, Sample, Joint7Channel
from safe_zone.encoder import FRAME
from .config import ROOT, Plant, center_pose


class SimBackend:
    name = 'simulation'

    def __init__(self, plant=Plant(), realtime=False):
        self.plant, self.realtime = plant, realtime
        self.anchor = center_pose()[6]
        self.time = 1.
        self.q, self.v = self.anchor, 0.
        self.torque = plant.gravity
        self.command = Command(self.anchor,0,40,1.8,plant.gravity)
        self.pending=[]
        self.positions=[0.]*7
        self.enabled=False
        self.sample=Sample(self.q,0,self.torque,2,self.time)

    def now(self):return self.time

    def prepare(self, stop, publish):
        start=list(self.positions);target=center_pose();target[6]=self.anchor
        for i in range(1,41):
            if stop.is_set():raise InterruptedError('Stopped during simulated setup')
            self.positions=[a+(b-a)*i/40 for a,b in zip(start,target)]
            publish(dict(phase='SIMULATED CENTER + CLOSED GRIPPER',positions=self.positions,gripper_deg=7.))
            if self.realtime:time.sleep(.025)
        self.q,self.v=self.anchor,0.;self.pending=[];self.enabled=True
        self.command=Command(self.anchor,0,40,1.8,self.plant.gravity)
        self.sample=Sample(self.q,0,self.plant.gravity,2,self.time)
        return self.anchor,self.plant.gravity

    def send(self, command):
        if not self.enabled:raise RuntimeError('Simulation is not prepared')
        self.pending.append((self.time+self.plant.delay,command))

    def advance(self, seconds, stop=None):
        if self.realtime and seconds>0:time.sleep(seconds)
        steps=max(1,math.ceil(seconds/.0005));dt=seconds/steps
        for _ in range(steps):
            if stop and stop.is_set():raise InterruptedError('Stopped')
            while self.pending and self.pending[0][0]<=self.time+1e-12:
                _,self.command=self.pending.pop(0)
            c,p=self.command,self.plant
            self.torque=c.kp*(c.position-self.q)+c.kd*(c.velocity-self.v)+c.torque
            a=(self.torque-p.gravity-p.friction*self.v)/p.inertia
            self.v+=a*dt;self.q+=self.v*dt;self.time+=dt
        q=round(self.q/self.plant.encoder_step)*self.plant.encoder_step if self.plant.encoder_step else self.q
        v=round(self.v/(66/65535))*(66/65535)
        self.sample=Sample(q,v,self.torque,2,self.time)
        self.positions[6]=q

    def receive(self):return self.sample
    def relax(self):self.enabled=False
    def close(self):self.relax()


def notify_motion():
    """Host's existing notifier. Delivery is required before physical setup."""
    from pathlib import Path
    path=Path.home()/'.local/bin/codex-ntfy.py'
    spec=importlib.util.spec_from_file_location('strike_lab_ntfy',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    if not module.post_ntfy('Strike lab will close the right gripper, center the right arm, then test J7 only. No stick or recording.',
                            'Robot experiment starting', 'warning,robot'):
        raise RuntimeError('Motion notification delivery failed; no movement authorized')
    module.play_sound('{"type":"agent-turn-complete"}')


class ExperimentChannel(Joint7Channel):
    temperature=None

    def decode(self,frame,at):
        super().decode(frame,at)
        cid,dlc,data=FRAME.unpack(frame)
        if ((cid>>24)&31)==2 and ((cid>>8)&255)==7 and dlc==8:
            self.temperature=int.from_bytes(data[6:8],'big')*.1


class HardwareBackend:
    name='hardware'

    def __init__(self, limits, notifier=notify_motion, bus_factory=None, channel_factory=None, motion_started=None):
        if os.environ.get('STRIKE_LAB_OFFLINE_ONLY')=='1':
            raise RuntimeError('Hardware is forbidden by STRIKE_LAB_OFFLINE_ONLY')
        self.limits=limits;self.notifier=notifier;self.motion_started=motion_started
        self.bus_factory=bus_factory;self.channel_factory=channel_factory
        self.bus=None;self.channel=None;self.positions=center_pose();self.last_poll=0.
        self.anchor=self.positions[6];self.held=None;self.temperature=None

    def now(self):return time.monotonic()

    def prepare(self, stop, publish):
        # Imports do not connect; constructors below are reached only with
        # explicit --hardware AND a prepare request in the worker process.
        from centering.motors import Motors, RIGHT_GRIPPER_CLOSED
        from camera_playback.mit_strike import Joint7Channel, Joint7Worker, StrikeSettings
        if self.notifier:self.notifier()
        for remaining in (range(10,0,-1) if self.notifier else ()):
            publish(dict(phase=f'NOTIFIED — movement in {remaining}s',positions=self.positions))
            if stop.wait(1):raise InterruptedError('Stopped before hardware connection')
        self.bus=(self.bus_factory or Motors)('right',True)
        deadline=self.now()+2
        while not self.bus.fresh():
            if stop.is_set():raise InterruptedError('Stopped')
            self.bus.poll()
            if self.now()>deadline:raise RuntimeError('Fresh encoders unavailable before setup')
            time.sleep(.01)
        if self.motion_started:self.motion_started()
        setup=self.bus.center(center_pose(),RIGHT_GRIPPER_CLOSED,confirm_enabled=True)
        for _ in setup:
            if stop.is_set():raise InterruptedError('Stopped during drive setup')
            self.bus.poll();time.sleep(.002)
        # Confirm gripper closure before moving the arm to custom center.
        deadline=self.now()+3
        while abs(self.bus.states['right',8][0]-RIGHT_GRIPPER_CLOSED)>math.radians(.75):
            if stop.is_set():raise InterruptedError('Stopped during gripper closure')
            self.bus.poll()
            if self.now()>deadline:raise RuntimeError('Gripper closure timeout')
            time.sleep(.01)
        self.bus.set_positions(center_pose())
        deadline=self.now()+15;stable=None
        while True:
            if stop.is_set():raise InterruptedError('Stopped during centering')
            self.bus.poll()
            self.positions=[self.bus.states['right',i][0] for i in range(1,8)]
            error=max(abs(a-b) for a,b in zip(self.positions,center_pose()))
            if error<math.radians(.15):
                stable=stable or self.now()
                if self.now()-stable>.15:break
            else:stable=None
            if self.now()>deadline:raise RuntimeError('Custom right-center timeout')
            publish(dict(phase='CENTERING RIGHT ARM',positions=self.positions,gripper_deg=7.))
            time.sleep(.01)
        # Fix one measured reference for this session. Never reset it per strike.
        self.anchor=min(center_pose()[6],self.positions[6])
        self.held=[self.bus.states['right',i][0] for i in range(1,9)]
        self.channel=(self.channel_factory or ExperimentChannel)(self.bus.sockets['right'].getsockname()[0])
        helper=Joint7Worker('unused',self.anchor,-1.4,1.4,StrikeSettings(),queue.Queue(),stop,
                           lambda s:publish(dict(phase='ARMING J7 MIT',positions=list(self.positions),
                                                 sample=asdict(s.sample) if s.sample else None)))
        helper._prepare(self.channel)
        self.bus.right_joint7_session=SimpleNamespace(owns_feedback=True)
        self.last_poll=self.now()
        return self.anchor,helper.controller.bias

    def send(self, command):self.channel.send(command.frame())

    def advance(self, seconds, stop=None):
        # A CAN reply usually arrives much sooner than the control period.
        # Drain it, but do not turn an early reply into an extra command tick.
        deadline=self.now()+max(0.,seconds)
        while True:
            if stop and stop.is_set():raise InterruptedError('Stopped')
            remaining=deadline-self.now()
            if remaining<=0:return
            self.channel.wait(remaining)
            self.channel.receive()

    def receive(self):
        self.channel.receive()
        self.temperature=getattr(self.channel,'temperature',None)
        now=self.now()
        if now-self.last_poll>=.02:
            self.bus.poll();self.last_poll=now
            for i in list(range(1,7))+[8]:
                q,state,_=self.bus.states['right',i]
                if state!=2:raise RuntimeError(f'Held right motor {i} is not running')
                if abs(q-self.held[i-1])>math.radians(self.limits.held_joint_drift_deg):
                    raise RuntimeError(f'Held right motor {i} moved unexpectedly')
            self.positions=[self.bus.states['right',i][0] for i in range(1,8)]
        sample=self.channel.sample
        if sample:self.positions[6]=sample.position
        return sample

    def relax(self):
        if self.bus is not None:
            self.bus.right_joint7_session=None
            self.bus.relax()
            # Let the final disable burst drain before poll sends state queries
            # on adapters configured with a ten-frame transmit queue.
            time.sleep(.02)
            deadline=self.now()+1.
            while self.now()<deadline:
                self.bus.poll()
                if all(self.bus.states.get(('right',i),(0,-1,0))[1]==0 for i in range(1,9)):
                    return
                time.sleep(.01)
            raise RuntimeError('Right-arm disable confirmation missing')

    def close(self):
        try:self.relax()
        finally:
            if self.channel:self.channel.close()
            if self.bus:
                for sock in self.bus.sockets.values():sock.close()


def emergency_disable_right():
    """Supervisor fallback only AFTER a dead physical worker; never enables/moves.

    Separate socket ownership starts after the old controller has exited. This
    cannot make a disconnected CAN adapter deliver a stop; failures stay visible.
    """
    if os.environ.get('STRIKE_LAB_OFFLINE_ONLY')=='1':
        raise RuntimeError('Physical disable transport forbidden in offline tests')
    import socket
    import select
    from centering.motors import packet
    from safe_zone.encoder import request_frame, EFF
    sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
    try:
        sock.bind(('can0',));sock.setblocking(False)
        for _ in range(3):
            for i in range(1,9):
                if sock.send(packet(4,i))!=16:raise RuntimeError('Short emergency disable write')
                time.sleep(.001)
        disabled=set();deadline=time.monotonic()+.6;last_query=0.
        while time.monotonic()<deadline:
            if time.monotonic()-last_query>.05:
                for i in range(1,9):sock.send(request_frame(i))
                last_query=time.monotonic()
            if not select.select([sock],[],[],.01)[0]:continue
            cid,dlc,data=FRAME.unpack(sock.recv(16));motor=(cid>>8)&255
            if cid&EFF and (cid>>24)&31==2 and dlc==8 and motor in range(1,9):
                if (cid>>22)&3==0:disabled.add(motor)
                else:disabled.discard(motor)
            if len(disabled)==8:return
        raise RuntimeError('Worker exited; right-arm disable could not be confirmed')
    finally:sock.close()
