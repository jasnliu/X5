"""Right-only CSP adapter over X5's proven centering packet/feedback code.

Never calls its generic center-confirm/retry or relax routines: those may
disable motors. This adapter has a single supported-rest release gate, fixed
closed-gripper target, no MIT/gain/zeroing commands, and audits every TX/RX.
"""
import json
import struct
import time

import numpy as np

from centering.motors import (Motors,packet,parameter,CURRENT,GRIPPER_CURRENT,
                              RIGHT_GRIPPER_CLOSED,SPEED,RIGHT_PLAYBACK_SPEED)
from safe_zone.encoder import FRAME,EFF,request_frame,joint_to_motor


class GateSocket:
    def __init__(self,sock,side,owner):self.sock=sock;self.side=side;self.owner=owner
    def send(self,frame):
        if frame not in self.owner.queries:
            if self.side!='right' or not self.owner.command_allowed(frame):
                raise RuntimeError('Planner wire gate rejected actuator packet')
        self.owner.audit('tx_attempt',self.side,frame)
        n=self.sock.send(frame);self.owner.audit('tx_sent',self.side,frame,bytes_sent=n)
        if n!=FRAME.size:raise RuntimeError('Short CAN transmission')
        return n
    def recv(self,n):
        data=self.sock.recv(n);self.owner.audit('rx',self.side,data)
        if len(data)!=FRAME.size:raise RuntimeError('Malformed CAN packet')
        cid,dlc,payload=FRAME.unpack(data);kind=(cid>>24)&31
        if cid&0x60000000:raise RuntimeError('CAN error/RTR frame')
        if cid&EFF and kind not in (2,17,21):
            raise RuntimeError('Unexpected controller traffic; exclusive ownership lost')
        return data
    def close(self):self.sock.close()
    def fileno(self):return self.sock.fileno()


class PlannerMotors(Motors):
    def __init__(self,folder,start):
        self.log=None;self.release_permitted=False;self.arming=False;self.enable_attempted=False
        self.expected=np.asarray(start,dtype=float);self.start_pose=self.expected.copy();self.query_at=0.
        self.queries=frozenset(request_frame(i) for i in range(1,9))
        try:
            self.log=(folder/'execution_can_audit.jsonl').open('x')
            super().__init__('right',control_gripper=True)
            self.sockets={side:GateSocket(sock,side,self) for side,sock in self.sockets.items()}
        except BaseException:
            if self.log:self.log.close()
            raise

    def audit(self,event,side,frame,**kw):
        self.log.write(json.dumps(dict(monotonic=time.monotonic(),event=event,side=side,
                                      packet_hex=frame.hex(),**kw))+'\n');self.log.flush()

    def command_allowed(self,frame):
        if not isinstance(frame,bytes) or len(frame)!=FRAME.size:return False
        cid,dlc,data=FRAME.unpack(frame);i=cid&255
        if dlc!=8 or i not in range(1,9) or cid!=(EFF|(((cid>>24)&31)<<24)|0xfd00|i):return False
        if frame==packet(4,i):return self.release_permitted
        if frame==packet(3,i):return self.arming
        if frame==packet(17,i,bytes.fromhex('0570000000000000')):return self.arming
        if self.arming and frame in (parameter(i,0x7005,5,True),parameter(i,0x7017,SPEED),
            parameter(i,0x7018,CURRENT[i-1] if i<=7 else GRIPPER_CURRENT)):
            return True
        if i<=7 and self.active and frame==parameter(i,0x7017,RIGHT_PLAYBACK_SPEED):return True
        target=joint_to_motor('right',i,self.expected[i-1]) if i<=7 else -RIGHT_GRIPPER_CLOSED
        # Float-serialized exact target; gripper can ONLY receive +7 degrees.
        return (self.arming or self.active) and frame==parameter(i,0x7016,target)

    def _send(self,side,frame):
        if frame in [packet(3,i) for i in range(1,9)]:self.enable_attempted=True
        return super()._send(side,frame)

    def poll(self):
        now=time.monotonic()
        if now-self.query_at>=.01:self.last_query=now-.051;self.query_at=now
        else:self.last_query=now
        return super().poll()

    def arm_at_start(self,start):
        self.expected=np.asarray(start,dtype=float).copy()
        if not np.array_equal(self.expected,self.start_pose):raise RuntimeError('Startup target changed')
        self.arming=True
        try:
            # Non-confirm variant configures ALL drives at this SAME pose before
            # enabling any; unlike confirm/retry it never sends an initial disable.
            for _ in super().center(target=start,gripper_target=RIGHT_GRIPPER_CLOSED,confirm_enabled=False):
                self.poll()
                q=np.array([self.states['right',i][0] for i in range(1,8)])
                if np.sum(abs(q-self.start_pose))>.01:raise RuntimeError('Arm shifted during enable; no relax')
                time.sleep(.002)
            deadline=time.monotonic()+.6
            while True:
                self.poll()
                if self.fresh() and all(self.states['right',i][1]==2 for i in range(1,9)):break
                if time.monotonic()>deadline:raise RuntimeError('Enable was not confirmed; no retry/disable')
                time.sleep(.002)
            self.set_right_arm_speed(RIGHT_PLAYBACK_SPEED)
        finally:self.arming=False

    def set_positions(self,joints):
        q=np.asarray(joints,dtype=float)
        if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Invalid joint target')
        self.expected=q.copy()
        return super().set_positions(q)

    def release_at_verified_rest(self,start):
        self.poll()
        if not self.fresh() or not np.array_equal(np.asarray(start),self.start_pose):
            raise RuntimeError('No verified starting pose for relaxation')
        q=np.array([self.states['right',i][0] for i in range(1,8)])
        if np.max(abs(q-self.start_pose))>np.radians(.2):raise RuntimeError('Not at saved rest; refuse relax')
        if any(self.states['right',i][1]!=2 for i in range(1,9)):raise RuntimeError('Motor stopped before release')
        self.release_permitted=True
        try:
            for i in range(1,9):self.send_control(packet(4,i))
            self.active=False
            deadline=time.monotonic()+.5
            while True:
                self.poll()
                if self.fresh() and all(self.states['right',i][1]==0 for i in range(1,9)):break
                if time.monotonic()>deadline:raise RuntimeError('Relax feedback not confirmed; support arm')
                time.sleep(.005)
        finally:self.release_permitted=False

    def relax(self):raise RuntimeError('Only verified supported-start release is available')

    def close(self):
        # Inherited close only closes sockets/locks. Never invokes relax().
        try:
            if hasattr(self,'sockets'):super().close()
        finally:
            if self.log:self.log.close()
