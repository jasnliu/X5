"""Incremental arming adapter; original execution transport remains unchanged."""
import errno
import json
import time
from pathlib import Path
import numpy as np
from ..motor_transport import PlannerMotors,GateSocket
from centering.motors import Motors,packet,parameter,CURRENT,GRIPPER_CURRENT,SPEED,RIGHT_PLAYBACK_SPEED
from safe_zone.encoder import FRAME,EFF,request_frame,joint_to_motor
from ..common import arm_side
from .arms import closed_gripper


class HealthSocket(GateSocket):
    def send(self,frame):
        # Local selected-arm gate: the legacy right-only execution adapter is
        # unchanged. Even bypassing _send cannot write to the passive bus.
        if frame not in self.owner.queries:
            if self.side!=self.owner.control_side or not self.owner.command_allowed(frame):
                raise RuntimeError('Planner wire gate rejected actuator packet')
        self.owner.audit('tx_attempt',self.side,frame)
        n=self.sock.send(frame);self.owner.audit('tx_sent',self.side,frame,bytes_sent=n)
        if n!=FRAME.size:raise RuntimeError('Short CAN transmission')
        return n

    def recv(self,n):
        data=super().recv(n)
        cid,dlc,payload=FRAME.unpack(data);kind=(cid>>24)&31;i=(cid>>8)&255
        if cid&EFF and kind==2 and dlc==8 and i in range(1,9):
            self.owner.health[self.side,i]=dict(stamp=time.monotonic(),fault=(cid>>16)&63,
                temperature=int.from_bytes(payload[6:8],'big')*.1)
        return data


class BaselineMotors(PlannerMotors):
    def __init__(self,folder,start,*,moving_side='right'):
        side=arm_side(moving_side)
        self.health={};self.failed_at=None;self.audit_error='';self.audit_rotations=0
        self.audit_bytes=0;self.folder=Path(folder)
        self.gripper_closed=closed_gripper(side)
        if side=='right':
            super().__init__(folder,start)
            self.sockets={s:HealthSocket(gate.sock,s,self) for s,gate in self.sockets.items()}
        else:
            # The shared centering helper deliberately disallows generic left
            # gripper commands. Keep it unchanged: this local adapter permits
            # exactly the established custom LEFT_GRIPPER_TARGET, never opening.
            self.log=None;self.release_permitted=False;self.arming=False;self.enable_attempted=False
            self.expected=np.asarray(start,dtype=float);self.start_pose=self.expected.copy();self.query_at=0.
            self.queries=frozenset(request_frame(i) for i in range(1,9))
            try:
                self.log=(self.folder/'execution_can_audit.jsonl').open('x')
                Motors.__init__(self,'left',control_gripper=False)
                self.control_gripper=True
                self.sockets={s:HealthSocket(sock,s,self) for s,sock in self.sockets.items()}
            except BaseException:
                if self.log:self.log.close()
                raise

    def command_allowed(self,frame):
        if self.control_side=='right':return super().command_allowed(frame)
        if not isinstance(frame,bytes) or len(frame)!=FRAME.size:return False
        cid,dlc,data=FRAME.unpack(frame);i=cid&255
        if dlc!=8 or i not in range(1,9) or cid!=(EFF|(((cid>>24)&31)<<24)|0xfd00|i):return False
        if frame==packet(4,i):return self.release_permitted
        if frame==packet(3,i):return self.arming
        if frame==packet(17,i,bytes.fromhex('0570000000000000')):return self.arming
        if self.arming and frame in (parameter(i,0x7005,5,True),parameter(i,0x7017,SPEED),
            parameter(i,0x7018,CURRENT[i-1] if i<=7 else GRIPPER_CURRENT)):return True
        if i<=7 and self.active and frame==parameter(i,0x7017,RIGHT_PLAYBACK_SPEED):return True
        target=joint_to_motor('left',i,self.expected[i-1]) if i<=7 else -self.gripper_closed
        return (self.arming or self.active) and frame==parameter(i,0x7016,target)

    def _send(self,side,frame):
        if self.control_side=='right':return super()._send(side,frame)
        if side!=self.control_side:raise RuntimeError(f'{side.capitalize()} arm is query-only')
        if not self.command_allowed(frame):raise RuntimeError('Invalid left-motor command')
        if frame in tuple(packet(3,i) for i in range(1,9)):self.enable_attempted=True
        # Same bounded write retries as the proven centering transport. The
        # selected-arm gate above admits only CSP, exact targets and rest release.
        for attempt in range(6):
            try:
                if self.sockets[side].send(frame)==FRAME.size:return
                raise RuntimeError('Short CAN write')
            except OSError as exc:
                if exc.errno not in (errno.EAGAIN,errno.ENOBUFS) or attempt==5:raise
                time.sleep(.002)

    def audit(self,event,side,frame,**kw):
        # Bounded 4 x 8 MiB raw wire history; long operator holds cannot fill
        # RAM/disk indefinitely. A disk error is diagnostic only: it must not
        # stop healthy playback or prevent a measured-pose hold/return.
        try:
            line=json.dumps(dict(monotonic=time.monotonic(),event=event,side=side,packet_hex=frame.hex(),**kw))+'\n'
            if self.audit_bytes+len(line)>8*1024*1024:
                self.log.close();base=self.folder/'execution_can_audit.jsonl'
                Path(str(base)+'.3').unlink(missing_ok=True)
                for i in (2,1):
                    source=Path(str(base)+'.'+str(i))
                    if source.exists():source.rename(str(base)+'.'+str(i+1))
                base.rename(str(base)+'.1');self.log=base.open('w');self.audit_bytes=0;self.audit_rotations+=1
            self.log.write(line);self.log.flush();self.audit_bytes+=len(line);self.audit_error=''
        except Exception as e:self.audit_error=str(e)

    def poll(self):
        try:super().poll()
        except Exception:
            self.failed_at=time.monotonic();raise
        # After a rejected fault/error frame, OLD healthy cached feedback is
        # not enough to claim that a hold/recovery command is safe to send.
        if self.failed_at is not None:
            if len(self.health)!=16 or any(v['stamp']<=self.failed_at or v['fault'] or v['temperature']>=65 for v in self.health.values()):
                raise RuntimeError('Waiting for complete fresh fault-free feedback after a bus/drive error; no motion commands')
            self.failed_at=None

    def begin_arm(self,start):
        self.arming=True
        try:
            # All drives configured at saved CURRENT pose before any enable.
            # Yield after each packet, so Hold can interrupt arming too.
            if self.control_side=='right':
                yield from Motors.center(self,target=start,gripper_target=self.gripper_closed,confirm_enabled=False)
            else:
                yield from self._left_start(start)
            while not self.fresh() or not all(self.states[self.control_side,i][1]==2 for i in range(1,9)):yield
            if self.control_side=='right':self.set_right_arm_speed(RIGHT_PLAYBACK_SPEED)
            else:
                for i in range(1,8):self.send_control(parameter(i,0x7017,RIGHT_PLAYBACK_SPEED))
        finally:self.arming=False

    def _left_start(self,start):
        start=np.asarray(start,dtype=float)
        if start.shape!=(7,) or not np.isfinite(start).all() or not np.array_equal(start,self.start_pose):
            raise RuntimeError('Startup target changed')
        if not self.fresh() or any(v[1]!=0 for v in self.states.values()):
            raise RuntimeError('Both arms and grippers must be relaxed with fresh encoder readings')
        self.active=True
        goals=list(start)+[self.gripper_closed]
        # Same all-configured-before-any-enable sequence as right playback.
        for i,q in enumerate(goals,1):
            target=joint_to_motor('left',i,q) if i<=7 else -q
            for frame in (parameter(i,0x7005,5,True),parameter(i,0x7017,SPEED),
                parameter(i,0x7018,CURRENT[i-1] if i<=7 else GRIPPER_CURRENT),parameter(i,0x7016,target)):
                self.send_control(frame);yield
            self.modes.pop(i,None)
            self.send_control(packet(17,i,bytes.fromhex('0570000000000000')))
            deadline=time.monotonic()+.3
            while i not in self.modes:
                if time.monotonic()>deadline:raise RuntimeError(f'Motor {i}: position-mode readback missing')
                yield
            if self.modes[i][0]!=5:raise RuntimeError(f'Motor {i}: position mode not accepted')
        for i,q in enumerate(goals,1):
            self.send_control(packet(3,i));yield
            self.send_control(parameter(i,0x7016,joint_to_motor('left',i,q) if i<=7 else -q));yield

    def arm_at_start(self,start):
        # Prevent accidental entry into the inherited right-only blocking API.
        raise RuntimeError('Use the selected-arm tick-driven begin_arm API')

    def release_at_verified_rest(self,start):
        if self.control_side=='right':return super().release_at_verified_rest(start)
        self.poll()
        if not self.fresh() or not np.array_equal(np.asarray(start),self.start_pose):
            raise RuntimeError('No verified starting pose for relaxation')
        q=np.array([self.states['left',i][0] for i in range(1,8)])
        if np.max(abs(q-self.start_pose))>np.radians(.2):raise RuntimeError('Not at saved rest; refuse relax')
        if any(self.states['left',i][1]!=2 for i in range(1,9)):raise RuntimeError('Motor stopped before release')
        self.release_permitted=True
        try:
            for i in range(1,9):self.send_control(packet(4,i))
            self.active=False;deadline=time.monotonic()+.5
            while True:
                self.poll()
                if self.fresh() and all(self.states['left',i][1]==0 for i in range(1,9)):break
                if time.monotonic()>deadline:raise RuntimeError('Relax feedback not confirmed; support arm')
                time.sleep(.005)
        finally:self.release_permitted=False

    def release_on_operator_request(self):
        """Explicit Relax click: disable selected joints AND gripper at any pose.

        No targets, centering, opening, enable, or opposite-arm commands. Do not
        require healthy pose feedback before an operator-requested torque off.
        The controller separately confirms fresh disabled feedback afterward.
        """
        errors=[];self.release_permitted=True
        try:
            for i in range(1,9):
                try:self.send_control(packet(4,i))
                except Exception as e:errors.append(f'{getattr(self,"control_side","right")} motor {i}: {e}')
        finally:
            self.release_permitted=False;self.active=False
        if errors:raise RuntimeError('Relax transmission incomplete: '+'; '.join(errors))
