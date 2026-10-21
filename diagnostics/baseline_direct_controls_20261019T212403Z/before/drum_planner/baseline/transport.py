"""Incremental arming adapter; original execution transport remains unchanged."""
import json
import time
from pathlib import Path
from ..motor_transport import PlannerMotors,GateSocket
from centering.motors import Motors,RIGHT_GRIPPER_CLOSED,RIGHT_PLAYBACK_SPEED
from safe_zone.encoder import FRAME,EFF


class HealthSocket(GateSocket):
    def recv(self,n):
        data=super().recv(n)
        cid,dlc,payload=FRAME.unpack(data);kind=(cid>>24)&31;i=(cid>>8)&255
        if cid&EFF and kind==2 and dlc==8 and i in range(1,9):
            self.owner.health[self.side,i]=dict(stamp=time.monotonic(),fault=(cid>>16)&63,
                temperature=int.from_bytes(payload[6:8],'big')*.1)
        return data


class BaselineMotors(PlannerMotors):
    def __init__(self,folder,start):
        self.health={};self.failed_at=None;self.audit_error='';self.audit_rotations=0
        self.audit_bytes=0;self.folder=Path(folder)
        super().__init__(folder,start)
        self.sockets={side:HealthSocket(gate.sock,side,self) for side,gate in self.sockets.items()}

    def audit(self,event,side,frame,**kw):
        # Bounded 4 x 8 MiB raw wire history; long operator holds cannot fill
        # RAM/disk indefinitely. A disk error stops progression but MUST NOT
        # prevent an emergency measured-pose hold packet from being sent.
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
            yield from Motors.center(self,target=start,gripper_target=RIGHT_GRIPPER_CLOSED,confirm_enabled=False)
            while not self.fresh() or not all(self.states['right',i][1]==2 for i in range(1,9)):yield
            self.set_right_arm_speed(RIGHT_PLAYBACK_SPEED)
        finally:self.arming=False
