"""Packet-level robot double. No device creation; real polling/setup above it.

Only requested replies and accepted motor-command replies are produced. Never
refresh every encoder on a GUI tick. Shared memory lets the real spawned path
controller use its own receiver while the GUI observes the same fake drives.
"""
import io
import multiprocessing as mp
import socket
import struct
import sys
import time
from types import SimpleNamespace
import numpy as np
from centering.motors import Motors,RIGHT_GRIPPER_CLOSED
from safe_zone.encoder import FRAME,EFF,joint_to_motor,encoder_to_joint
from camera_playback.left_hold import LEFT_GRIPPER_TARGET
from camera_playback.generated_stream import StreamMotors
from camera_playback.generated_can import FeedbackQueries
from camera_playback.playback_transport import RetrySocket


def forbid_devices():
    def audit(event,args):
        if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:
            raise AssertionError('Physical CAN forbidden in packet-level simulation')
        if event=='open' and isinstance(args[0],(str,bytes)):
            path=str(args[0])
            if any(path.startswith(p) for p in ('/dev/tty','/dev/video','/dev/snd','/dev/bus/usb')):
                raise AssertionError('Physical peripheral forbidden: '+path)
    sys.addaudithook(audit)


class PacketDevice:
    def __init__(self,rest,clock=time.monotonic):
        self.clock=clock;ctx=mp.get_context('spawn')
        self.shared=SimpleNamespace(
            q=ctx.Array('d',[v for side in ('left','right') for v in rest[side]]),
            grip=ctx.Array('d',[LEFT_GRIPPER_TARGET,RIGHT_GRIPPER_CLOSED]),
            state=ctx.Array('i',[0]*16),mode=ctx.Array('i',[5]*16),
            version=ctx.Array('Q',[0]*16),reply_at=ctx.Array('d',[0.]*16),
            reply_q=ctx.Array('d',[0.]*16),reply_state=ctx.Array('i',[0]*16),
            ready_version=ctx.Array('Q',[0]*16),ready_q=ctx.Array('d',[0.]*16),
            ready_state=ctx.Array('i',[0]*16),lock=ctx.RLock())
        self.calls=[];self.drop=set();self.reply_delay=.001
        self.sockets={s:PacketSocket(self,s) for s in ('left','right')}

    def reply(self,key):
        if key in self.drop:return
        s=self.shared;now=self.clock()
        with s.lock:
            # A later in-flight reply must not erase an already delivered but
            # unread reply. Real CAN receivers retain those queued frames.
            if s.reply_at[key]<=now:
                s.ready_version[key]=s.version[key]
                s.ready_q[key]=s.reply_q[key];s.ready_state[key]=s.reply_state[key]
            side,i=divmod(key,8)
            s.reply_q[key]=s.q[side*7+i] if i<7 else s.grip[side]
            s.reply_state[key]=s.state[key]
            s.reply_at[key]=now+self.reply_delay
            s.version[key]+=1

    def owner(self):
        # Exactly the real owner's state fields, without Observer/device init.
        b=Motors.__new__(Motors)
        b.control_side='right';b.control_gripper=True;b.active=False
        b.states={};b.modes={};b.sockets=self.sockets.copy();b.locks=[]
        b.isolated_motors=set();b.last_query=0.;b.right_joint7_query_period=None
        b.last_right_joint7_query=0.;b.right_joint7_session=b.left_joint6_session=None
        return b


class PacketSocket:
    def __init__(self,device,side):
        self.device=device;self.side=side;self.base=0 if side=='left' else 8
        self.seen=list(device.shared.version[:]);self.mode_replies=[];self.closed=False

    def send(self,frame):
        assert not self.closed
        cid,dlc,data=FRAME.unpack(frame);kind=(cid>>24)&31;i=cid&255
        assert dlc==8 and 1<=i<=8 and kind in (2,3,4,17,18)
        d=self.device;s=d.shared;key=self.base+i-1;now=d.clock()
        d.calls.append((now,self.side,kind,i,frame))
        if kind==2:d.reply(key)
        elif kind in (3,4):s.state[key]=2 if kind==3 else 0;d.reply(key)
        elif kind==17:
            assert data[:2]==b'\x05\x70'
            self.mode_replies.append((now+d.reply_delay,FRAME.pack(EFF|17<<24|i<<8,8,
                                      bytes.fromhex('05700000')+bytes([s.mode[key],0,0,0]))))
        else:
            index=int.from_bytes(data[:2],'little')
            assert index in (0x7005,0x7016,0x7017,0x7018)
            if index==0x7005:s.mode[key]=data[4]
            elif index==0x7016:
                value=struct.unpack('<f',data[4:])[0]
                if i==8:s.grip[self.base//8]=-value
                else:s.q[self.base//8*7+i-1]=encoder_to_joint(self.side,i,value)
                if s.state[key]==2:d.reply(key)
        return FRAME.size

    def recv(self,n):
        d=self.device;s=d.shared;now=d.clock()
        if self.mode_replies and self.mode_replies[0][0]<=now:return self.mode_replies.pop(0)[1]
        for i in range(1,9):
            key=self.base+i-1
            with s.lock:
                if s.reply_at[key]<=now:
                    version,q,state=s.version[key],s.reply_q[key],s.reply_state[key]
                else:version,q,state=s.ready_version[key],s.ready_q[key],s.ready_state[key]
            if self.seen[key]>=version:continue
            self.seen[key]=version
            raw=joint_to_motor(self.side,i,q) if i<8 else -q
            encoded=round((raw+12.57)/25.14*65535)
            data=encoded.to_bytes(2,'big')+b'\x80\x00\x80\x00\x00\xc8'
            return FRAME.pack(EFF|2<<24|state<<22|i<<8,8,data)
        raise BlockingIOError

    def close(self):self.closed=True


class PacketStreamMotors(StreamMotors):
    """Real stream prime/poll/command code; only its sockets are replaced."""
    def __init__(self,directory,handles,side,special,shared):
        forbid_devices()
        d=PacketDevice.__new__(PacketDevice);d.clock=time.monotonic;d.shared=shared
        d.calls=[];d.drop=set();d.reply_delay=.001
        d.sockets={s:PacketSocket(d,s) for s in ('left','right')}
        self.__dict__.update(d.owner().__dict__)
        self.control_side=side;self.special=special;self.device=d
        self.sockets={s:RetrySocket(sock) for s,sock in d.sockets.items()}
        self.feedback_queries=FeedbackQueries(.005,side)
        self.command_file=io.StringIO()
    def close(self):
        for s in self.sockets.values():s.close()
        self.command_file.close()
