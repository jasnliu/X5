"""Right-arm-only CSP transport, fresh feedback capture and center-before-relax gate."""
from collections import deque
import csv
import errno
import importlib.util
import json
import math
from pathlib import Path
import threading
import time
import struct
import numpy as np
from centering.motors import Motors, packet, parameter, RIGHT_GRIPPER_CLOSED
from safe_zone.encoder import FRAME, EFF, request_frame, encoder_to_joint


def notify_motion():
    path=Path.home()/'.local/bin/codex-ntfy.py'
    spec=importlib.util.spec_from_file_location('playback_ntfy',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    if not module.post_ntfy('Robot movement starts in 10 seconds: center, play recording, return to center, then relax. Keep clear.',
                            'Robot playback starting', 'warning,robot'):
        raise RuntimeError('ntfy delivery failed: motion is blocked')
    module.play_sound('{"type":"agent-turn-complete"}')


class CaptureSocket:
    def __init__(self,sock,side,owner):self.sock=sock;self.side=side;self.owner=owner
    def __getattr__(self,name):return getattr(self.sock,name)
    def recv(self,n):
        frame=self.sock.recv(n)
        now=time.monotonic();cid,dlc,data=FRAME.unpack(frame)
        if cid&EFF and not cid&0x60000000 and (cid>>24)&31==17 and dlc==8 and data[:2]==b'\x1e\x70':
            self.owner.parameter_values[(cid>>8)&255]=(struct.unpack('<f',data[4:])[0],now)
        if cid&EFF and not cid&0x60000000 and (cid>>24)&31==2 and dlc==8:
            i=(cid>>8)&255
            if i in range(1,9):
                raw=int.from_bytes(data[:2],'big')/65535*25.14-12.57
                q=encoder_to_joint(self.side,i,raw) if i<8 else -raw
                self.owner.feedback.writerow([now,self.owner.phase,self.side,i,q,(cid>>22)&3,
                    int.from_bytes(data[2:4],'big'),int.from_bytes(data[4:6],'big'),int.from_bytes(data[6:8],'big')*.1])
        return frame


class AuditedMotors(Motors):
    def __init__(self,directory,gripper):
        self.phase='CONNECT';self.center_permission=False
        self.directory=directory;self.parameter_values={};self.gains_original={};self.gains_restored=True
        self.feedback_file=open(directory/'feedback.csv','w',buffering=1024*128)
        self.feedback=csv.writer(self.feedback_file)
        self.feedback.writerow(['monotonic_s','phase','side','motor','position_rad','state','velocity_raw','torque_raw','temperature_c'])
        self.command_file=open(directory/'commands.jsonl','w',buffering=1024*64)
        self.right_query=0.;self.left_query=0.
        try:
            super().__init__('right',control_gripper=gripper)
            self.sockets={side:CaptureSocket(s,side,self) for side,s in self.sockets.items()}
        except BaseException:
            self.feedback_file.close();self.command_file.close();raise

    def _send(self,side,frame):
        cid,_,data=FRAME.unpack(frame);kind=(cid>>24)&31;motor=cid&255
        if kind in (17,18) and data[:2]==b'\x1e\x70':
            if side!='right' or motor not in range(1,8):raise RuntimeError('Gain access is right-arm-only')
            if kind==18:
                value=struct.unpack('<f',data[4:])[0];original=self.gains_original.get(motor)
                if original is None or not math.isfinite(value) or not original*.125<=value<=original:
                    raise RuntimeError('Gain write outside saved-original 12.5–100% envelope')
            for attempt in range(6):
                try:
                    if self.sockets[side].send(frame)!=16:raise RuntimeError('Short parameter transfer')
                    break
                except OSError as exc:
                    if exc.errno not in (errno.EAGAIN,errno.ENOBUFS) or attempt==5:raise
                    time.sleep(.002)
            self.command_file.write(json.dumps(dict(t=time.monotonic(),phase=self.phase,side=side,kind=kind,motor=motor,frame=frame.hex()))+'\n')
            return
        if kind==4 and not self.center_permission:
            state=self.states.get(('right',motor))
            if state is None or state[1]!=0:
                raise RuntimeError('REFUSED disable: arm has not reached verified center')
        if side!='right':raise RuntimeError('Left-arm control prohibited')
        if motor==8 and not self.control_gripper:raise RuntimeError('Gripper is untouched')
        super()._send(side,frame)
        self.command_file.write(json.dumps(dict(t=time.monotonic(),phase=self.phase,side=side,kind=kind,motor=motor,frame=frame.hex()))+'\n')

    def poll(self):
        # Reuse existing fault/temperature/state decoding, but request right
        # feedback at 100 Hz for evidence. Left remains state-query-only, 20 Hz.
        self.last_query=time.monotonic()
        super().poll()
        now=time.monotonic()
        for side,period,attribute in (('right',.01,'right_query'),('left',.05,'left_query')):
            if now-getattr(self,attribute)<period:continue
            setattr(self,attribute,now)
            for i in range(1,9):
                for attempt in range(6):
                    try:
                        if self.sockets[side].send(request_frame(i))!=16:raise RuntimeError('Short state query')
                        break
                    except OSError as exc:
                        if exc.errno not in (errno.EAGAIN,errno.ENOBUFS) or attempt==5:raise
                        time.sleep(.001)

    def relax(self):
        if not self.center_permission:raise RuntimeError('Center verification is required before relaxation')
        if not self.gains_restored:raise RuntimeError('Original position gains must be verified restored before relaxation')
        for _ in range(3):
            for i in range(1,9 if self.control_gripper else 8):self.send_control(packet(4,i))
        self.active=False

    def set_playback_speeds(self,speeds):
        if len(speeds)!=7 or any(s not in (.4,.8) for s in speeds):
            raise ValueError('Only existing 0.4/0.8 CSP speed ceilings are permitted')
        if not self.active:raise RuntimeError('Arm must be active')
        for i,s in enumerate(speeds,1):self.send_control(parameter(i,0x7017,s))

    def read_position_gain(self,motor):
        self.parameter_values.pop(motor,None)
        self.send_control(packet(17,motor,struct.pack('<H6x',0x701e)))
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:
            self.poll()
            if motor in self.parameter_values:return self.parameter_values[motor][0]
            time.sleep(.001)
        raise RuntimeError(f'J{motor} position-gain readback missing')

    def apply_position_gain_factor(self,factor):
        if factor not in (.5,.75):raise ValueError('Only tested candidate gain factors are allowed')
        self._apply_position_gains(lambda value:value*factor)

    def apply_position_gain_cap(self,cap):
        if cap not in (10.,20.,30.):raise ValueError('Only bounded candidate gain caps are allowed')
        self._apply_position_gains(lambda value:min(value,cap))

    def _apply_position_gains(self,target_for):
        if not self.gains_restored:raise RuntimeError('Previous gains must be restored first')
        self.gains_original={i:self.read_position_gain(i) for i in range(1,8)}
        if any(not math.isfinite(v) or not 5<=v<=200 for v in self.gains_original.values()):
            raise RuntimeError('Unexpected original position gain; refusing change')
        targets={i:target_for(value) for i,value in self.gains_original.items()}
        if any(not self.gains_original[i]*.125<=v<=self.gains_original[i] for i,v in targets.items()):
            raise RuntimeError('Gain target outside saved-original 12.5–100% envelope')
        (self.directory/'gains_original.json').write_text(json.dumps(self.gains_original,indent=2)+'\n')
        (self.directory/'gains_temporary.json').write_text(json.dumps(targets,indent=2)+'\n')
        self.gains_restored=False
        for i,target in targets.items():
            self.send_control(parameter(i,0x701e,target))
            if abs(self.read_position_gain(i)-target)>1e-5:raise RuntimeError('Gain change readback mismatch')

    def restore_position_gains(self):
        if self.gains_restored:return
        for i,value in self.gains_original.items():
            self.send_control(parameter(i,0x701e,value))
            if abs(self.read_position_gain(i)-value)>1e-5:raise RuntimeError('Original gain restoration readback mismatch')
        self.gains_restored=True
        (self.directory/'gains_restored.json').write_text(json.dumps(self.gains_original,indent=2)+'\n')

    def close(self):
        # Deliberately NOT a relax call: a finally block cannot bypass the gate.
        super().close()
        self.feedback_file.close();self.command_file.close()


class Video:
    def __init__(self,directory,phase):
        self.directory=directory;self.phase=phase;self.stop=threading.Event();self.error=None
        self.ready=threading.Event();self.count=0
        self.thread=threading.Thread(target=self.run,daemon=True)
    def start(self):
        self.thread.start();self.ready.wait(5)
    def run(self):
        import cv2
        cap=cv2.VideoCapture(0,cv2.CAP_V4L2);writer=None
        try:
            if not cap.isOpened():raise RuntimeError('Camera unavailable')
            cap.set(cv2.CAP_PROP_FPS,30)
            with open(self.directory/'video_frames.csv','w') as f:
                out=csv.writer(f);out.writerow(['frame','monotonic_s','phase'])
                while not self.stop.is_set():
                    ok,frame=cap.read();now=time.monotonic()
                    if not ok:raise RuntimeError('Camera read failed')
                    if writer is None:
                        h,w=frame.shape[:2]
                        writer=cv2.VideoWriter(str(self.directory/'video.avi'),cv2.VideoWriter_fourcc(*'MJPG'),30,(w,h))
                        if not writer.isOpened():raise RuntimeError('Video writer unavailable')
                        self.ready.set()
                    writer.write(frame);out.writerow([self.count,now,self.phase()]);self.count+=1
        except Exception as e:self.error=str(e);self.ready.set()
        finally:
            cap.release()
            if writer:writer.release()
    def close(self):
        self.stop.set();self.thread.join(5)
        if self.thread.is_alive():self.error='Video thread did not exit promptly'
