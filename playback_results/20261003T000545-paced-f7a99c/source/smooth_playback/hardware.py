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
import numpy as np
from centering.motors import Motors, packet, RIGHT_GRIPPER_CLOSED
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
        cid,_,_=FRAME.unpack(frame);kind=(cid>>24)&31;motor=cid&255
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
        for _ in range(3):
            for i in range(1,9 if self.control_gripper else 8):self.send_control(packet(4,i))
        self.active=False

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
