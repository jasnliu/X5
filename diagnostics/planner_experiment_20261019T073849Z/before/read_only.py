"""Audited, disabled-motor telemetry only. No actuator control imports or API.

Reuse X5's existing encoder conversion, bus ownership locks and disabled-state
checks. An exact packet allow-list sits immediately before every socket send.
Closing this observer ONLY closes descriptors; it never disables/relaxes motors.
"""
from collections import deque
import json
from pathlib import Path
import threading
import time

import numpy as np

from safe_zone.encoder import Observer,FRAME,EFF,request_frame
from .common import JOINTS,LEFT_JOINTS

ALLOWED=frozenset(request_frame(m) for m in range(1,9))
BUSES={'right':'can0','left':'can1'}


class QuerySocket:
    def __init__(self,sock,bus,tx_log,rx_log):
        self._socket=sock;self.bus=bus;self.tx_log=tx_log;self.rx_log=rx_log
        self.requests={};self.feedback={};self.sent=0;self.received=0

    def send(self,frame):
        if not isinstance(frame,bytes) or frame not in ALLOWED:
            raise RuntimeError('Read-only hardware gate blocked a non-query CAN packet')
        motor=FRAME.unpack(frame)[0]&255;now=time.monotonic()
        event=dict(monotonic=now,bus=self.bus,motor=motor,packet_hex=frame.hex(),operation='state_request_only')
        # Log intent before sending, and outcome after; failures remain auditable.
        self.tx_log.write(json.dumps(dict(event,event='attempt'))+'\n');self.tx_log.flush()
        count=self._socket.send(frame)
        self.tx_log.write(json.dumps(dict(event,event='sent',bytes_sent=count))+'\n');self.tx_log.flush()
        if count!=FRAME.size:raise RuntimeError('Short state-query CAN write')
        self.requests[motor]=now;self.sent+=1
        return count

    def recv(self,size):
        frame=self._socket.recv(size);now=time.monotonic();self.received+=1
        self.rx_log.write(json.dumps(dict(monotonic=now,bus=self.bus,packet_hex=frame.hex()))+'\n')
        self.rx_log.flush()
        if len(frame)==FRAME.size:
            cid,dlc,data=FRAME.unpack(frame);kind=(cid>>24)&31;motor=(cid>>8)&255
            if cid&EFF and not cid&0x60000000:
                # No controller should share these reserved arm buses. Reject
                # even parameter/enable/disable traffic rather than ignoring it.
                if frame not in ALLOWED and kind not in (2,21):
                    raise RuntimeError(f'Unexpected non-feedback traffic on {self.bus}; stop other controllers')
                if kind==2 and motor in range(1,9) and dlc==8:
                    count=int.from_bytes(data[:2],'big')
                    self.feedback[motor]=dict(motor=motor,received_monotonic=now,
                        request_monotonic=self.requests.get(motor),encoder_count=count,
                        raw_rad=count/65535*25.14-12.57,mode=(cid>>22)&3,fault_bits=(cid>>16)&63,
                        temperature_c=int.from_bytes(data[6:8],'big')*.1)
        # Existing Observer.decode rejects malformed/error/RTR/fault/enabled frames.
        return frame

    def fileno(self):return self._socket.fileno()
    def close(self):self._socket.close()


class ReadOnlyObserver:
    def __init__(self,folder):
        self.folder=Path(folder);self.observer=None;self.tx_log=None;self.rx_log=None
        self.gates={};self.closed=False;self.batches=0
        try:
            self.tx_log=(self.folder/'can_query_audit.jsonl').open('x')
            self.rx_log=(self.folder/'can_rx_audit.jsonl').open('x')
            # Constructor takes the same two exclusive locks as existing X5
            # controllers. It opens sockets but sends nothing until sample().
            self.observer=Observer(left='can1',right='can0')
            for side,sock in self.observer.sockets.items():
                gate=QuerySocket(sock,BUSES[side],self.tx_log,self.rx_log)
                self.gates[side]=gate;self.observer.sockets[side]=gate
        except BaseException:
            self.close();raise

    def sample(self):
        start=time.monotonic();q=self.observer.sample();end=time.monotonic()
        feedback={}
        for side,gate in self.gates.items():
            feedback[side]=[]
            for motor in range(1,9):
                value=dict(gate.feedback[motor])
                request=value['request_monotonic'];received=value['received_monotonic']
                if request is None or not start<=request<=received<=end or end-received>.2:
                    raise RuntimeError(f'Stale {side} motor {motor} telemetry')
                if value['mode']!=0 or value['fault_bits']:
                    raise RuntimeError(f'{side} motor {motor} is not disabled and fault-free')
                if value['temperature_c']>=65:
                    raise RuntimeError(f'{side} motor {motor} temperature is at least 65 C')
                feedback[side].append(value)
        if not np.isfinite([q[k] for k in JOINTS+LEFT_JOINTS]).all():
            raise RuntimeError('Non-finite measured joint angles')
        self.batches+=1
        return dict(start=start,end=end,joints=dict(q),motor8_feedback=q.motor8_feedback,
                    feedback=feedback,all_16_motors_disabled=True)

    def report(self):
        return dict(batches=self.batches,requests_sent={side:gate.sent for side,gate in self.gates.items()},
                    feedback_frames_received={side:gate.received for side,gate in self.gates.items()},
                    exact_query_allowlist=True,mode_changes=0,enable_commands=0,disable_commands=0,
                    motion_commands=0,gripper_commands=0,closed=self.closed)

    def close(self):
        if self.closed:return
        try:
            if self.observer:self.observer.close()
        finally:
            for log in (self.tx_log,self.rx_log):
                if log:log.close()
            self.closed=True


class Telemetry:
    """Bounded history for synchronization plus an append-only full audit."""
    def __init__(self,folder,observer_factory=ReadOnlyObserver):
        self.folder=Path(folder);self.factory=observer_factory;self.error=None;self.count=0
        self.history=deque(maxlen=15000);self.lock=threading.Lock();self.stop=threading.Event()
        self.observer=None;self.first=None;self.last=None
        self.thread=threading.Thread(target=self._run,name='planner-query-only',daemon=True)
        self.thread.start()

    def _run(self):
        try:
            with (self.folder/'encoder_batches.jsonl').open('x') as log:
                self.observer=self.factory(self.folder)
                while not self.stop.is_set():
                    began=time.monotonic();sample=self.observer.sample()
                    log.write(json.dumps(sample)+'\n');log.flush()
                    with self.lock:
                        self.history.append(sample);self.count+=1
                        if self.first is None:self.first=sample['start']
                        self.last=sample['end']
                    self.stop.wait(max(0,.05-(time.monotonic()-began)))
        except BaseException as exc:
            self.error=str(exc) or type(exc).__name__
        finally:
            if self.observer:self.observer.close()

    def samples(self):
        with self.lock:return list(self.history)

    def close(self):
        self.stop.set();self.thread.join(timeout=3)
        if self.thread.is_alive():raise RuntimeError('Read-only telemetry thread did not stop')

    def report(self):
        return dict(sample_batches=self.count,error=self.error,thread_stopped=not self.thread.is_alive(),
                    duration_s=0 if self.first is None else self.last-self.first,
                    observer=None if self.observer is None else self.observer.report())


def select_stationary(samples,frame_time=None):
    """No interpolation: choose a complete measured batch in a quiet window.

    Host receive timestamps are NOT hardware synchronization. Require encoder
    batches on both sides of a camera timestamp and document the residual skew.
    """
    if len(samples)<15:raise ValueError('Need at least 15 complete disabled-motor batches')
    starts=np.array([p['start'] for p in samples]);ends=np.array([p['end'] for p in samples])
    if (not np.isfinite(starts).all() or not np.isfinite(ends).all() or
            np.any(ends<starts) or np.any(np.diff(starts)<=0) or np.any(starts[1:]<ends[:-1]) or
            ends[-1]-starts[0]<.75 or np.max(np.diff(starts))>.25 or np.max(ends-starts)>.2):
        raise ValueError('Encoder acquisition gaps or insufficient stationary observation')
    values=np.array([[s['joints'][name] for name in JOINTS+LEFT_JOINTS] for s in samples])
    if not np.isfinite(values).all():raise ValueError('Non-finite encoder data')
    spread=np.degrees(np.ptp(values,axis=0))
    if spread.max()>.35:raise ValueError(f'Arm moved {spread.max():.2f} degrees during the snapshot; no motor state was changed')
    if not all(p.get('all_16_motors_disabled') is True for p in samples):
        raise ValueError('All 16 motors must report disabled')
    times=(starts+ends)/2
    if frame_time is None:
        index=len(samples)-1;skew=None;bracket=None
    else:
        if not np.isfinite(frame_time):raise ValueError('Invalid camera timestamp')
        before=np.flatnonzero(ends<=frame_time);after=np.flatnonzero(starts>=frame_time)
        if not len(before) or not len(after):raise ValueError('Encoder batches do not bracket the camera frame')
        left=int(before[-1]);right=int(after[0])
        if frame_time-ends[left]>.2 or starts[right]-frame_time>.2:
            raise ValueError('Camera/encoder bracketing gap exceeds 200 ms')
        index=int(np.argmin(abs(times-frame_time)));skew=float(abs(times[index]-frame_time))
        if skew>.1:raise ValueError('Camera/encoder observation skew exceeds 100 ms')
        bracket=[left,right]
    evidence=dict(method='stationary disabled-motor batches; host receive timestamps, NOT hardware synchronization',
                  batch_count=len(samples),duration_s=float(ends[-1]-starts[0]),
                  maximum_batch_duration_s=float(np.max(ends-starts)),maximum_batch_start_gap_s=float(np.max(np.diff(starts))),
                  joint_spread_deg=dict(zip(JOINTS+LEFT_JOINTS,spread.tolist())),
                  camera_received_monotonic=frame_time,selected_batch_index=index,
                  nearest_midpoint_skew_s=skew,bracketing_indices=bracket)
    return samples[index],evidence
