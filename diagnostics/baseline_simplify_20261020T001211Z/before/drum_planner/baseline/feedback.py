"""Display-only encoder stream. Exact queries, bounded storage, no motor API.

Idle: one background reader owns the normal exclusive CAN locks. It is stopped
and joined before capture or controller ownership. Active: the GUI consumes the
existing owner's feedback instead; it never opens competing CAN sockets.
"""
from pathlib import Path
import threading
import time

import numpy as np

from ..common import JOINTS,LEFT_JOINTS,atomic_json
from ..read_only import ReadOnlyObserver

STALE_SECONDS=.5
DISPLAY_JOINTS=JOINTS+LEFT_JOINTS+['openarmx_left_finger_joint1']


def movement_error(anchor,pose):
    """Stationary-pose guard, NOT a moving-servo tracking tolerance."""
    if anchor is None:return ''
    for side,names in [('Right',JOINTS),('Left',LEFT_JOINTS)]:
        delta=np.abs([pose['joints'][j]-anchor['joints'][j] for j in names])
        if delta.sum()>.01 or delta.max()>np.radians(.35):
            return f'{side} arm moved since Calculate ({np.degrees(delta.max()):.2f}° on one joint). Plan discarded. Click Calculate again and leave the arms still.'
    name='openarmx_left_finger_joint1'
    if abs(pose['joints'][name]-anchor['joints'][name])>.001:
        return 'Left gripper moved since Calculate. Plan discarded. Click Calculate again.'
    return ''


class RollingAudit:
    """At most two 2 MiB files per stream, including across ownership handoffs."""
    def __init__(self,path,limit=2*1024*1024):
        self.path=Path(path);self.limit=limit;self.file=self.path.open('a')
        self.size=self.path.stat().st_size
    def write(self,line):
        size=len(line.encode())
        if self.size+size>self.limit:
            self.file.close();self.path.replace(str(self.path)+'.1')
            self.file=self.path.open('w');self.size=0
        self.file.write(line);self.size+=size
    def flush(self):self.file.flush()
    def close(self):self.file.close()


def normalized_pose(joints,stamp):
    values=np.asarray([joints[j] for j in DISPLAY_JOINTS],dtype=float)
    if not np.isfinite(values).all() or not np.isfinite(stamp):raise ValueError('Invalid display feedback')
    if not 0<=values[-1]<=.044:raise ValueError('Invalid measured left gripper opening')
    # The right custom claw/stick are fixed URDF attachments. Do not invent
    # an opening transform for their rubber-band-constrained mechanism.
    return dict(joints=dict(zip(DISPLAY_JOINTS,values.tolist())),stamp=float(stamp))


def owner_pose(states):
    """Serialize MEASURED encoder states, never controller targets/path time."""
    if len(states)!=16:return None
    try:
        joints={name:states[side,i][0] for side,names in [('right',JOINTS),('left',LEFT_JOINTS)]
                for i,name in enumerate(names,1)}
        joints['openarmx_left_finger_joint1']=float(np.clip(.044*states['left',8][0]/1.0472,0,.044))
        return normalized_pose(joints,min(v[2] for v in states.values()))
    except (KeyError,TypeError,ValueError):return None  # invalid telemetry must not break the owner's fault/status channel


class DisplayTelemetry:
    """Latest sample only (no history queue); 20 Hz disabled-arm readback."""
    def __init__(self,folder,observer_factory=None):
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
        self.factory=observer_factory or (lambda folder:ReadOnlyObserver(folder,log_factory=RollingAudit))
        self.stop=threading.Event();self.lock=threading.Lock();self.latest=None
        self.error='';self.count=0;self.observer=None;self.audit=None
        self.anchor=None;self.guard_error=''
        self.thread=threading.Thread(target=self._run,name='baseline-display-queries',daemon=True)
        self.thread.start()
    def _run(self):
        try:
            self.observer=self.factory(self.folder)
            while not self.stop.is_set():
                began=time.monotonic();batch=self.observer.sample()
                pose=normalized_pose(batch['joints'],batch['start'])
                with self.lock:
                    self.latest=pose;self.count+=1
                    if not self.guard_error:self.guard_error=movement_error(self.anchor,pose)
                self.stop.wait(max(0.,.05-(time.monotonic()-began)))
        except Exception as exc:self.error=str(exc)
        finally:
            if self.observer:
                self.observer.close();self.audit=self.observer.report()
    def snapshot(self):
        with self.lock:return self.latest
    def set_guard(self,anchor):
        with self.lock:
            self.anchor=anchor;self.guard_error=movement_error(anchor,self.latest) if self.latest else ''
    def movement_error(self):
        with self.lock:return self.guard_error
    def close(self):
        self.stop.set();self.thread.join(timeout=2.)
        if self.thread.is_alive():raise RuntimeError('Display reader has not released CAN; no new owner started')
        atomic_json(self.folder/'display_query_report.json',dict(error=self.error,batches=self.count,
            history_capacity=1,audit_max_bytes=8*1024*1024,audit=self.audit,motor_commands=0))
