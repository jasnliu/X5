"""Single physical cycle. A controlled cancellation returns to center before disable."""
from collections import deque
import csv
from datetime import datetime,timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid
import numpy as np
from centering.motors import RIGHT_GRIPPER_CLOSED
from .hardware import AuditedMotors, Video, notify_motion
from .trajectory import ROOT,SOURCE,transition

POSE_TOLERANCE=math.radians(.20)
SETTLE_RANGE=math.radians(.12)
SETTLE_SECONDS=.6


class Cancelled(Exception):pass


class Runner:
    def __init__(self,g,trajectory,directory,gripper=True,video=True):
        self.g=g;self.trajectory=trajectory;self.directory=Path(directory)
        self.gripper=gripper;self.want_video=video;self.bus=None;self.video=None
        self.phase='PREPARED';self.cancel=False;self.recovering=False;self.enabled=False
        self.last_goal=None;self.last_geometry=0.;self.last_log=0.
        self.events=[];self.endpoint=None;self.center_evidence=None;self.disabled=False
        self.command_rows=[]
        self.trace_file=open(self.directory/'trace.csv','w')
        self.trace=csv.writer(self.trace_file)
        self.trace.writerow(['monotonic_s','phase','trajectory_time_s','lateness_s']+[f'q{i}' for i in range(1,8)]+[f'target{i}' for i in range(1,8)])

    def event(self,phase,**data):
        self.phase=phase
        if self.bus:self.bus.phase=phase
        item={'t':time.monotonic(),'phase':phase,**data};self.events.append(item)
        with open(self.directory/'events.jsonl','a') as f:f.write(json.dumps(item)+'\n')
        print(phase, json.dumps(data) if data else '',flush=True)

    def signal(self,number,frame):
        self.cancel=True
        print('Stop requested: returning to center before relaxing. Do not kill the process.',flush=True)

    def positions(self):return np.array([self.bus.states['right',i][0] for i in range(1,8)])

    def poll(self, check_running=True):
        self.bus.poll()
        if not self.bus.fresh():raise RuntimeError('Fresh feedback unavailable')
        if self.enabled and check_running:
            if any(self.bus.states['right',i][1]!=2 for i in range(1,9 if self.gripper else 8)):
                raise RuntimeError('A controlled right motor stopped')
            if self.gripper and abs(self.bus.states['right',8][0]-RIGHT_GRIPPER_CLOSED)>math.radians(1.):
                raise RuntimeError('Gripper did not maintain closed position')
        q=self.positions()
        if time.monotonic()-self.last_geometry>.05:
            self.g.check(q,measured=True);self.last_geometry=time.monotonic()
        return q

    def cancellation(self):
        if self.cancel and not self.recovering:raise Cancelled('User requested controlled center + relax')

    def tick(self,desired,t,lateness=0.):
        self.cancellation()
        q=self.poll()
        error=float(np.max(np.abs(q-desired)))
        if error>math.radians(8.):raise RuntimeError(f'Tracking error {math.degrees(error):.3f} deg > 8 deg')
        self.bus.set_positions(desired);self.last_goal=np.array(desired)
        self.trace.writerow([time.monotonic(),self.phase,t,lateness,*q,*desired])

    def stream(self,duration,at,hz=100):
        start=time.monotonic();index=0;period=1./hz
        while True:
            deadline=start+index*period
            while time.monotonic()<deadline:
                self.cancellation();self.poll()
                time.sleep(min(.001,max(0.,deadline-time.monotonic())))
            now=time.monotonic();elapsed=now-start
            # Never send a burst of stale targets after a scheduling pause.
            if now-deadline>.08:raise RuntimeError('Command scheduler missed 80 ms deadline')
            self.tick(at(min(elapsed,duration)),min(elapsed,duration),max(0.,now-deadline))
            if elapsed>=duration:break
            index=max(index+1,int((time.monotonic()-start)/period)+1)

    def settle(self,goal,label):
        self.event(label);start=time.monotonic();history=deque();next_command=0.
        while True:
            self.cancellation();q=self.poll();now=time.monotonic()
            if now>=next_command:
                self.bus.set_positions(goal);self.last_goal=np.array(goal);next_command=now+.02
                self.trace.writerow([now,self.phase,0,0,*q,*goal])
            history.append((now,q.copy()))
            while history and now-history[0][0]>SETTLE_SECONDS+.03:history.popleft()
            worst=float(np.max(np.abs(q-goal)))
            if len(history)>1 and now-history[0][0]>=SETTLE_SECONDS:
                values=np.array([p for _,p in history])
                if np.max(np.abs(values-goal))<=POSE_TOLERANCE and np.max(np.ptp(values,axis=0))<=SETTLE_RANGE:
                    return dict(t=now,goal_rad=np.asarray(goal).tolist(),actual_rad=q.tolist(),
                                max_error_deg=math.degrees(worst),settle_range_deg=math.degrees(float(np.max(np.ptp(values,axis=0)))),
                                settled_seconds=now-history[0][0])
            if now-start>12:raise RuntimeError(f'{label}: did not settle within 0.20 degrees; error={math.degrees(worst):.3f}')
            time.sleep(.002)

    def move(self,target,label):
        self.event('CHECKING '+label)
        q=self.poll();self.g.check_line(q,target,measured_start=True)
        duration,at=transition(q,target)
        self.event(label,duration_s=duration)
        self.stream(duration,at)
        return self.settle(target,label+' SETTLED')

    def recover_passive_sag(self):
        initial=self.positions();legal=self.g.check_relaxed_start(initial)
        if np.max(np.abs(initial-legal))<=3*25.14/65535:return
        self.event('INWARD STARTUP RECOVERY',initial_rad=initial.tolist(),legal_rad=legal.tolist())
        self.bus.set_positions(legal);self.last_goal=legal.copy()
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            self.bus.poll();q=self.positions()
            if not self.bus.fresh() or any(self.bus.states['right',i][1]!=2 for i in range(1,8)):
                raise RuntimeError('Inward recovery lost running/fresh feedback')
            if np.any(q<np.minimum(initial,self.g.lower)-3*25.14/65535) or np.any(q>np.maximum(initial,self.g.upper)+3*25.14/65535):
                raise RuntimeError('Passive sag recovery moved farther outward')
            if np.all(q>=self.g.lower-3*25.14/65535) and np.all(q<=self.g.upper+3*25.14/65535):
                self.g.check(q,measured=True);return
            time.sleep(.002)
        raise RuntimeError('Inward startup recovery did not reach modeled limits')

    def centered_relax(self):
        self.bus.poll()
        if not self.bus.fresh() or any(self.bus.states['right',i][1]!=2 for i in range(1,9 if self.gripper else 8)):
            raise RuntimeError('Cannot recenter without fresh feedback and every selected drive running')
        self.enabled=True
        self.bus.set_right_arm_speed(.4)
        self.center_evidence=self.move(self.g.center,'RECENTER')
        # Center permission is issued only after fresh measured, settled arrival.
        self.bus.center_permission=True
        self.event('RELAX',center_evidence=self.center_evidence)
        self.bus.relax();self.enabled=False
        time.sleep(.025);deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            self.bus.poll()
            if all(s[1]==0 for s in self.bus.states.values()) and self.bus.fresh():
                self.disabled=True;self.event('RELAXED VERIFIED');return
            time.sleep(.005)
        raise RuntimeError('Disable confirmation failed')

    def run(self):
        error=None;old_handlers={}
        try:
            for sig in (signal.SIGINT,signal.SIGTERM):
                old_handlers[sig]=signal.signal(sig,self.signal)
            # This notification is mandatory and happens before any motor setup.
            notify_motion();self.event('NTFY ACCEPTED')
            for n in range(10,0,-1):
                self.cancellation();print(f'Movement in {n}s',flush=True);time.sleep(1)
            self.bus=AuditedMotors(self.directory,self.gripper)
            deadline=time.monotonic()+3
            while not self.bus.fresh():
                self.bus.poll()
                if time.monotonic()>deadline:raise RuntimeError('Preflight encoder timeout')
                time.sleep(.005)
            if any(s[1]!=0 for s in self.bus.states.values()):raise RuntimeError('Both arms must initially be relaxed')
            self.g.check_relaxed_start(self.positions())
            self.cancellation()
            if self.want_video:
                self.video=Video(self.directory,lambda:self.phase);self.video.start()
            # Camera startup and geometric validation can outlast the 300 ms
            # freshness window. Obtain new state replies before safe enabling.
            refresh_until=time.monotonic()+.12
            while time.monotonic()<refresh_until:
                self.bus.poll();time.sleep(.002)
            self.cancellation()
            self.event('ENABLE AT LIVE POSE',gripper_closed=self.gripper)
            # A cancellation during setup is deferred until all selected joints
            # have confirmed safe CSP holds; never disable a partially enabled arm.
            for _ in self.bus.center(self.g.center,RIGHT_GRIPPER_CLOSED if self.gripper else None,confirm_enabled=True):
                self.bus.poll();time.sleep(.002)
            deadline=time.monotonic()+3
            while self.gripper and abs(self.bus.states['right',8][0]-RIGHT_GRIPPER_CLOSED)>math.radians(.75):
                self.bus.poll()
                if time.monotonic()>deadline:raise RuntimeError('Gripper closure timeout')
                time.sleep(.005)
            self.enabled=True
            self.recover_passive_sag()
            self.move(self.g.center,'CENTER')
            self.move(self.trajectory.first,'APPROACH START')
            self.bus.set_right_arm_speed(self.trajectory.metadata['firmware_speed_rad_s'])
            self.event('PLAYBACK',method=self.trajectory.name,duration_s=self.trajectory.duration)
            self.stream(self.trajectory.duration,self.trajectory.at,self.trajectory.metadata['command_hz'])
            self.endpoint=self.settle(self.trajectory.last,'ENDPOINT HOLD')
            self.event('ENDPOINT VERIFIED',**self.endpoint)
            self.centered_relax()
        except BaseException as exc:
            error=f'{type(exc).__name__}: {exc}';self.event('ERROR',error=error)
            if self.bus and self.bus.active and not self.disabled:
                self.recovering=True
                try:
                    self.event('CONTROLLED RECOVERY')
                    self.centered_relax()
                except BaseException as recovery:
                    # Keep the drive's last position targets; never blindly disable
                    # or blindly move after lost feedback/faults. Operator action is
                    # required if the controlled center return cannot be completed.
                    self.event('RECOVERY FAILED - NOT RELAXED',error=str(recovery),last_goal=None if self.last_goal is None else self.last_goal.tolist())
                    print('ATTENTION: center return failed. NO RELAX command was sent. Drives may still hold. Inspect before any further operation.',flush=True)
        finally:
            if self.video:self.video.close()
            if self.bus:self.bus.close()
            self.trace_file.close()
            for sig,handler in old_handlers.items():signal.signal(sig,handler)
            summary=dict(backend='physical',method=self.trajectory.name,trajectory=self.trajectory.metadata,
                         success=error is None and self.disabled,error=error,endpoint=self.endpoint,
                         center=self.center_evidence,relaxed_verified=self.disabled,
                         camera_error=self.video.error if self.video else None,
                         video_frames=self.video.count if self.video else 0,
                         gripper='closed' if self.gripper else 'untouched',
                         source_sha256_after=hashlib.sha256(SOURCE.read_bytes()).hexdigest())
            (self.directory/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        return summary


def new_session(method):
    path=ROOT/'playback_results'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+method+'-'+uuid.uuid4().hex[:6])
    path.mkdir(parents=True)
    sources=list((ROOT/'smooth_playback').glob('*.py'))+[ROOT/'playback.sh',ROOT/'centering/motors.py',ROOT/'safe_zone/encoder.py',ROOT/'safe_zone/geometry.py',ROOT/'camera_playback/trajectory.py',SOURCE,ROOT/'model/openarmx.urdf',ROOT/'right_zones/zone1.json']
    hashes={}
    for p in sources:
        rel=p.relative_to(ROOT);dest=path/'source'/rel;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(p,dest);hashes[str(rel)]=hashlib.sha256(p.read_bytes()).hexdigest()
    (path/'source_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    state=subprocess.run(['ip','-j','-details','-statistics','link','show'],capture_output=True,text=True,check=True)
    links=[x for x in json.loads(state.stdout) if x.get('ifname') in ('can0','can1')]
    (path/'can_before.json').write_text(json.dumps(links,indent=2)+'\n')
    if len(links)!=2 or any(x.get('linkinfo',{}).get('info_data',{}).get('state')!='ERROR-ACTIVE' or x['linkinfo']['info_data'].get('bittiming',{}).get('bitrate')!=1000000 for x in links):
        raise RuntimeError('Both CAN interfaces must be ERROR-ACTIVE at 1 Mbit/s')
    return path
