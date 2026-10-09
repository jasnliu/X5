"""Finite snare/ride/hi-hat collection, adapted from the existing hi-hat collector."""
import argparse
from collections import deque
from dataclasses import replace
from datetime import datetime, timezone
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

import numpy as np

from camera_playback.hihat import DEFAULT_ESP_PORT
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET
from camera_playback.left_return import ReverseRecording
from camera_playback.recording_only import refresh_feedback
from centering.motors import Motors, RIGHT_GRIPPER_CLOSED
from smooth_playback.trajectory import transition
from snare_lab import runner as snare
from snare_lab.config import LEFT_RECORDING
from x5_collection.collect import Collector, preflight, dump, query_disabled
from x5_collection.hybrid import CollectionSession
from .control import HiHatSession
from .plan import make_plan, CLOSE_DEGREES, SNARE_DEGREES, SNARE_MAX_DEGREES
from .transport import DataMotors

ROOT = Path(__file__).resolve().parents[1]


def notify(directory, stage, text):
    spec = importlib.util.spec_from_file_location('data_ntfy', Path.home()/'.local/bin/codex-ntfy.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    delivered = module.post_ntfy(text, 'X5 data collection: motion warning', 'warning,robot')
    dump(directory/f'notification_{stage}.json',dict(delivered=bool(delivered),message=text,t=time.monotonic()))
    if not delivered: raise RuntimeError('ntfy delivery failed; no motion allowed')


def prepare_left():
    model, zone, lo, hi = snare.load_geometry()
    geometry = snare.left_geometry(model, zone, lo, hi)
    recording = snare.load_recording(LEFT_RECORDING, model, zone, lo, hi)
    parameters, rules, limits = snare.prepare_strike(geometry, recording, SNARE_DEGREES)
    limits = replace(limits, hard_depth_deg=SNARE_MAX_DEGREES)
    snare.validate_strike_path(geometry, recording, SNARE_MAX_DEGREES)
    # Prevalidate synchronized controlled center paths at all strike positions.
    for degrees in np.linspace(0., SNARE_MAX_DEGREES, 15):
        geometry.check_line(snare.dipped_target(recording, degrees), LEFT_CENTER, measured_start=True)
    return geometry, recording, (parameters, rules, limits)


class HiHatCollector(Collector):
    """Existing collector extended to a dual-arm, snare-targeted dataset."""
    def __init__(self, prepared, directory, plan, left_prepared):
        super().__init__(prepared, directory, plan)
        self.lg, self.left_recording, self.left_tuning = left_prepared
        self.hihat = None
        self.examples = []
        self.left_ready = False
        self.left_last_geometry = 0.
        self.left_recovering = False

    def check_audio(self):
        super().check_audio()
        if self.hihat is not None and not self.recovering:
            self.hihat.check()

    def poll(self, check_running=True):
        if self.recovering and not self.near_center_recovery:
            q = self.positions()
            if (np.max(np.abs(q[:6]-self.g.center[:6])) <= math.radians(.2)
                    and abs(q[6]-self.g.center[6]) <= math.radians(1.)):
                self.near_center_recovery = True
        q = super().poll(check_running)
        if self.hihat is not None and not self.recovering: self.hihat.check()
        if self.left_ready:
            if any(self.bus.states['left', i][1] != 2 for i in range(1, 9)):
                raise RuntimeError('Left motor stopped; refusing blind motion/relax')
            if abs(self.bus.states['left',8][0]-LEFT_GRIPPER_TARGET) > math.radians(1.):
                raise RuntimeError('Left gripper lost closed hold')
            if time.monotonic()-self.left_last_geometry > .05:
                self.lg.check(self.left_positions(), measured=True)
                self.left_last_geometry = time.monotonic()
        return q

    def left_positions(self):
        return np.array([self.bus.states['left', i][0] for i in range(1, 8)])

    def left_settle(self, goal, label, timeout=20.):
        self.event(label)
        history = deque(); started = time.monotonic(); last_command = 0.
        while True:
            self.cancellation(); self.poll(); now = time.monotonic()
            if now-last_command >= .02:
                self.bus.left_drive.set_positions(goal); last_command = now
            q = self.left_positions()
            history.append((now,q.copy()))
            while history and now-history[0][0] > .65: history.popleft()
            if history and now-history[0][0] >= .6:
                values = np.array([v for _,v in history])
                if np.max(np.abs(values-goal)) <= math.radians(.2) and np.max(np.ptp(values,axis=0)) <= math.radians(.12):
                    return dict(t=now,max_error_deg=float(np.degrees(np.max(np.abs(values-goal)))),
                                span_deg=float(np.degrees(np.max(np.ptp(values,axis=0)))),settled_s=now-history[0][0])
            if now-started > timeout: raise RuntimeError(label+': position did not settle')
            time.sleep(.002)

    def left_move(self, goal, label):
        q = self.left_positions()
        # Maintain both powered MIT holds while doing FK validation.
        for pose in np.linspace(q, goal, max(2,int(np.max(np.abs(goal-q))/math.radians(.25))+1)):
            self.lg.check(pose, measured=True); self.bus.poll()
        duration, at = transition(q, goal)
        self.event(label,duration_s=duration)
        self.left_stream(duration, lambda t:(at(t), t>=duration))
        return self.left_settle(goal,label+' SETTLED')

    def left_stream(self, duration, at):
        start = time.monotonic(); next_command = start
        while True:
            self.cancellation(); self.poll(); now = time.monotonic()
            if now >= next_command:
                goal, done = at(min(duration,now-start))
                if np.max(np.abs(self.left_positions()-goal)) > math.radians(8.):
                    raise RuntimeError('Left path tracking error above 8 degrees')
                self.bus.left_drive.set_positions(goal)
                if done: break
                next_command = now+.01
            time.sleep(.002)

    def play_left(self, reverse=False):
        recording = ReverseRecording(self.left_recording,self.left_recording.duration_s) if reverse else self.left_recording
        self.left_move(recording.first_joints,'LEFT REVERSE START' if reverse else 'LEFT RECORDING START')
        self.event('LEFT REVERSE RECORDING' if reverse else 'LEFT RECORDING')
        self.left_stream(recording.duration_s,recording.joints_at)
        self.left_settle(recording.last_joints,'LEFT ENDPOINT VERIFIED')

    def before_arm(self):
        self.hihat = HiHatSession(DEFAULT_ESP_PORT,self.directory)
        deadline=time.monotonic()+5.
        while self.hihat.check()['state']!='idle':
            self.check_audio(); self.cancellation()
            if time.monotonic()>deadline: raise RuntimeError('Hi-hat readiness timeout')
            time.sleep(.01)

    def collect_blocks(self):
        self.bus.mit_center_return = None
        self.hybrid = CollectionSession(self.bus,self.trajectory.last[6],self.lower_j7,self.upper_j7,self.tuning,self.directory)
        refresh_feedback(self.bus,.1)
        self.wait_hybrid(lambda s:s.ready)
        # 2 seconds of settled, quiet context before beginning the batch.
        self.wait_until(time.monotonic()+2.)
        for row in self.plan:
            self.check_audio()
            self.left_settle(self.left_recording.last_joints,'SNARE ANCHOR VERIFIED')
            begin = time.monotonic(); nominal = begin+2.
            if row['hihat_closures']:
                self.hihat.block(row['block'],nominal+row['hihat_offset_s'],1)
            schedule=[]
            for hit in range(row['snare_hits']): schedule.append((nominal+hit*1.2,'snare'))
            if row['ride_hits']: schedule.append((nominal+row['ride_offset_s'],'ride'))
            schedule.sort()
            request=None; snare_start_index=len(self.bus.snare_done)
            self.event('CAPTURING',**row)
            while time.monotonic()<begin+row['duration_s']:
                self.cancellation(); self.check_audio(); s=self.hybrid_tick()
                now=time.monotonic()
                if schedule and now>=schedule[0][0]:
                    due,kind=schedule.pop(0)
                    if now-due>.08: raise RuntimeError('Collection event missed 80 ms deadline')
                    if kind=='snare': self.bus.start_snare(row['block'])
                    else: request=self.hybrid.block(row['ride_depth_deg'],1,row['block'])
                    self.event('STRIKE REQUEST',block=row['block'],instrument=kind,due=due)
                time.sleep(.001)
            hits=self.bus.snare_done[snare_start_index:]
            if schedule or len(hits)!=row['snare_hits'] or self.bus.snare_method:
                raise RuntimeError('Snare block incomplete')
            if row['ride_hits'] and not(s.request_id>=request and s.ready and not s.swing and s.count==1):
                raise RuntimeError('Ride block incomplete')
            if row['hihat_closures'] and self.hihat.check()['completed']!=row['block']:
                raise RuntimeError('Hi-hat block incomplete')
            item=dict(row,start=begin,end=time.monotonic(),snare_events=hits,
                      measured_ride_peak_deg=math.degrees(s.peak_drop) if row['ride_hits'] else None)
            self.examples.append(item)
            self.blocks.append(dict(item,strikes=row['ride_hits']))
            dump(self.directory/'examples.json',self.examples)
            dump(self.directory/'blocks.json',self.blocks)
            self.event('CAPTURED',block=row['block'],kind=row['kind'],snare_hits=len(hits))
        self.event('FINITE BATCH COMPLETE',clips=len(self.examples))

    def wait_until(self, deadline):
        while time.monotonic()<deadline:
            self.cancellation(); self.check_audio(); self.hybrid_tick(); time.sleep(.002)

    def center_and_relax(self):
        self.recovering=True
        if self.hihat is not None: self.hihat.request_stop()
        # Cancel any unfinished snare reference without disabling or mode switching.
        self.bus.cancel_snare()
        if self.hybrid is not None:
            status=self.hybrid.status
            if not status.error:
                request=self.hybrid.request('finish')
                self.wait_hybrid(lambda s:s.request_id>=request and s.ready and not s.swing,allow_cancel=False)
        # Left centers while right remains held by its original owner.
        # Direct synchronized recovery is prevalidated; normal path reverses recording.
        if self.left_ready:
            self.left_move(LEFT_CENTER,'LEFT CENTER BEFORE RELAX')
        else:
            # A partially initialized arm is not eligible for a blind disable.
            self.bus.left_center_evidence()
        # Existing right collector returns powered J7 in its current mode.
        # DataMotors.relax adds the strict LEFT proof to its existing RIGHT gate.
        super().center_and_relax()

    def after_arm(self):
        if self.hihat is not None:
            self.hihat.stop()
            result=json.loads((self.directory/'hihat_result.json').read_text())
            if result['error'] or not result['release_ack']: raise RuntimeError(str(result))
            self.event('HIHAT OPEN RETURN AND RELEASE ACK',**result)

    def run_collection(self):
        error=recovery_error=None
        old={sig:signal.signal(sig,self.signal) for sig in (signal.SIGINT,signal.SIGTERM)}
        try:
            query_disabled(self.directory,'query_preflight.json')
            self.start_audio()
            notify(self.directory,'all',f'Robot moves in 10 seconds: {len(self.plan)} data clips; snare always 11 degrees, ride 10-12 degrees, hi-hat 90 degrees. BOTH arms center before relax. Keep clear.')
            for n in range(10,0,-1):
                self.cancellation(); print(f'Motion in {n}s',flush=True); time.sleep(1)
            self.before_arm()
            self.bus=DataMotors.adopt(Motors('right',control_gripper=True),self.g.center,directory=self.directory)
            self.bus.start_audit(self.directory)
            self.bus.require_center_before_relax=True
            refresh_feedback(self.bus,.15)
            if any(s[1]!=0 for s in self.bus.states.values()): raise RuntimeError('Both arms must initially be disabled')
            self.check_startup_path(self.positions())
            self.lg.check_line(self.left_positions(),LEFT_CENTER,measured_start=True)
            refresh_feedback(self.bus)
            self.powered_at=time.monotonic()
            # Configure right drives at their LIVE pose, not at a moving center
            # target. The validated synchronized move below owns centering.
            right_live=self.positions().copy()
            for _ in self.bus.center(right_live,RIGHT_GRIPPER_CLOSED,confirm_enabled=True):
                self.cancellation(); self.bus.poll(); self.hihat.check(); time.sleep(.002)
            # Grippers may still be closing after arm enable.
            deadline=time.monotonic()+5.
            while max(abs(self.bus.states['right',8][0]-RIGHT_GRIPPER_CLOSED),abs(self.bus.states['left',8][0]-LEFT_GRIPPER_TARGET))>math.radians(.75):
                self.bus.poll();self.hihat.check()
                if time.monotonic()>deadline:raise RuntimeError('Gripper close timeout')
                time.sleep(.002)
            self.enabled=self.left_ready=True
            self.move(self.g.center,'INITIAL RIGHT CENTER')
            self.left_settle(LEFT_CENTER,'INITIAL LEFT CENTER')
            self.bus.start_left_mit_at_center(self.left_tuning,self.left_recording.last_joints[5])
            self.left_settle(LEFT_CENTER,'LEFT MIT CENTER VERIFIED')
            self.bus.enter_mit_at_center()
            self.settle(self.g.center,'RIGHT MIT CENTER VERIFIED')
            self.move(self.trajectory.first,'RECORD3 START')
            self.play_record3()
            self.play_left()
            self.collect_blocks()
        except BaseException as exc:
            error=f'{type(exc).__name__}: {exc}'
            self.event('COLLECTION STOP',error=error)
        finally:
            if self.bus and self.bus.active and not self.disabled:
                try: self.center_and_relax()
                except BaseException as exc:
                    recovery_error=str(exc)
                    self.event('NOT RELAXED - ATTENTION REQUIRED',error=recovery_error)
                    # Never convert an unconfirmed pose into an unconditional relax.
                    # No new emergency-disable policy: leave drives powered if recovery fails.
            try: self.after_arm()
            except BaseException as exc:
                recovery_error=(recovery_error or '')+'; accessory: '+str(exc)
                error=error or recovery_error
            if self.audio:
                self.audio.terminate()
                try:self.audio.wait(timeout=5.)
                except Exception:
                    self.audio.kill();self.audio.wait();error=(error or '')+'; audio writer killed'
            if self.audio_log:self.audio_log.close()
            if self.bus:self.bus.close()
            self.trace_file.close()
            for sig,handler in old.items():signal.signal(sig,handler)
            if self.disabled: query_disabled(self.directory,'query_final_disabled.json')
            result=dict(success=error is None and recovery_error is None and self.disabled,
                error=error,recovery_error=recovery_error,relaxed_verified=self.disabled,
                clips=len(self.examples),snare_hits=sum(r['snare_hits'] for r in self.examples),
                center=self.center_evidence,finished_at=time.monotonic(),labels_created=False)
            dump(self.directory/'result.json',result)
            print('COLLECTION RESULT '+json.dumps(result),flush=True)
        return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true',help='Offline only; no devices opened')
    parser.add_argument('--session',choices=('development','test'),help='Collect only one of the two planned sessions')
    args=parser.parse_args()
    plan=make_plan(); prepared=preflight(); left_prepared=prepare_left()
    if args.check:
        print(json.dumps(dict(plan=plan,snare_degrees=SNARE_DEGREES,hihat_degrees=CLOSE_DEGREES,
            clips=len(plan),snare_hits=sum(r['snare_hits'] for r in plan),training=False),indent=2))
        return 0
    if os.environ.get('PLAYBACK_OFFLINE_ONLY')=='1' or os.environ.get('STRIKE_LAB_OFFLINE_ONLY')=='1':
        raise RuntimeError('Hardware forbidden by offline environment')
    for split in ([args.session] if args.session else ['development','test']):
        directory=ROOT/'diagnostics/snare_data_collection'/(
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+split+'-'+uuid.uuid4().hex[:8])
        directory.mkdir(parents=True)
        rows=[row for row in plan if row['split']==split]
        dump(directory/'plan.json',dict(plan=rows,split=split,snare_degrees=SNARE_DEGREES,hihat_degrees=CLOSE_DEGREES))
        print('Session:',directory,flush=True)
        result=HiHatCollector(prepared,directory,rows,left_prepared).run_collection()
        if not result['success']:return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
