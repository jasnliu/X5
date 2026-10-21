"""Finite, separately reviewed bass data collection. Never trains or loads models."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

from camera_playback.hihat import DEFAULT_ESP_PORT
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET
from camera_playback.left_return import ReverseRecording
from camera_playback.recording_only import refresh_feedback
from centering.motors import Motors, RIGHT_GRIPPER_CLOSED
from hihat_collection.collect import HiHatCollector, prepare_left, notify
from x5_collection.collect import Collector, preflight, dump, query_disabled
from x5_collection.hybrid import CollectionSession
from .control import BassSession
from .plan import make_plan, schedule, PULSE, SPLITS
from .transport import BassDataMotors, damaging_fault

ROOT=Path(__file__).resolve().parents[1]


class BassCollector(HiHatCollector):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.left_recording_progress=0.
        self.left_recording_started=False
        self.left_reverse_verified=False

    def play_left(self, reverse=False):
        if reverse:
            return self.reverse_left_to_start()
        recording=self.left_recording
        self.left_move(recording.first_joints,'LEFT RECORDING START')
        self.left_recording_started=True
        self.left_reverse_verified=False
        self.event('LEFT RECORDING')
        def at(elapsed):
            target=recording.joints_at(elapsed)
            self.left_recording_progress=min(recording.duration_s,max(0.,elapsed))
            return target
        self.left_stream(recording.duration_s,at)
        self.left_settle(recording.last_joints,'LEFT ENDPOINT VERIFIED')

    def reverse_left_to_start(self):
        # As in start_beat: rejoin the played path, traverse it backward,
        # verify its start, and only THEN permit the separate center leg.
        # An interrupted forward/reverse playback retains its last commanded
        # progress, so recovery never plays an untraversed remainder forward.
        from_seconds=self.left_recording_progress
        recording=ReverseRecording(self.left_recording,from_seconds)
        self.left_move(recording.first_joints,'LEFT REVERSE START')
        self.event('LEFT REVERSE RECORDING',reverse_from_seconds=from_seconds)
        def at(elapsed):
            target=recording.joints_at(elapsed)
            self.left_recording_progress=max(0.,from_seconds-max(0.,elapsed))
            return target
        self.left_stream(recording.duration_s,at)
        proof=self.left_settle(recording.last_joints,'LEFT RECORDING START VERIFIED')
        self.left_recording_progress=0.
        self.left_reverse_verified=True
        dump(self.directory/'left_reverse_before_center.json',dict(
            reverse_from_seconds=from_seconds,recording_start_verified=True,
            start_proof=proof,finished_at=time.monotonic()))

    def center_and_relax(self):
        self.recovering=True
        if self.hihat is not None: self.hihat.request_stop()
        self.bus.cancel_snare()
        if self.hybrid is not None:
            status=self.hybrid.status
            if not status.error:
                request=self.hybrid.request('finish')
                self.wait_hybrid(lambda s:s.request_id>=request and s.ready and not s.swing,allow_cancel=False)
        if self.left_ready:
            if self.left_recording_started and not self.left_reverse_verified:
                self.reverse_left_to_start()
            if self.left_recording_started and not self.left_reverse_verified:
                raise RuntimeError('Left reverse/start verification required before centering')
            self.left_move(LEFT_CENTER,'LEFT CENTER AFTER REVERSE' if self.left_recording_started else 'LEFT INITIAL CENTER RECOVERY')
        else:
            self.bus.left_center_evidence()
        # Bypass the older dual collector's direct-left-center implementation.
        # The existing right return retains the strict BOTH-center relax gate.
        Collector.center_and_relax(self)

    def before_arm(self):
        self.hihat=BassSession(DEFAULT_ESP_PORT,self.directory)
        deadline=time.monotonic()+8.
        while self.hihat.check()['state']!='idle':
            self.check_audio(); self.cancellation()
            if time.monotonic()>deadline: raise RuntimeError('Accessory readiness timeout')
            time.sleep(.01)

    def collect_blocks(self):
        self.bus.mit_center_return=None
        self.hybrid=CollectionSession(self.bus,self.trajectory.last[6],self.lower_j7,self.upper_j7,self.tuning,self.directory)
        refresh_feedback(self.bus,.1)
        self.wait_hybrid(lambda s:s.ready)
        self.wait_until(time.monotonic()+2.)
        for row in self.plan:
            self.check_audio()
            self.left_settle(self.left_recording.last_joints,'SNARE ANCHOR VERIFIED')
            self.wait_hybrid(lambda s:s.ready and not s.swing)
            begin=time.monotonic(); nominal=begin+2.
            accessories,arms=schedule(row,nominal)
            self.hihat.block(row['block'],accessories)
            snare_index=len(self.bus.snare_done); requests=[]; requested=[]
            self.event('CAPTURING',**row)
            while time.monotonic()<begin+row['duration_s']:
                self.cancellation(); self.check_audio(); s=self.hybrid_tick()
                now=time.monotonic()
                if arms and now>=arms[0][0]:
                    due,kind=arms.pop(0)
                    if now-due>.08: raise RuntimeError('Arm event missed 80 ms deadline')
                    if kind=='snare': self.bus.start_snare(row['block'])
                    else:
                        if not s.ready or s.swing: raise RuntimeError('Ride not fully returned before next stroke')
                        requests.append(self.hybrid.block(row['ride_depth_deg'],1,row['block']))
                    requested.append(dict(instrument=kind,due=due,requested=time.monotonic()))
                    self.event('STRIKE REQUEST',block=row['block'],instrument=kind,due=due)
                time.sleep(.001)
            hits=self.bus.snare_done[snare_index:]
            if arms or len(hits)!=row['snare_hits'] or self.bus.snare_method:
                raise RuntimeError('Snare block incomplete')
            if requests and not(s.request_id>=requests[-1] and s.ready and not s.swing and s.count==1):
                raise RuntimeError('Ride block incomplete')
            if self.hihat.check()['completed']!=row['block']: raise RuntimeError('Accessory block incomplete')
            item=dict(row,start=begin,end=time.monotonic(),nominal=nominal,
                      accessory_schedule=accessories,arm_requests=requested,snare_events=hits)
            self.examples.append(item); self.blocks.append(dict(item,strikes=row['ride_hits']))
            dump(self.directory/'examples.json',self.examples)
            dump(self.directory/'blocks.json',self.blocks)
            self.event('CAPTURED',block=row['block'],kind=row['kind'])
        self.event('FINITE BATCH COMPLETE',clips=len(self.examples))

    def finish_safely(self, error=None):
        if error and damaging_fault(error):
            self.emergency_shutdown(error)
        else:
            self.center_and_relax()

    def emergency_shutdown(self, reason):
        self.recovering=True
        if self.hihat: self.hihat.request_stop()
        if self.hybrid:
            self.hybrid.stop(); self.hybrid=None
        self.bus.cancel_snare()
        self.bus.emergency_stop(reason)
        self.event('HARDWARE EMERGENCY DISABLE',reason=str(reason))
        # Never claim centered success after this path, even if disable succeeded.

    def supervised_hold(self, failure):
        """Keep ownership alive after a failed normal return; never implicit relax.

        A separate supervised retry_center file can request another prevalidated
        return after the operator has diagnosed the cause. SIGINT is not a bypass.
        Firmware faults/overtemperature/stall use the explicit emergency exception.
        """
        self.recovering=True
        if self.hihat: self.hihat.request_stop()
        if damaging_fault(failure):
            self.emergency_shutdown(str(failure)); return
        # Cancel moving references without disabling. Existing worker fault
        # handling retains its bounded powered hold until joined.
        if self.hybrid:
            self.hybrid.stop(); self.hybrid=None
        self.bus.cancel_snare()
        try:
            refresh_feedback(self.bus,.1)
            if self.bus.active and self.bus.fresh():
                q=self.positions().copy()
                self.bus.request_right_joint7_mode_readback(clear=True)
                refresh_feedback(self.bus,.1)
                if self.bus.right_joint7_mode_readback()==0:
                    self.bus.begin_mit_center_return(self.tuning[0]['load_torque'])
                    self.bus.mit_center_return.goal=float(q[6])
                self.bus.set_positions(q)
                if self.bus.left_drive.active: self.bus.left_drive.set_positions(self.left_positions())
        except Exception as exc:
            if damaging_fault(exc): self.emergency_shutdown(str(exc)); return
            failure=str(failure)+'; hold setup: '+str(exc)
        last=0.
        while self.bus.active and not self.disabled:
            try:
                self.bus.poll()
                retry=self.directory/'retry_center'
                if retry.exists():
                    retry.unlink(); self.center_and_relax()
            except BaseException as exc:
                if damaging_fault(exc): self.emergency_shutdown(str(exc)); return
                failure=str(exc)
            now=time.monotonic()
            if now-last>2.:
                dump(self.directory/'attention.json',dict(error=str(failure),t=now,
                     action='Supervised powered hold; diagnose before requesting retry_center. Do not kill.'))
                self.event('POWERED HOLD - OPERATOR ATTENTION REQUIRED',error=str(failure));last=now
            time.sleep(.002)

    def run_collection(self):
        error=recovery_error=None
        old={sig:signal.signal(sig,self.signal) for sig in (signal.SIGINT,signal.SIGTERM)}
        try:
            query_disabled(self.directory,'query_preflight.json')
            self.start_audio()
            notify(self.directory,'all','Robot motion begins in 10 seconds. Keep clear; controlled centering precedes normal shutdown.')
            for n in range(10,0,-1):
                self.cancellation(); print(f'Motion in {n}s',flush=True); time.sleep(1)
            self.before_arm()
            self.bus=BassDataMotors.adopt(Motors('right',control_gripper=True),self.g.center,directory=self.directory)
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
                try: self.finish_safely(error)
                except BaseException as exc:
                    recovery_error=str(exc)
                    self.event('NOT RELAXED - ATTENTION REQUIRED',error=recovery_error)
                    # An ordinary failure cannot exit and abandon active owners.
                    self.supervised_hold(exc)
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
                clips=len(self.examples),bass_hits=sum(r['bass_hits'] for r in self.examples),
                training_performed=False,left_reverse_verified=self.left_reverse_verified,
                hardware_emergency=bool(getattr(self.bus,'bass_emergency',False)),
                center=self.center_evidence,finished_at=time.monotonic(),labels_created=False)
            dump(self.directory/'result.json',result)
            print('COLLECTION RESULT '+json.dumps(result),flush=True)
        return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true',help='Offline plan/geometry validation only')
    parser.add_argument('--execute',action='store_true',help='Enable authorized physical collection')
    parser.add_argument('--session',choices=SPLITS)
    parser.add_argument('--blocks',help='Optional comma-separated planned block numbers for staged capture')
    args=parser.parse_args()
    plan=make_plan()
    if args.session: plan=[r for r in plan if r['split']==args.session]
    if args.blocks:
        ids={int(i) for i in args.blocks.split(',')}
        plan=[r for r in plan if r['block'] in ids]
        if {r['block'] for r in plan}!=ids: parser.error('Unknown block or session mismatch')
    if not plan: parser.error('Empty plan')
    prepared=preflight(); left_prepared=prepare_left()
    left_prepared[0].check_line(left_prepared[1].first_joints,LEFT_CENTER,measured_start=True)
    from snare_lab.runner import dipped_target
    left_prepared[0].check_line(dipped_target(left_prepared[1],14.),left_prepared[1].last_joints,measured_start=True)
    if args.check:
        print(json.dumps(dict(plan=plan,clips=len(plan),bass_pulse=PULSE,training=False),indent=2)); return 0
    if not args.execute: parser.error('Physical motion requires --execute (or use --check)')
    if os.environ.get('PLAYBACK_OFFLINE_ONLY')=='1' or os.environ.get('STRIKE_LAB_OFFLINE_ONLY')=='1':
        raise RuntimeError('Hardware forbidden by offline environment')
    for split in SPLITS:
        rows=[r for r in plan if r['split']==split]
        if not rows: continue
        directory=ROOT/'diagnostics/bass_data_collection'/(
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+split+'-'+uuid.uuid4().hex[:8])
        directory.mkdir(parents=True)
        dump(directory/'plan.json',dict(plan=rows,split=split,bass_pulse=PULSE,training_performed=False))
        print('Session:',directory,flush=True)
        result=BassCollector(prepared,directory,rows,left_prepared).run_collection()
        if not result['success']: return 1
    return 0


if __name__=='__main__': raise SystemExit(main())
