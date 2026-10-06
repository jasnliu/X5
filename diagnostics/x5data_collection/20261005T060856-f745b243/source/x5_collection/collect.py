"""Center + close, record3, one finite capture, verified center + relax. No camera."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid

import numpy as np

from centering.motors import Motors, RIGHT_GRIPPER_CLOSED
from camera_playback.hybrid_strike import load_tuning
from camera_playback.playback_transport import PlaybackMotors
from camera_playback.recording_only import refresh_feedback
from camera_playback.smooth_recording import load_smooth_recording
from camera_playback.smooth_recording_worker import RecordingSession
from safe_zone.encoder import Observer, FRAME, EFF, PAYLOAD
from smooth_playback.runner import Runner
from smooth_playback.trajectory import ROOT, SOURCE, Geometry, transition
from .hybrid import CollectionSession
from .plan import make_plan, TONOR
from .transport import CollectionMotors


def dump(path, value):
    path.write_text(json.dumps(value, indent=2)+'\n')


def query_disabled(directory, name):
    """State-only query with independent audit of all transmitted requests."""
    sockets = {}
    observer = None
    frames = []
    try:
        for interface in ('can0', 'can1'):
            s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.bind((interface,)); s.setblocking(False); sockets[interface] = s
        observer = Observer()
        for _ in range(3):
            q = observer.sample()
            for interface, s in sockets.items():
                while True:
                    try:
                        cid, dlc, data = FRAME.unpack(s.recv(16))
                        frames.append(dict(bus=interface, id=cid, dlc=dlc, data=data.hex()))
                    except BlockingIOError:
                        break
            time.sleep(.05)
        requests = [f for f in frames if (f['id'] >> 8) & 255 == 0xfd]
        if len(requests) != observer.tx_count or len(requests) != 48 or not all(
                f['id'] in [EFF | 0x0200fd00 | i for i in range(1, 9)] and
                f['dlc'] == 8 and f['data'] == PAYLOAD.hex() for f in requests):
            raise RuntimeError('Query-only transmit audit failed')
        result = dict(query_only=True, all_16_disabled_fault_free=True, positions=q,
                      requests=len(requests), frames=frames)
        dump(directory/name, result)
        return result
    finally:
        if observer:
            observer.close()
        for s in sockets.values():
            s.close()


def notification(directory):
    spec = importlib.util.spec_from_file_location('x5data_ntfy', Path.home()/'.local/bin/codex-ntfy.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    text = ('Robot moves in 10 seconds: center + close gripper, play record3, '
            'collect 20 isolated and 10 double hybrid examples at J7 10-12 degrees '
            '(about 4 minutes), then CENTER and relax. Keep clear.')
    delivered = module.post_ntfy(text, 'X5 audio collection: robot will move', 'warning,robot')
    dump(directory/'notification.json', dict(delivered=bool(delivered), message=text, t=time.monotonic()))
    if not delivered:
        raise RuntimeError('ntfy delivery failed: refusing to enable motors')
    module.play_sound('{"type":"agent-turn-complete"}')


def preflight():
    g = Geometry()
    recording = load_smooth_recording(SOURCE, g.model, g.zone, g.lower, g.upper, g.center, .4)
    if recording.source.resolve() != (ROOT/'recordings/record3.json').resolve():
        raise RuntimeError('Collector must use record3.json')
    tuning = load_tuning()
    if tuning[2].hard_depth_deg != 12.:
        raise RuntimeError('Collector requires the strict 12-degree corridor')
    anchor = recording.last_joints.copy()
    low = anchor.copy(); low[6] -= math.radians(12.)
    high = anchor.copy(); high[6] += math.radians(tuning[2].upper_excursion_deg)
    g.check_line(low, high)
    # Center on synchronized joint-space paths, including interrupted strokes.
    # Do not send J7 directly to center while the other six remain at record3.
    for j7 in np.linspace(low[6], high[6], 31):
        q = anchor.copy(); q[6] = j7
        g.check_line(q, g.center)
    return g, recording, tuning, low[6], high[6]


class Collector(Runner):
    def __init__(self, prepared, directory, plan):
        self.recording, self.tuning, self.lower_j7, self.upper_j7 = prepared[1:]
        super().__init__(prepared[0], self.recording.smooth_motion, directory, gripper=True, video=False)
        self.plan = plan
        self.recording_session = self.hybrid = self.audio = None
        self.blocks = []
        self.audio_log = None
        self.powered_at = None
        self.startup_recovery = False
        self.startup_best_excess = 0.
        self.near_center_recovery = False

    def zone_excess(self, q):
        planes = self.g.zone.hull.equations
        return float(np.max((planes[:, :3] @ self.g.tcp(q)+planes[:, 3]) /
                            np.linalg.norm(planes[:, :3], axis=1)))-.005

    def check_startup_path(self, q):
        # The normal X5 workflow RECENTERS when a relaxed pose is outside
        # zone1. Allow only a tiny (<=2 mm beyond its existing 5 mm buffer),
        # prevalidated INWARD center recovery, not an expanded strike envelope.
        if not np.isfinite(q).all() or np.any(q < self.g.lower) or np.any(q > self.g.upper):
            raise RuntimeError('Startup outside joint limits; operator recovery required')
        excess = self.zone_excess(q)
        if excess > .002:
            raise RuntimeError('Relaxed pose too far outside TCP envelope for bounded center recovery')
        previous = excess
        for pose in np.linspace(q, self.g.center, 401):
            distance = self.zone_excess(pose)
            if distance > max(previous, 0.)+1e-8:
                raise RuntimeError('Initial center path does not move inward toward zone1')
            previous = distance
        self.startup_recovery = excess > 0.
        self.startup_best_excess = max(0., excess)
        self.event('INITIAL CENTER PATH VERIFIED', outside_buffer_mm=max(0., excess)*1000,
                   recovery_only=self.startup_recovery)

    def poll(self, check_running=True):
        if self.recovering and self.near_center_recovery:
            self.last_geometry = time.monotonic()
            q = super().poll(check_running)
            # A minor measured J7 overshoot at center must not make the return
            # path abort while leaving the arm powered. Only inward J7-to-center
            # recovery is allowed; J1-J6 must already remain at center.
            if (np.max(np.abs(q[:6]-self.g.center[:6])) > math.radians(.2)
                    or abs(q[6]-self.g.center[6]) > math.radians(1.)):
                raise RuntimeError('Near-center J7 recovery left its bounded region')
            return q
        if not self.startup_recovery:
            return super().poll(check_running)
        # Continue all normal feedback/running/gripper checks; replace only
        # zone membership during the explicitly bounded inward startup path.
        self.last_geometry = time.monotonic()
        q = super().poll(check_running)
        tolerance = 3*25.14/65535
        if np.any(q < self.g.lower-tolerance) or np.any(q > self.g.upper+tolerance):
            raise RuntimeError('Initial center recovery exceeded a joint limit')
        excess = self.zone_excess(q)
        if excess > self.startup_best_excess+.001:
            raise RuntimeError('Initial center recovery moved farther outside zone1')
        self.startup_best_excess = min(self.startup_best_excess, max(0., excess))
        if excess < -.001:
            self.startup_recovery = False
            self.event('ENTERED ORIGINAL ZONE1', outside_buffer_mm=excess*1000)
        return q

    def start_audio(self):
        sources = json.loads(subprocess.check_output(['pactl', '-f', 'json', 'list', 'sources']))
        source = next((s for s in sources if s['name'] == TONOR), None)
        if source is None:
            raise RuntimeError('The explicitly selected TONOR microphone is missing')
        python = ROOT.parent/'st7/.venv/bin/python'
        application_name = 'X5data-'+self.directory.name
        env = dict(os.environ, PULSE_SOURCE=TONOR, PA_ALSA_PLUGHW='1',
                   PIPEWIRE_PROPS=json.dumps({'application.name': application_name,
                                              'target.object': TONOR}))
        self.audio_log = (self.directory/'audio_capture.log').open('w')
        self.audio = subprocess.Popen([str(python), '-m', 'x5_collection.audio_capture', str(self.directory)],
            env=env, stdout=self.audio_log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+8.
        while time.monotonic() < deadline:
            self.cancellation()
            if self.audio.poll() is not None:
                raise RuntimeError('TONOR capture failed before movement; see audio_capture.log')
            if (self.directory/'audio_status.json').exists():
                self.check_audio()
                outputs = json.loads(subprocess.check_output(['pactl', '-f', 'json', 'list', 'source-outputs']))
                selected = [s for s in outputs if s.get('properties', {}).get('application.name') == application_name]
                if not selected or any(s['source'] != source['index'] for s in selected):
                    raise RuntimeError('Cannot verify capture is connected to TONOR; no motion allowed')
                dump(self.directory/'microphone.json', dict(source=source, streams=selected))
                return
            time.sleep(.05)
        raise RuntimeError('TONOR did not become ready')

    def check_audio(self):
        if self.audio is None or self.audio.poll() is not None:
            raise RuntimeError('Audio writer stopped; aborting collection')
        status = json.loads((self.directory/'audio_status.json').read_text())
        if status['errors'] or time.monotonic()-status['heartbeat'] > 1.:
            raise RuntimeError('Audio overflow/clock/heartbeat failure')

    def move(self, target, label):
        # Validation is CPU-only and may outlast the inherited 300 ms watchdog.
        self.event('CHECKING '+label)
        q = self.poll()
        if self.startup_recovery and np.array_equal(target, self.g.center):
            self.check_startup_path(q)
        else:
            self.validate_line_live(q, target)
        refresh_feedback(self.bus)
        if np.max(np.abs(self.positions()-q)) > math.radians(.2):
            raise RuntimeError('Arm moved during stationary path validation')
        duration, at = transition(q, target)
        self.event(label, duration_s=duration)
        self.stream(duration, at)
        return self.settle(target, label+' SETTLED')

    def validate_line_live(self, q, target):
        # Keep the existing powered MIT hold refreshed during expensive FK.
        # This is path validation, never a stream of path motion commands.
        n = max(2, int(np.ceil(np.max(np.abs(target-q))/math.radians(.25)))+1)
        for index, pose in enumerate(np.linspace(q, target, n)):
            self.g.check(pose, measured=True)
            if index % 4 == 0:
                self.cancellation()
                self.bus.poll()

    def tick(self, desired, t, lateness=0.):
        if self.bus.mit_center_return is not None:
            self.bus.mit_center_return.goal = float(desired[6])
        super().tick(desired, t, lateness)

    def settle(self, goal, label, *args, **kwargs):
        if self.bus.mit_center_return is not None:
            self.bus.mit_center_return.goal = float(goal[6])
        return super().settle(goal, label, *args, **kwargs)

    def hybrid_tick(self, allow_cancel=True):
        if allow_cancel:
            self.cancellation()
        s = self.hybrid.status
        self.poll()
        if s.error:
            raise RuntimeError(s.error)
        if s.sample is not None and time.monotonic()-s.at > .15:
            raise RuntimeError('Hybrid heartbeat stale')
        if np.max(np.abs(self.positions()[:6]-self.trajectory.last[:6])) > math.radians(self.tuning[2].held_joint_drift_deg):
            raise RuntimeError('Held joints drifted during collection')
        return s

    def wait_hybrid(self, predicate, timeout=4., allow_cancel=True):
        deadline = time.monotonic()+timeout
        while True:
            s = self.hybrid_tick(allow_cancel)
            if predicate(s):
                return s
            if time.monotonic() > deadline:
                raise RuntimeError('Finite hybrid operation timeout')
            time.sleep(.002)

    def play_record3(self):
        # The same precomputed record3 trajectory; J1-J6 use CSP, J7 tracks
        # its recorded reference in MIT after the CENTER-ONLY mode change.
        # This avoids the old off-center disable/mode-switch entirely.
        self.settle(self.trajectory.first, 'START HOLD')
        self.event('TEMPORARY POSITION GAINS')
        self.bus.apply_position_gain_cap(10.)
        self.bus.set_playback_speeds(self.trajectory.metadata['firmware_joint_speeds_rad_s'])
        self.event('PLAYBACK RECORD3', duration_s=self.trajectory.duration, j7_mode='powered MIT')
        self.stream(self.trajectory.duration, self.trajectory.at, 200)
        self.settle(self.trajectory.last, 'ENDPOINT SOFT HOLD')
        self.event('ENDPOINT GAIN RESTORATION', ramp_s=.5)
        def guarded_hold():
            self.cancellation()
            if np.max(np.abs(self.poll()-self.trajectory.last)) > math.radians(.25):
                raise RuntimeError('Endpoint gain restoration left hold envelope')
            self.bus.set_positions(self.trajectory.last)
        self.bus.restore_position_gains(ramp_seconds=.5, tick=guarded_hold)
        self.endpoint = self.settle(self.trajectory.last, 'ENDPOINT HOLD', math.radians(.05), math.radians(.066))
        self.bus.set_right_arm_speed(.4)
        self.event('RECORD3 ENDPOINT VERIFIED', **self.endpoint)

    def collect_blocks(self):
        # Ownership transfers from parent MIT hold to child MIT hold, never
        # changing modes or disabling an airborne joint.
        self.bus.mit_center_return = None
        self.hybrid = CollectionSession(self.bus, self.trajectory.last[6], self.lower_j7,
                                        self.upper_j7, self.tuning, self.directory)
        refresh_feedback(self.bus, .1)
        self.wait_hybrid(lambda s: s.ready)
        # Baseline at the endpoint, then all attempts without analysis/labeling.
        until = time.monotonic()+2.
        while time.monotonic() < until:
            self.hybrid_tick(); self.check_audio(); time.sleep(.005)
        for row in self.plan:
            self.check_audio()
            begin = time.monotonic()
            request = self.hybrid.block(row['depth_deg'], row['strikes'], row['block'])
            s = self.wait_hybrid(lambda s: s.request_id >= request and s.ready and not s.swing and s.count == row['strikes'])
            block = dict(row, requested_at=begin, completed_at=time.monotonic(),
                         last_release=s.released_at, last_return=s.returned_at,
                         last_peak_deg=math.degrees(s.peak_drop), partial_returns=s.partial_returns)
            self.blocks.append(block)
            self.event('CAPTURED BLOCK', **block)
            dump(self.directory/'blocks.json', self.blocks)
            # Fixed bounded recording time; no listening/analysis with motors on.
            until = time.monotonic()+row['ringdown_s']
            while time.monotonic() < until:
                self.hybrid_tick(); self.check_audio(); time.sleep(.005)
        self.event('FINITE BATCH COMPLETE', blocks=len(self.blocks), attempted_strikes=sum(b['strikes'] for b in self.blocks))

    def center_and_relax(self):
        self.recovering = True
        if self.recording_session is not None:
            result = self.recording_session.stop()
            self.recording_session = self.bus.playback_session = None
            self.event('RECORDING WORKER JOINED', result=result)
        if self.hybrid is not None:
            s = self.hybrid.status
            if not s.error:
                request = self.hybrid.request('finish')
                try:
                    self.wait_hybrid(lambda x: x.request_id >= request and x.ready and not x.swing, allow_cancel=False)
                except Exception as exc:
                    self.event('HYBRID FINISH ERROR', error=str(exc))
            self.hybrid.stop()
            self.hybrid = None
        refresh_feedback(self.bus, .12)
        self.bus.request_right_joint7_mode_readback(clear=True)
        start = time.monotonic()
        while True:
            self.bus.poll()
            mode = self.bus.right_joint7_mode_readback()
            if mode in (0, 5):
                break
            if time.monotonic()-start > .5:
                raise RuntimeError('Cannot safely center: J7 mode not confirmed')
            time.sleep(.002)
        if any(self.bus.states['right', i][1] != 2 for i in range(1, 9)):
            raise RuntimeError('Cannot blindly return: a right drive is not running')
        self.enabled = True
        q = self.positions()
        self.near_center_recovery = (np.max(np.abs(q[:6]-self.g.center[:6])) <= math.radians(.2)
                                    and abs(q[6]-self.g.center[6]) <= math.radians(1.))
        if self.near_center_recovery:
            self.g.check(self.g.center)
        elif self.startup_recovery:
            self.check_startup_path(q)
        else:
            self.validate_line_live(q, self.g.center)
        refresh_feedback(self.bus)
        self.bus.set_right_arm_speed(.4)
        if mode == 0:
            self.bus.begin_mit_center_return(0. if self.near_center_recovery else self.tuning[0]['load_torque'])
            self.bus.mit_center_return.goal = q[6]
        duration, at = transition(q, self.g.center)
        self.event('CENTER BEFORE RELAX', j7_mode=mode, duration_s=duration)
        # J7 remains in its CURRENT powered mode, following the same slow
        # return path as J1-J6 through the bounded MIT reference controller.
        start = time.monotonic()
        while time.monotonic()-start < duration:
            target = at(time.monotonic()-start)
            if self.bus.mit_center_return is not None:
                self.bus.mit_center_return.goal = float(target[6])
            q_now = self.poll()
            if np.max(np.abs(q_now[:6]-target[:6])) > math.radians(8.):
                raise RuntimeError('Center return tracking error')
            self.bus.set_positions(target)
            self.trace.writerow([time.monotonic(), self.phase, time.monotonic()-start, 0, *q_now, *target])
            time.sleep(.005)
        if self.bus.mit_center_return is not None:
            self.bus.mit_center_return.goal = float(self.g.center[6])
        self.center_evidence = self.settle(self.g.center, 'CENTER SETTLED')
        if not self.bus.gains_restored:
            self.bus.restore_position_gains()
            self.center_evidence = self.settle(self.g.center, 'CENTER GAINS RESTORED')
        self.bus.verify_original_gains_at_center()
        self.bus.relax()
        self.enabled = False
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            self.bus.poll()
            if self.bus.fresh() and all(v[1] == 0 for v in self.bus.states.values()):
                self.disabled = True
                self.event('RELAXED VERIFIED', center=self.bus.last_center_evidence)
                return
            time.sleep(.005)
        raise RuntimeError('Disable readback not confirmed')

    def run_collection(self):
        error = recovery_error = None
        old = {sig: signal.signal(sig, self.signal) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            query_disabled(self.directory, 'query_preflight.json')
            self.start_audio()
            notification(self.directory)
            self.event('NTFY ACCEPTED')
            for seconds in range(10, 0, -1):
                self.cancellation(); print(f'Motion in {seconds}s', flush=True); time.sleep(1)
            self.check_audio()
            self.bus = CollectionMotors.adopt(Motors('right', control_gripper=True), self.g.center, directory=self.directory)
            self.bus.start_audit(self.directory)
            self.bus.require_center_before_relax = True
            refresh_feedback(self.bus, .15)
            if any(s[1] != 0 for s in self.bus.states.values()):
                raise RuntimeError('Both arms must be disabled before collection')
            self.check_startup_path(self.positions())
            refresh_feedback(self.bus)
            self.powered_at = time.monotonic()
            for _ in self.bus.center(self.g.center, RIGHT_GRIPPER_CLOSED, confirm_enabled=True):
                self.bus.poll(); time.sleep(.002)
            deadline = time.monotonic()+3.
            while abs(self.bus.states['right', 8][0]-RIGHT_GRIPPER_CLOSED) > math.radians(.75):
                self.bus.poll()
                if time.monotonic() > deadline:
                    raise RuntimeError('Gripper closure timeout')
                time.sleep(.005)
            self.enabled = True
            self.move(self.g.center, 'INITIAL CENTER AND CLOSE')
            self.bus.enter_mit_at_center()
            self.event('J7 MIT CONFIRMED AT CENTER')
            self.settle(self.g.center, 'MIT CENTER SETTLED', math.radians(.05), math.radians(.066))
            self.move(self.trajectory.first, 'RECORD3 START')
            self.play_record3()
            self.collect_blocks()
        except BaseException as exc:
            error = f'{type(exc).__name__}: {exc}'
            self.event('COLLECTION STOP', error=error)
        finally:
            if self.bus and self.bus.active and not self.disabled:
                try:
                    self.center_and_relax()
                except BaseException as exc:
                    recovery_error = str(exc)
                    self.event('NOT RELAXED - ATTENTION REQUIRED', error=recovery_error)
                    # No unconditional disable in finally, even after a fault.
            if self.audio:
                self.audio.terminate()
                try:
                    self.audio.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    self.audio.kill(); self.audio.wait()
                    error = (error or '')+'; Audio writer required kill'
            if self.audio_log:
                self.audio_log.close()
            if self.bus:
                self.bus.close()
            self.trace_file.close()
            for sig, handler in old.items():
                signal.signal(sig, handler)
            if self.disabled:
                query_disabled(self.directory, 'query_final_disabled.json')
            result = dict(success=error is None and self.disabled, error=error,
                recovery_error=recovery_error, center=self.center_evidence,
                relaxed_verified=self.disabled, blocks=len(self.blocks),
                attempted_strikes=sum(b['strikes'] for b in self.blocks),
                endpoint=self.endpoint, powered_at=self.powered_at,
                finished_at=time.monotonic(), directory=str(self.directory), labels_created=False)
            dump(self.directory/'result.json', result)
            print('COLLECTION RESULT '+json.dumps(result), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Offline preflight only: no microphone, CAN or motion')
    args = parser.parse_args()
    print('Preparing only recordings/record3.json and all bounded paths OFFLINE...', flush=True)
    prepared = preflight()
    plan = make_plan()
    if args.check:
        print(json.dumps(dict(recording=str(SOURCE), plan=plan, commanded_depths=[10, 10.5, 11, 11.5, 12],
            measured_corridor_deg=12, hybrid_parameters=prepared[2][0],
            duration_s=prepared[1].duration_s), indent=2))
        return 0
    if os.environ.get('PLAYBACK_OFFLINE_ONLY') == '1' or os.environ.get('STRIKE_LAB_OFFLINE_ONLY') == '1':
        raise RuntimeError('Hardware forbidden by offline environment')
    directory = ROOT/'diagnostics/x5data_collection'/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    for kind in ('recordings', 'timestamps'):
        (ROOT/'X5data'/kind).mkdir(parents=True, exist_ok=True)
    dump(directory/'plan.json', dict(plan=plan, source=str(SOURCE),
        source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        tuning=[prepared[2][0], asdict(prepared[2][1]), asdict(prepared[2][2])],
        microphone=TONOR, mode='hybrid', camera=False, training=False))
    print('Session:', directory, flush=True)
    result = Collector(prepared, directory, plan).run_collection()
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
