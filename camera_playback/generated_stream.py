"""Shared plan_drums Controller at 200 Hz, isolated from Tk/ROS callbacks.

The existing GUI retains motor ownership. This delegated writer can ONLY query
and send selected-arm positions (or feed an already-held MIT owner's target).
It never changes a path, time law, gain, speed, current, mode or enable state.
Readiness/GUI acknowledgement precede the path clock and active watchdog.
"""
import csv
import json
import multiprocessing as mp
from multiprocessing.reduction import DupFd
import os
import pickle
from pathlib import Path
import signal
import struct
import time
import uuid

import numpy as np

from centering.motors import Motors, parameter, packet, RIGHT_GRIPPER_CLOSED
from safe_zone.encoder import FRAME, EFF, joint_to_motor
from drum_planner.baseline.paths import recovery_at, blend
from .left_hold import LEFT_GRIPPER_TARGET
from .smooth_recording_worker import RecordingMotors
from .generated_controller import GeneratedController
from .generated_can import FeedbackQueries, receive_without_query_burst
from drum_planner.baseline.motion import HOLD_PHASES

PERIOD = .005
STARTUP_TIMEOUT = 10.  # Bounded initialization, NOT the active GUI watchdog.
HEARTBEAT_TIMEOUT = .5  # Unchanged once the GUI acknowledges a ready worker.


class Progress:
    """Single-writer double buffer; a killed worker cannot strand a mutex."""
    def __init__(self, ctx, values):
        self.values=ctx.Array('d', list(values)*2, lock=False)
        self.generation=ctx.Value('Q', 0, lock=False)

    def read(self):
        for _ in range(3):
            generation=self.generation.value; slot=generation%2
            result=list(self.values[slot*10:(slot+1)*10])
            if self.generation.value==generation:return result
        raise RuntimeError('Generated progress snapshot unavailable')

    def write(self, values):
        generation=self.generation.value+1; slot=generation%2
        self.values[slot*10:(slot+1)*10]=values
        self.generation.value=generation


def reference(curve, step, elapsed):
    """Evaluate the original law, including the unchanged checked return law."""
    duration = curve.duration if step is None else step['duration']
    elapsed = min(max(0., elapsed), duration)
    if step is None:
        return curve.at(elapsed), elapsed
    if step['kind'] == 'prefix':
        q, tau, _ = recovery_at(curve, step['spec'], elapsed)
        return q, tau
    if step['kind'] == 'planned':
        if elapsed == 0.:
            # Parent IPC initialization only. The worker validates/evaluates
            # the full exact curve before readiness; rebuilding it here can
            # block Tk longer than the STILL-POWERED strike owners' watchdog.
            q=np.asarray(step['start'] if 'start' in step else step['plan']['outward'][0]['positions_rad'],float)
            if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Invalid calculated return start')
            return q,None
        from drum_planner.baseline.curve import Curve
        return Curve(step['plan']).at(elapsed), None
    return (np.asarray(step['start']) + (np.asarray(step['goal'])-step['start'])
            * blend(elapsed/duration)), None


class StreamMotors(RecordingMotors):
    """Reuse established CAN/sign/readback handling with a narrower allowlist."""
    def __init__(self, directory, handles, side, special=None, shared=None):
        self.special = special
        super().__init__(directory, handles, side=side)
        self.feedback_queries=FeedbackQueries(PERIOD,side)

    def poll(self):
        # Same decoder, CaptureSocket audit and faults, without AuditedMotors'
        # eight-query bursts on top of the 200 Hz CSP / MIT writers.
        try:receive_without_query_burst(self,lambda:Motors.poll(self))
        except Exception:
            try:self.feedback_queries.poll(self)
            except Exception:pass
            raise
        self.feedback_queries.poll(self)

    def _send(self, side, frame):
        cid, dlc, data = FRAME.unpack(frame)
        kind, motor = (cid >> 24) & 31, cid & 255
        index = int.from_bytes(data[:2], 'little')
        if (side != self.control_side or motor not in range(1, 8) or dlc != 8
                or cid != (EFF | kind<<24 | 0xfd<<8 | motor)
                or not (kind == 17 and index == 0x7005 or kind == 18 and index == 0x7016)):
            raise RuntimeError('Generated writer permits ONLY mode queries and selected-arm positions')
        if kind == 18 and not np.isfinite(struct.unpack('<f',data[4:])[0]):
            raise RuntimeError('Nonfinite generated position packet')
        if self.special is not None and motor == self.special['motor'] and kind == 18:
            raise RuntimeError('MIT-owned joint cannot receive a CSP target')
        # No inherited gain/speed writes; sign conversion is done by command().
        if self.sockets[side].send(frame) != FRAME.size:
            raise RuntimeError('Short generated position transmission')
        self.command_file.write(json.dumps(dict(t=time.monotonic(), side=side,
            kind=kind, motor=motor, frame=frame.hex()))+'\n')

    def command(self, q):
        q = np.asarray(q, float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise RuntimeError('Invalid generated target')
        for i, value in enumerate(q, 1):
            if self.special is not None and i == self.special['motor']:
                if not self.special['hold'].is_set():
                    raise RuntimeError('Strike owner must remain in Hold during generated return')
                self.special['goal'].value = float(value)  # display coordinates, same as path_goal()
            else:
                self.send_control(parameter(i, 0x7016, joint_to_motor(self.control_side, i, float(value))))

    def prime(self, check):
        until = time.monotonic()+3.
        while not self.fresh():
            check(); self.poll()
            if time.monotonic() > until:
                raise RuntimeError('Generated writer feedback startup timeout')
            time.sleep(.001)
        for i in range(1, 8):
            self.send_control(packet(17, i, bytes.fromhex('0570000000000000')))
            check(); self.poll(); time.sleep(.001)  # Pace startup mode queries too.
        until = time.monotonic()+.5
        while len(self.modes) != 7:
            check(); self.poll()
            if time.monotonic() > until:
                raise RuntimeError('Generated writer mode readback timeout')
            time.sleep(.001)
        for i, (mode, _) in self.modes.items():
            expected = 0 if self.special is not None and i == self.special['motor'] else 5
            if mode != expected:
                raise RuntimeError('Generated writer found unexpected motor mode; no mode changes made')
        self.active = True


def feedback(bus, geometries, now):
    """Same generated-mode bounds, zones, freshness and gripper guards as Tk."""
    if (not bus.fresh() or len(bus.states) != 16
            or any(now-v[2] > .12 or v[1] != 2 for v in bus.states.values())):
        raise RuntimeError('Generated writer requires fresh running feedback from both arms')
    values = {s:np.array([bus.states[s, i][0] for i in range(1, 8)]) for s in ('left', 'right')}
    for side, q in values.items():
        if not np.isfinite(q).all():raise RuntimeError('Nonfinite generated feedback')
        grip = LEFT_GRIPPER_TARGET if side == 'left' else RIGHT_GRIPPER_CLOSED
        if abs(bus.states[side, 8][0]-grip) > np.radians(1.):
            raise RuntimeError('Custom gripper lost its closed position')
        if side in geometries:
            g = geometries[side]; lower = g.lower.copy()
            if side == 'right':lower[1] = np.radians(-1.)
            if np.any(q < lower-.001151) or np.any(q > g.upper+.001151):
                raise RuntimeError(side+' joint outside feedback limits')
            if not g.zone.contains(g.tcp(q), .005):
                raise RuntimeError(side+' TCP outside its original zone')
    return values


class Cancelled(Exception):pass


def run_controller(engine, check, progress, *, clock=time.monotonic, wait=time.sleep):
    """Schedule the SHARED plan_drums controller at 200 Hz; no second follower.

    Controller.step owns elapsed-time evaluation and the 80 ms deadline guard.
    The scheduler skips missed ticks instead of bursting stale targets.
    """
    engine.begin_segment()
    start = engine.segment_started; index = 0
    while not engine.complete:
        check()
        now = clock(); deadline = start+index*PERIOD
        if now < deadline:
            wait(min(.001, deadline-now)); continue
        engine.step()
        if engine.phase in HOLD_PHASES:
            raise RuntimeError(engine.error)
        elapsed = min(engine.previous-start, engine.segment_duration)
        tau = None if engine.segment is not None and not engine.prefix else engine.tau
        progress(elapsed, tau, engine.anchor, engine.previous-deadline)
        index = max(index+1, int((clock()-start)/PERIOD)+1)


def _worker(curve, step, side, geometries, targets, directory, handles, special, shared,
            cancel, heartbeat, ready, run, state, parent, created, factory, inputs_path=None):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    bus = engine = None
    result = dict(success=False, error=None, held=None, executor='plan_drums.baseline.motion.Controller')
    last_geometry = [0.]; started = time.monotonic(); running = False
    max_heartbeat_age = 0.
    def check():
        nonlocal max_heartbeat_age
        if cancel.is_set():raise Cancelled('Generated playback cancelled')
        if os.getppid() != parent:
            raise RuntimeError('Generated GUI owner exited; no further trajectory advancement')
        now = time.monotonic()
        if running:
            age = now-heartbeat.value
            max_heartbeat_age = max(max_heartbeat_age, age)
            if age > HEARTBEAT_TIMEOUT:
                raise RuntimeError('Generated GUI heartbeat lost; no further trajectory advancement')
        elif now-created > STARTUP_TIMEOUT:
            raise RuntimeError('Generated controller startup/ready acknowledgement timed out; no trajectory started')
    def sample(bus, now):
        geometry = geometries if now-last_geometry[0] >= .02 else {}
        values = feedback(bus, geometry, now)
        if geometry:last_geometry[0] = now
        return values
    try:
        check()
        if inputs_path is not None:
            # Private, locally-created IPC, never user-supplied pickle. Keep the
            # multiprocessing bootstrap pipe tiny: a large payload blocks the
            # parent in Process.start() until this child's imports complete.
            with open(inputs_path,'rb') as source:
                curve,step,geometries,targets=pickle.load(source)
            Path(inputs_path).unlink()
            check()
        bus = factory(directory, handles, side, special, shared)
        bus.prime(check)
        engine = GeneratedController(bus, curve, side, step, targets, sample)
        engine.started = True  # Parent already owns powered motors; no enables here.
        engine.sample(); check()
        result['ready_after_s'] = time.monotonic()-created
        ready.set()
        # Spawn/import/unpickling and read-only feedback initialization are NOT
        # lost heartbeats. No path clock or position command starts until a
        # live GUI callback acknowledges readiness. Cancel/parent death and a
        # separate bounded initialization deadline still apply throughout.
        while not run.is_set():
            check(); engine.sample(); time.sleep(.005)
        running = True; check()
        result['start_ack_after_s'] = time.monotonic()-created
        with open(Path(directory)/'trace.csv', 'w', buffering=65536) as trace:
            rows = csv.writer(trace); rows.writerow(['monotonic_s','elapsed_s','path_time_s','lateness_s',*[f'q{i}' for i in range(1,8)]])
            def progress(elapsed, tau, q, lateness):
                state.write([elapsed, -1. if tau is None else tau, *q, 1.])
                rows.writerow([time.monotonic(),elapsed,tau,lateness,*q])
            run_controller(engine, check, progress)
        result['success'] = True
    except BaseException as exc:
        result['error'] = str(exc)
        result['cancelled'] = isinstance(exc, Cancelled)
        # Use the SAME measured-pose Hold as plan_drums. Failure to obtain healthy
        # feedback is reported, never authority to relax, replan or invent a pose.
        if engine is not None:
            engine.halt(str(exc), fault=True)
            if engine.held is not None:result['held'] = engine.held.tolist()
            if engine.feedback_error:result['hold_error'] = engine.feedback_error
        elif bus is not None:
            try:
                bus.poll(); q = feedback(bus, {}, time.monotonic())[side]
                bus.command(q); result['held'] = q.tolist()
            except Exception as hold_error:result['hold_error'] = str(hold_error)
    finally:
        if engine is not None:
            result['controller_phase'] = engine.phase
            result['peak_tracking_error_rad'] = engine.peak_tracking_error.tolist()
        if bus is not None:
            try:bus.close()  # sockets only; never disables
            except Exception as exc:result.update(success=False,error='Writer cleanup: '+str(exc))
        values=state.read(); values[9]=0.; state.write(values)
        result.update(command_hz=200, elapsed_wall_s=time.monotonic()-started,
                      startup_timeout_s=STARTUP_TIMEOUT, heartbeat_timeout_s=HEARTBEAT_TIMEOUT,
                      max_running_heartbeat_age_s=max_heartbeat_age,
                      path_and_timing_unchanged=True, motor_settings_changed=False)
        path = Path(directory)/'result.json'
        path.with_suffix('.tmp').write_text(json.dumps(result, indent=2)+'\n')
        path.with_suffix('.tmp').replace(path)


class StreamSession:
    """Small GUI handle; one selected-arm writer, joined before any handoff."""
    def __init__(self, bus, curve, side, step, *, geometries, directory,
                 factory=StreamMotors, shared=None):
        if getattr(bus, 'path_session', None) is not None:
            raise RuntimeError('Generated position writer already owns an arm')
        self.bus, self.side = bus, side
        self.directory = Path(directory)/('stream-'+side+'-'+uuid.uuid4().hex[:8])
        self.directory.mkdir(parents=True)
        ctx = mp.get_context('spawn')
        created = time.monotonic()
        self.cancel = ctx.Event(); self.ready = ctx.Event(); self.run = ctx.Event()
        self.heartbeat = ctx.Value('d', created, lock=False)
        q, tau = reference(curve, step, 0.)
        self.state = Progress(ctx, [0., -1. if tau is None else tau, *q, 0.])
        special = None
        motor = 7 if side == 'right' else 6
        owner = getattr(bus, f'{side}_joint{motor}_session', None) if factory is StreamMotors else None
        if owner is not None:
            if not owner.path_hold.is_set() or owner.hold_position is None or owner.status.error:
                raise RuntimeError('Hold strike owner before generated return')
            special = dict(motor=motor, hold=owner.path_hold,
                           goal=owner.path_target if side == 'right' else owner.goal)
        handles = [DupFd(fd) for fd in bus.locks] if factory is StreamMotors else []
        self.inputs_path=self.directory/'private_worker_inputs.pkl'
        fd=os.open(self.inputs_path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,'wb') as destination:
            pickle.dump((curve,step,geometries,bus.target),destination,protocol=pickle.HIGHEST_PROTOCOL)
        self.process = ctx.Process(target=_worker, args=(None, None, side, None, None,
            self.directory, handles, special, shared, self.cancel, self.heartbeat, self.ready, self.run,
            self.state, os.getpid(), created, factory,self.inputs_path), name='generated-plan-drums-controller', daemon=True)
        previous = {k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS')}
        try:
            os.environ.update({k:'1' for k in previous}); self.process.start()
        except BaseException:
            self.inputs_path.unlink(missing_ok=True);raise
        finally:
            for k,v in previous.items():
                if v is None:os.environ.pop(k,None)
                else:os.environ[k] = v
        bus.path_session = self
        self.parent_startup_s=time.monotonic()-created
        print(f'GENERATED {side.upper()}: plan_drums Controller at 200 Hz; initializing → ready → GUI acknowledgement; unchanged path/timing/settings',flush=True)

    @property
    def owns_feedback(self):return self.process.is_alive() and bool(self.state.read()[9])

    @property
    def queries_ready(self):
        # Read-only polling is active while awaiting the GUI's motion ack.
        # Keep owns_feedback's historical active-stream meaning unchanged.
        return self.process.is_alive() and self.ready.is_set()

    def poll(self):
        self.heartbeat.value = time.monotonic()
        if self.ready.is_set() and not self.cancel.is_set() and self.process.is_alive():
            self.run.set()
        values = self.state.read()
        done = not self.process.is_alive()
        result = None
        if done:
            self.process.join()
            path = self.directory/'result.json'
            result = json.loads(path.read_text()) if path.exists() else dict(success=False,
                error=f'Generated worker exited without handoff (exit {self.process.exitcode})')
        return dict(done=done, ready=self.ready.is_set(), acknowledged=self.run.is_set(), elapsed=values[0], tau=None if values[1]<0 else values[1],
                    target=np.array(values[2:9]), result=result)

    def stop(self):
        self.cancel.set()
        from .emergency import stop_writer
        stop_writer(self)  # position-only delegate; GUI retains powered ownership
        return self.poll()

    def detach(self):
        if self.process.is_alive():raise RuntimeError('Position writer still running; no competing handoff')
        if getattr(self.bus, 'path_session', None) is self:self.bus.path_session = None
        if getattr(self.bus, 'playback_session', None) is self:self.bus.playback_session = None
        self.inputs_path.unlink(missing_ok=True)  # Cancel before child load: no stale IPC payload.
