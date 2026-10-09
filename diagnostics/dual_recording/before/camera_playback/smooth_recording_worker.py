"""Isolated 200 Hz recording worker; never enables, centers, or relaxes an arm."""
import csv
from datetime import datetime, timezone
import json
import math
import multiprocessing as mp
from multiprocessing.reduction import DupFd
import os
from pathlib import Path
import signal
import socket
import struct
import time
import uuid

import numpy as np

from centering.motors import Motors, packet
from safe_zone.encoder import FRAME
from smooth_playback.hardware import AuditedMotors, CaptureSocket
from smooth_playback.runner import Runner, Cancelled
from smooth_playback.trajectory import ROOT
from .playback_transport import RetrySocket


class RecordingMotors(AuditedMotors):
    """Independent receive sockets, with lock ownership delegated by the GUI.

    A duplicated lock descriptor retains the parent's existing flock (not a
    competing lock acquisition). Separate CAN sockets receive independent
    copies: the GUI and worker never steal each other's feedback/readbacks.
    The send allowlist excludes every enable, mode write, disable, gripper,
    left-arm command, current change, and persistent save.
    """
    def __init__(self, directory, lock_handles):
        self.directory = Path(directory)
        self.phase = 'CONNECT'
        self.center_permission = False
        self.parameter_values, self.gains_original = {}, {}
        self.gains_restored = True
        self.sockets, self.locks = {}, []
        self.tx_count = 0
        self.control_side, self.control_gripper = 'right', True
        self.states, self.modes = {}, {}
        self.last_query = self.right_query = self.left_query = 0.
        self.active = False
        self.isolated_motors = set()
        self.right_joint7_query_period = None
        self.last_right_joint7_query = 0.
        self.right_joint7_session = None
        self.feedback_file = open(self.directory/'feedback.csv', 'w', buffering=128*1024)
        self.command_file = open(self.directory/'commands.jsonl', 'w', buffering=64*1024)
        self.feedback = csv.writer(self.feedback_file)
        self.feedback.writerow(['monotonic_s', 'phase', 'side', 'motor', 'position_rad',
                                'state', 'velocity_raw', 'torque_raw', 'temperature_c'])
        try:
            if len(lock_handles) != 2:
                raise RuntimeError('Playback requires both locks from the owning GUI')
            for handle in lock_handles:
                self.locks.append(handle.detach())
            for side, interface in (('left', 'can1'), ('right', 'can0')):
                sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.sockets[side] = CaptureSocket(RetrySocket(sock), side, self)
                sock.setsockopt(socket.SOL_CAN_RAW, 2, struct.pack('=I', 0x1fffffff))
                sock.bind((interface,))
                sock.setblocking(False)
        except BaseException:
            self.close()
            raise

    def feedback_controlled(self, side, motor):
        # The GUI now holds the left arm. Accept its powered feedback, but
        # retain the right-only write allowlist below; this worker never owns it.
        return side == 'left' or super().feedback_controlled(side, motor)

    def _send(self, side, frame):
        cid, dlc, data = FRAME.unpack(frame)
        kind, motor = (cid >> 24) & 31, cid & 255
        index = int.from_bytes(data[:2], 'little')
        permitted = (kind == 17 and index in (0x7005, 0x701e)
                     or kind == 18 and index in (0x7016, 0x7017, 0x701e))
        if side != 'right' or motor not in range(1, 8) or dlc != 8 or not permitted:
            raise RuntimeError('Recording worker may only read CSP mode and use right J1-J7 position/speed/gain parameters')
        super()._send(side, frame)

    def center(self, *args, **kwargs):
        raise RuntimeError('The GUI alone owns centering and enabling')

    def relax(self):
        raise RuntimeError('The GUI alone owns relaxation')


PHASES = ('STARTING', 'START HOLD', 'TEMPORARY POSITION GAINS', 'PLAYBACK',
          'ENDPOINT SOFT HOLD', 'ENDPOINT GAIN RESTORATION', 'ENDPOINT HOLD',
          'ENDPOINT VERIFIED', 'CANCEL HOLD', 'ERROR')


class RecordingRunner(Runner):
    """Reuse the tested clock, feedback guards, settling, and gain ramp only."""
    def __init__(self, recording, directory, cancel, heartbeat, progress, parent_pid):
        super().__init__(recording.geometry, recording.smooth_motion, directory,
                         gripper=True, video=False)
        self.cancel_event, self.heartbeat, self.progress = cancel, heartbeat, progress
        self.parent_pid = parent_pid

    def cancellation(self):
        if self.recovering:
            return
        if self.cancel_event.is_set():
            raise Cancelled('Recording worker stopped; returning control to the existing GUI workflow')
        if os.getppid() != self.parent_pid or time.monotonic()-self.heartbeat.value > 2.:
            raise RuntimeError('Recording GUI heartbeat lost')

    def event(self, phase, **data):
        super().event(phase, **data)
        if phase in PHASES:
            self.progress[0] = PHASES.index(phase)

    def tick(self, desired, t, lateness=0.):
        super().tick(desired, t, lateness)
        self.progress[1] = t

    def guarded_restore(self, target):
        last_command = 0.
        def hold():
            nonlocal last_command
            self.cancellation()
            q = self.poll()
            if np.max(np.abs(q-target)) > math.radians(.25):
                raise RuntimeError('Gain ramp left the 0.25 degree hold envelope')
            now = time.monotonic()
            if now-last_command >= .01:
                self.bus.set_positions(target)
                last_command = now
        self.bus.restore_position_gains(ramp_seconds=.5, tick=hold)

    def play_only(self):
        deadline = time.monotonic()+3.
        while not self.bus.fresh():
            self.cancellation()
            self.bus.poll()
            if time.monotonic() > deadline:
                raise RuntimeError('Recording worker feedback startup timeout')
            time.sleep(.002)
        self.enabled = True
        q = self.poll()
        if np.max(np.abs(q-self.trajectory.first)) > math.radians(3.1):
            raise RuntimeError('Recording start was not reached')
        # Confirm existing CSP mode; do not change/re-enable any motor.
        for i in range(1, 8):
            self.bus.send_control(packet(17, i, struct.pack('<H6x', 0x7005)))
        deadline = time.monotonic()+.5
        while len(self.bus.modes) != 7:
            self.cancellation()
            self.poll()
            if time.monotonic() > deadline:
                raise RuntimeError('Recording worker CSP readback timeout')
            time.sleep(.002)
        if any(mode != 5 for mode, _ in self.bus.modes.values()):
            raise RuntimeError('Recording worker requires existing CSP mode')
        self.bus.active = True
        self.g.check_line(q, self.trajectory.first, measured_start=True)
        self.settle(self.trajectory.first, 'START HOLD')
        self.cancellation()
        self.event('TEMPORARY POSITION GAINS')
        self.bus.apply_position_gain_cap(10.)
        self.bus.set_playback_speeds(self.trajectory.metadata['firmware_joint_speeds_rad_s'])
        self.event('PLAYBACK', duration_s=self.trajectory.duration)
        self.stream(self.trajectory.duration, self.trajectory.at, 200)
        self.settle(self.trajectory.last, 'ENDPOINT SOFT HOLD')
        self.event('ENDPOINT GAIN RESTORATION', ramp_s=.5)
        self.guarded_restore(self.trajectory.last)
        self.endpoint = self.settle(self.trajectory.last, 'ENDPOINT HOLD',
                                    math.radians(.05), math.radians(.066))
        self.bus.set_right_arm_speed(.4)
        self.event('ENDPOINT VERIFIED', **self.endpoint)


def _worker(recording, directory, cancel, heartbeat, progress, parent_pid,
            lock_handles, bus_factory=RecordingMotors):
    # Terminal SIGINT belongs to the GUI. Its cancellation event requests a
    # bounded handoff, rather than KeyboardInterrupt halfway through a gain write.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    runner = RecordingRunner(recording, directory, cancel, heartbeat, progress, parent_pid)
    error = cleanup_error = None
    try:
        runner.bus = bus_factory(Path(directory), lock_handles)
        runner.play_only()
    except BaseException as exc:
        error = f'{type(exc).__name__}: {exc}'
        runner.event('ERROR', error=error)
        runner.recovering = True
        if runner.bus and runner.bus.active:
            try:
                # Stop advancement and hold the *fresh measured* pose before
                # restoring gains. Never issue a blind move or relax here.
                q = runner.poll()
                runner.bus.set_right_arm_speed(.4)
                runner.bus.set_positions(q)
                # State replies already in flight can still describe the last
                # moving target. Establish a settled hold before applying the
                # tight gain-ramp envelope; otherwise a safe stop can falsely
                # fail restoration merely because feedback was one tick old.
                runner.settle(q, 'CANCEL HOLD')
                runner.guarded_restore(q)
            except BaseException as cleanup:
                cleanup_error = str(cleanup)
    finally:
        result = dict(success=error is None, error=error, cleanup_error=cleanup_error,
                      endpoint=runner.endpoint,
                      gains_restored=getattr(runner.bus, 'gains_restored', True),
                      original_gains=getattr(runner.bus, 'gains_original', {}),
                      method=recording.smooth_motion.metadata,
                      transport={side: dict(retries=s.retries, maximum_wait_s=s.maximum_wait_s)
                                 for side, s in getattr(runner.bus, 'sockets', {}).items()})
        try:
            if runner.bus:
                runner.bus.close()  # closes sockets/locks/logs, NEVER disables
        finally:
            runner.trace_file.close()
            temporary = Path(directory)/'result.tmp'
            temporary.write_text(json.dumps(result, indent=2)+'\n')
            temporary.replace(Path(directory)/'result.json')


class RecordingSession:
    """GUI handle. stop() always joins the writer before any next-stage command."""
    def __init__(self, recording, bus, *, directory=None, bus_factory=RecordingMotors):
        if (not bus.active or bus.control_side != 'right' or not bus.control_gripper
                or bus.isolated_motors or bus.right_joint7_session is not None):
            raise RuntimeError('Smooth recording requires exclusive healthy right-arm CSP ownership')
        if bus_factory is RecordingMotors and (not isinstance(bus, Motors) or len(bus.locks) != 2):
            raise RuntimeError('Smooth recording requires the GUI motor bus and its two CAN locks')
        self.directory = Path(directory) if directory else ROOT/'playback_results'/'start_beat'/(
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
        self.directory.mkdir(parents=True, exist_ok=True)
        (self.directory/'trajectory.json').write_text(json.dumps(recording.smooth_motion.metadata, indent=2)+'\n')
        context = mp.get_context('spawn')
        self.cancel = context.Event()
        self.heartbeat = context.Value('d', time.monotonic(), lock=False)
        self.progress = context.Array('d', [0., 0.], lock=False)
        handles = [DupFd(fd) for fd in bus.locks] if bus_factory is RecordingMotors else []
        self.process = context.Process(target=_worker, args=(recording, self.directory,
            self.cancel, self.heartbeat, self.progress, os.getpid(), handles, bus_factory),
            name='smooth-recording', daemon=True)
        # Bound BLAS workers in the child without changing the GUI's environment.
        previous = {k: os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS')}
        try:
            os.environ.update({k: '1' for k in previous})
            self.process.start()
        finally:
            for k, value in previous.items():
                if value is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = value

    def _result(self):
        path = self.directory/'result.json'
        if path.exists():
            return json.loads(path.read_text())
        return dict(success=False, error=f'Recording worker exited without verified handoff (exit {self.process.exitcode})',
                    cleanup_error='Original gains not verified', gains_restored=False)

    @property
    def owns_feedback(self):
        # START HOLD is entered only after all sixteen feedback states and CSP
        # readbacks have arrived. Keep GUI queries alive during child startup.
        return self.process.is_alive() and self.progress[0] >= PHASES.index('START HOLD')

    def poll(self):
        self.heartbeat.value = time.monotonic()
        done = not self.process.is_alive()
        if done:
            self.process.join()
        return dict(done=done, elapsed=float(self.progress[1]),
                    phase=PHASES[int(self.progress[0])],
                    result=self._result() if done else None)

    def stop(self):
        self.cancel.set()
        self.process.join(15.)  # bounded 12 s settle + readbacks and gain ramp
        if self.process.is_alive():
            # An unresponsive owned writer must not race the GUI's recovery or
            # disable commands. This is an error, never a successful handoff.
            self.process.terminate()
            self.process.join(2.)
            if self.process.is_alive():
                self.process.kill()
                self.process.join()
            return dict(success=False, error='Recording worker did not stop promptly',
                        cleanup_error='Original gains not verified', gains_restored=False)
        return self._result()
