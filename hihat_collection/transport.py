"""Collection-local dual-arm transport; no off-center disable/mode handoffs."""
from collections import deque
import csv
import json
import math
import queue
import threading
import time

import numpy as np

from camera_playback.left_hold import LeftHoldDrive, LEFT_CENTER, LEFT_GRIPPER_TARGET
from camera_playback.mit_center_return import MitCenterReturn
from camera_playback.mit_strike import Joint7Worker, StrikeSettings
from centering.motors import parameter
from safe_zone.encoder import FRAME, joint_to_motor
from snare_lab.runner import _LeftJoint6Channel
from snare_lab.motion import build_method, stroke_timeout
from strike_lab.engine import StableWindow, limit_command
from x5_collection.transport import CollectionMotors
from .plan import SNARE_DEGREES, SNARE_MAX_DEGREES


def settled_center(states, history, goal, side, now):
    samples = [states.get((side, i)) for i in range(1, 9)]
    if any(s is None or now-s[2] > .1 or s[1] != 2 for s in samples):
        raise RuntimeError(f'REFUSED disable: {side} fresh running feedback required')
    if len(history) < 2 or history[-1][0]-history[0][0] < .6:
        raise RuntimeError(f'REFUSED disable: {side} center history too short')
    values = np.array([q for _, q in history])
    error = float(np.max(np.abs(values-goal)))
    span = float(np.max(np.ptp(values, axis=0)))
    current = np.array([s[0] for s in samples[:7]])
    if (now-history[-1][0] > .1 or error > math.radians(.2)
            or span > math.radians(.12) or np.max(np.abs(current-goal)) > math.radians(.2)):
        raise RuntimeError(f'REFUSED disable: {side} not settled at center')
    return dict(t=now, goal_rad=goal.tolist(), actual_rad=current.tolist(),
                max_error_deg=math.degrees(error), span_deg=math.degrees(span),
                settled_s=history[-1][0]-history[0][0])


class CollectionLeftDrive(LeftHoldDrive):
    def _send(self, side, frame):
        self.owner.guard_left_disable(frame)
        super()._send(side, frame)
        self.owner.log_left_command(frame)

    def set_positions(self, joints):
        if not self.active or len(joints) != 7:
            raise RuntimeError('Left arm is not active')
        if self.owner.left_mit is None:
            return super().set_positions(joints)
        if self.owner.snare_method is not None:
            raise RuntimeError('Left path cannot move during a snare strike')
        self.owner.left_mit.goal = -float(joints[5])
        for i, q in enumerate(joints, 1):
            if i != 6:
                self.send_control(parameter(i, 0x7016, joint_to_motor('left', i, q)))


class DataMotors(CollectionMotors):
    @classmethod
    def adopt(cls, bus, center, **kwargs):
        result = super().adopt(bus, center, left_hold=True, **kwargs)
        view = CollectionLeftDrive.__new__(CollectionLeftDrive)
        view.__dict__.update(result.left_drive.__dict__)
        view.owner = result
        result.left_drive = view
        result.left_history = deque()
        result.left_channel = result.left_mit = result.snare_method = None
        result.snare_done = []
        result.snare_window = None
        result.left_setup = False
        result.left_disabled_at = None
        result.left_tick_at = 0.
        result.snare_fault = None
        result.snare_trace = None
        return result

    def log_left_command(self, frame):
        cid, _, _ = FRAME.unpack(frame)
        self.command_file.write(json.dumps(dict(t=time.monotonic(), phase=self.phase,
            side='left', kind=(cid>>24)&31, motor=cid&255, frame=frame.hex()))+'\n')

    def left_center_evidence(self):
        return settled_center(self.states, self.left_history, LEFT_CENTER, 'left', time.monotonic())

    def guard_left_disable(self, frame):
        cid, _, _ = FRAME.unpack(frame)
        if (cid>>24)&31 != 4:
            return
        motor = cid & 255
        state = self.states.get(('left', motor))
        if state is not None and state[1] == 0:
            return  # Setup of a drive already disabled, never an active-arm relax.
        # A batch of disable writes changes running flags immediately. The first
        # write consumes fresh center evidence; later writes must be within 80 ms
        # and still observe the center pose (not a reusable permission).
        now = time.monotonic()
        if self.left_disabled_at is not None and now-self.left_disabled_at < .08:
            q = np.array([self.states['left', i][0] for i in range(1, 8)])
            if self.fresh() and np.max(np.abs(q-LEFT_CENTER)) <= math.radians(.2):
                return
        proof = self.left_center_evidence()
        with (self.directory/'left_disable_audit.jsonl').open('a') as out:
            out.write(json.dumps(dict(motor=motor, **proof))+'\n')
        self.left_disabled_at = now

    def start_left_mit_at_center(self, tuning, anchor_displayed):
        proof = self.left_center_evidence()
        self.snare_parameters, self.snare_rules, self.snare_limits = tuning
        self.snare_anchor = -float(anchor_displayed)
        owner = self
        class GuardedChannel(_LeftJoint6Channel):
            def send(self, frame):
                owner.guard_left_disable(frame)
                result = super().send(frame)
                owner.log_left_command(frame)
                return result
        self.left_channel = GuardedChannel(self.sockets['left'].getsockname()[0])
        helper = Joint7Worker('can1', -float(self.states['left', 6][0]), -.75, .75,
                             StrikeSettings(), queue.Queue(), threading.Event(), lambda s: None,
                             motor=6, sign=-1.)
        # Single center-only MIT transition; J6 stays powered in MIT through
        # playback, all strikes, reversal and final center.
        self.left_monitor = None
        self.left_setup = True
        try:
            helper._prepare(self.left_channel)
        finally:
            self.left_setup = False
        sample = self.left_channel.sample
        self.left_mit = MitCenterReturn(sample.position, 0., 0., time.monotonic())
        self.left_tick_at = 0.
        self.snare_trace_file = (self.directory/'j6.csv').open('w', buffering=65536)
        self.snare_trace = csv.writer(self.snare_trace_file)
        self.snare_trace.writerow(['monotonic_s','sample_at','phase','position_rad','velocity_rad_s','torque_nm','block'])
        (self.directory/'left_center_mode_transfer.json').write_text(json.dumps(proof, indent=2)+'\n')

    def start_snare(self, block):
        if self.snare_method is not None or self.snare_fault or self.left_mit is None:
            raise RuntimeError('Snare not ready')
        s = self.left_channel.sample
        if time.monotonic()-s.at > .05 or s.state != 2 or abs(s.position-self.snare_anchor) > math.radians(.2) or abs(s.velocity) > .15:
            raise RuntimeError('Snare anchor must be stationary before strike')
        self.snare_method = build_method(self.snare_anchor, self.left_mit.bias,
                                        self.snare_parameters, SNARE_DEGREES, self.snare_limits)
        self.snare_started = time.monotonic()
        self.snare_method.start(self.snare_started, [s])
        self.snare_window = StableWindow(self.snare_anchor, self.snare_rules)
        self.snare_peak = 0.
        self.snare_block = block
        self.snare_last_send = None
        self.snare_last_torque = self.left_mit.bias
        self.snare_returned = False

    def cancel_snare(self):
        # No mode switch and no disable. Continue a bounded powered return.
        self.snare_method = None
        if self.left_mit is not None:
            s = self.left_channel.sample
            self.left_mit = MitCenterReturn(s.position, s.position,
                                           self.left_mit.bias, time.monotonic())
        self.snare_fault = None

    def _tick_left(self):
        if self.left_mit is None or self.left_setup or not self.active:
            return
        self.left_channel.receive()
        now = time.monotonic()
        if now-self.left_tick_at < .002:
            return
        s = self.left_channel.sample
        timeout = self.snare_limits.feedback_timeout if self.snare_method else .3
        if s is None or now-s.at > timeout or s.state != 2:
            raise RuntimeError('Left J6 fresh running MIT feedback lost')
        if not -.75 <= s.position <= .75:
            raise RuntimeError('Left J6 exceeded joint range')
        method = self.snare_method
        if method:
            if self.snare_last_send and now-self.snare_last_send > self.snare_limits.control_timeout:
                raise RuntimeError('Snare control deadline missed')
            depth = math.degrees(self.snare_anchor-s.position)
            if depth > SNARE_MAX_DEGREES or depth < -self.snare_limits.upper_excursion_deg:
                raise RuntimeError('Snare measured excursion exceeded collection corridor')
            self.snare_peak = max(self.snare_peak, depth)
            command = method.update(s, now)
            if method.done:
                self.snare_window.add(s)
                command = method.hold()
            command, self.snare_last_torque, _ = limit_command(command, s, now,
                self.snare_anchor, self.snare_limits, self.snare_last_send, self.snare_last_torque)
            if not self.snare_anchor-math.radians(SNARE_MAX_DEGREES) <= command.position <= self.snare_anchor+math.radians(self.snare_limits.upper_excursion_deg):
                raise RuntimeError('Snare command exceeded collection corridor')
            phase = method.phase
            if method.done and self.snare_window.ready:
                self.snare_done.append(dict(block=self.snare_block, started=self.snare_started,
                    finished=now, commanded_deg=SNARE_DEGREES, peak_deg=self.snare_peak))
                self.left_mit = MitCenterReturn(s.position, self.snare_anchor,
                                               self.left_mit.bias, now)
                self.snare_method = None
            elif now-self.snare_started > stroke_timeout(method, self.snare_limits):
                raise RuntimeError('Snare failed to settle after strike')
            self.snare_last_send = now
        else:
            command = self.left_mit.update(s.position, now)
            phase = 'powered_path_hold'
        self.left_channel.send(command.frame(6, -1.))
        self.left_tick_at = now
        self.snare_trace.writerow([now,s.at,phase,s.position,s.velocity,s.torque,getattr(self,'snare_block',None)])

    def poll(self):
        # Keep ride worker heartbeat alive even while playing/centering left.
        if self.right_joint7_session is not None:
            self.right_joint7_session.status
        super().poll()
        now = time.monotonic()
        if self.fresh():
            q = np.array([self.states['left', i][0] for i in range(1, 8)])
            self.left_history.append((now, q))
            while self.left_history and now-self.left_history[0][0] > .65:
                self.left_history.popleft()
        self._tick_left()

    def relax(self):
        left_proof = self.left_center_evidence()
        right_proof = self.center_evidence()
        if self.snare_method is not None or self.right_joint7_session is not None:
            raise RuntimeError('REFUSED relax: strike worker still owns an arm')
        (self.directory/'both_center_before_relax.json').write_text(json.dumps(
            dict(left=left_proof, right=right_proof), indent=2)+'\n')
        super().relax()
        self.left_mit = None

    def close(self):
        if self.left_channel is not None:
            self.left_channel.close()
        if self.snare_trace is not None:
            self.snare_trace_file.close()
        super().close()
