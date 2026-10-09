"""Playback-local CAN backpressure handling and optional no-strike relax gate."""
from collections import deque
import errno
import json
import math
import time

import numpy as np

from centering.motors import Motors, packet, parameter, CURRENT, GRIPPER_CURRENT, MOTOR_RUNNING_STATE
from safe_zone.encoder import FRAME, joint_to_motor
from .mit_center_return import MitCenterReturn
from .left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET, LeftCenterMonitor, left_setup_view


class RetrySocket:
    """Retry transient queue pressure, never discard a motor command.

    In particular Motors.poll()'s state requests previously bypassed its command
    retry loop. With a ten-frame kernel TX queue, the GUI's query burst could
    collide with the independent 200 Hz playback writer and raise ENOBUFS.
    """
    def __init__(self, sock):
        self.sock = sock
        self.retries = 0
        self.maximum_wait_s = 0.
        self.emergency_latched = False

    def __getattr__(self, name):
        return getattr(self.sock, name)

    def send(self, frame):
        if self.emergency_latched and ((FRAME.unpack(frame)[0] >> 24) & 31) not in (2, 4, 17):
            raise RuntimeError('Motor motion blocked after emergency stop; restart required')
        start = time.monotonic()
        while True:
            try:
                sent = self.sock.send(frame)
                if sent != len(frame):
                    raise RuntimeError('Short CAN write')
                self.maximum_wait_s = max(self.maximum_wait_s, time.monotonic()-start)
                return sent
            except OSError as exc:
                if exc.errno not in (errno.ENOBUFS, errno.EAGAIN) or time.monotonic()-start >= .020:
                    raise
                self.retries += 1
                time.sleep(.0005)


class PlaybackMotors(Motors):
    @classmethod
    def adopt(cls, bus, center, strict_center_relax=False, directory=None, left_hold=False):
        """Transfer the GUI's already-open bus; do not open/enable new hardware."""
        result = cls.__new__(cls)
        result.__dict__.update(bus.__dict__)
        result.sockets = {side: RetrySocket(sock) for side, sock in bus.sockets.items()}
        bus.sockets = {}
        bus.locks = []
        result.playback_session = None
        result.strict_center_relax = strict_center_relax
        result.require_center_before_relax = False
        result.emergency_relax_reason = None
        result.center_goal = np.asarray(center).copy()
        result.center_history = deque()
        result.audit_directory = directory
        result.last_center_evidence = None
        result.mit_center_return = None
        result.left_hold_enabled = left_hold
        result.left_drive = left_setup_view(result) if left_hold else None
        result.left_monitor = None
        result.left_playback_active = False
        result.center_disabled = {}
        result.dual_center_history = None
        result.snare_center_history = deque()
        result.left_disable_at = None
        return result

    def snare_center_evidence(self):
        now = time.monotonic()
        h = self.snare_center_history
        if (not self.fresh() or len(h) < 2 or h[-1][0]-h[0][0] < .6
                or now-h[-1][0] > .1):
            raise RuntimeError('Left center not settled yet')
        q = np.array([self.states['left', i][0] for i in range(1, 8)])
        values = np.array([q for _, q in h])
        if (any(self.states['left', i][1] != MOTOR_RUNNING_STATE for i in range(1, 9))
                or np.max(np.abs(q-LEFT_CENTER)) > math.radians(.2)
                or np.max(np.abs(values-LEFT_CENTER)) > math.radians(.2)
                or np.max(np.ptp(values, axis=0)) > math.radians(.12)):
            raise RuntimeError('Left center not settled yet')
        return dict(at=now, max_error_deg=math.degrees(float(np.max(np.abs(values-LEFT_CENTER)))))

    def guard_left_disable(self, frame):
        if not getattr(self, 'require_center_before_relax', False) or self.emergency_relax_reason:
            return
        cid, _, _ = FRAME.unpack(frame)
        if (cid >> 24) & 31 != 4:
            return
        state = self.states.get(('left', cid & 255))
        if state is not None and state[1] == 0:
            return  # Initial setup of an already disabled drive.
        now = time.monotonic()
        q = np.array([self.states['left', i][0] for i in range(1, 8)])
        if (self.left_disable_at is not None and now-self.left_disable_at < .08
                and self.fresh() and np.max(np.abs(q-LEFT_CENTER)) <= math.radians(.2)):
            return
        self.snare_center_evidence()
        self.left_disable_at = now

    def feedback_controlled(self, side, motor):
        return (side == 'left' and getattr(self, 'left_hold_enabled', False)
                or super().feedback_controlled(side, motor))

    def mode_feedback(self, side, motor, mode, now):
        if side == 'left' and getattr(self, 'left_hold_enabled', False):
            self.left_drive.modes[motor] = (mode, now)
        else:
            super().mode_feedback(side, motor, mode, now)

    def center(self, target=None, gripper_target=None, confirm_enabled=False):
        if not self.left_hold_enabled:
            yield from super().center(target, gripper_target, confirm_enabled)
            return
        self.left_monitor = None
        self.left_playback_active = False
        self.center_disabled = {}
        self.dual_center_history = None
        self.emergency_relax_reason = None
        self.center_history.clear()
        self.snare_center_history.clear()
        self.left_disable_at = None
        # Start the inherited generator first: it validates the target and
        # fresh, disabled feedback for ALL 16 drives before any enable.
        right = super().center(target, gripper_target, confirm_enabled=True)
        next(right)
        left = self.left_drive
        left.active = True
        live = [self.states['left', i][0] for i in range(1, 8)]

        def prepare_left():
            for i, (q, current) in enumerate(zip(
                    live + [LEFT_GRIPPER_TARGET], CURRENT + (GRIPPER_CURRENT,)), 1):
                yield from left._prepare_center_motor_confirmed(i, q, current)

        pending = [right, prepare_left()]
        while pending:
            for setup in pending[:]:
                try:
                    next(setup)
                except StopIteration:
                    pending.remove(setup)
            yield
        if not self.fresh() or any(state[1] != MOTOR_RUNNING_STATE
                                  for state in self.states.values()):
            raise RuntimeError('Both arms must confirm running before simultaneous centering')
        # Both arms held their live joint poses during setup. Dispatch BOTH
        # goals in this same GUI tick, not after either arm has arrived.
        left.set_positions(LEFT_CENTER)
        self.set_positions([0.] * 7 if target is None else target)
        self.left_monitor = LeftCenterMonitor(time.monotonic())

    def left_center_ready(self):
        return (not self.left_hold_enabled or self.left_monitor is not None
                and np.array_equal(self.left_monitor.goal, LEFT_CENTER)
                and self.left_monitor.ready and not self.left_monitor.fault)

    def set_left_hold(self, target):
        if not self.active or not self.left_hold_enabled or 'left' in self.center_disabled:
            raise RuntimeError('Left arm is not active')
        self.left_playback_active = False
        self.left_drive.set_positions(target)
        self.left_monitor = LeftCenterMonitor(time.monotonic(), target)

    def left_hold_ready(self):
        return self.left_monitor is not None and self.left_monitor.ready and not self.left_monitor.fault

    def begin_left_playback(self):
        if not self.left_hold_ready():
            raise RuntimeError('Left recording start not reached')
        self.left_monitor = None
        self.left_playback_active = True

    def set_left_positions(self, target):
        if not self.left_playback_active:
            raise RuntimeError('No left recording owns the targets')
        self.left_drive.set_positions(target)

    def begin_dual_center(self, right_start=None, left_target=None):
        self.dual_center_history = {side: deque() for side in ('left', 'right')}
        self.center_history.clear()
        if 'left' not in self.center_disabled:
            self.set_left_hold(LEFT_CENTER if left_target is None else left_target)
        if 'right' not in self.center_disabled:
            if right_start is None:
                self.set_positions(self.center_goal)
            else:
                right_start()

    def disable_centered_side(self, side):
        if side not in ('left', 'right'):
            raise ValueError('Unknown arm')
        if side in self.center_disabled:
            return
        session = self.playback_session
        left_return_only = (side == 'right' and getattr(session, 'side', None) == 'left'
                            and getattr(session, 'center_return', False) is True)
        if not self.fresh() or (session is not None and not left_return_only):
            raise RuntimeError('Fresh feedback and exclusive ownership required before relax')
        history = self.dual_center_history[side]
        now = time.monotonic()
        goal = LEFT_CENTER if side == 'left' else self.center_goal
        actual = np.array([self.states[side, i][0] for i in range(1, 8)])
        if (any(self.states[side, i][1] != MOTOR_RUNNING_STATE for i in range(1, 9))
                or np.max(np.abs(actual-goal)) > math.radians(.20)):
            raise RuntimeError('Center not settled yet')
        if side == 'left' and abs(self.states[side, 8][0]-LEFT_GRIPPER_TARGET) > math.radians(1.):
            raise RuntimeError('Left gripper lost its closed hold')
        if len(history) < 2 or history[-1][0] - history[0][0] < .6:
            raise RuntimeError('Center not settled yet')
        values = np.array([q for _, q in history])
        if (now - history[-1][0] > .1 or np.max(np.abs(values-goal)) > math.radians(.20)
                or np.max(np.ptp(values, axis=0)) > math.radians(.12)):
            raise RuntimeError('Center not settled yet')
        # Each side is disabled only after its OWN measured center evidence.
        if side == 'left':
            snare = getattr(self, 'left_joint6_session', None)
            if snare is not None:
                if not snare.finished:
                    raise RuntimeError('Snare still owns an active stroke')
                snare.stop()  # Join writer BEFORE disabling centered J6.
                if not self.fresh():
                    raise RuntimeError('Fresh feedback required after snare worker join')
            self.left_drive.relax()
            self.left_monitor = None
        elif side == 'right':
            for _ in range(3):
                for i in range(1, 9):
                    self.send_control(packet(4, i))
            self.mit_center_return = None
        else:
            raise ValueError('Unknown arm')
        self.center_disabled[side] = now

    def poll(self):
        snare = getattr(self, 'left_joint6_session', None)
        if snare is not None:
            snare.status  # Heartbeat also during the two recording workers.
        session = self.playback_session
        if (session is not None and session.owns_feedback
                and not getattr(session, 'center_return', False)):
            # The child's independent sockets supply fresh feedback to BOTH
            # receivers. Drain ours normally, but don't duplicate its queries.
            self.last_query = time.monotonic()
        super().poll()
        if hasattr(self, 'snare_center_history') and self.active and self.fresh():
            now = time.monotonic()
            self.snare_center_history.append((now, np.array([self.states['left', i][0] for i in range(1, 8)])))
            while self.snare_center_history and now-self.snare_center_history[0][0] > .65:
                self.snare_center_history.popleft()
        if getattr(self, 'left_playback_active', False) and self.active:
            if any(self.states['left', i][1] != MOTOR_RUNNING_STATE for i in range(1, 9)):
                raise RuntimeError('Left recording motor stopped')
            if abs(self.states['left', 8][0] - LEFT_GRIPPER_TARGET) > math.radians(1.):
                raise RuntimeError('Left gripper did not maintain centered closed position')
        if getattr(self, 'dual_center_history', None) is not None and self.active and self.fresh():
            now = time.monotonic()
            for side, history in self.dual_center_history.items():
                if side in self.center_disabled:
                    continue
                history.append((now, np.array([self.states[side, i][0] for i in range(1, 8)])))
                while history and now-history[0][0] > .65:
                    history.popleft()
        if getattr(self, 'left_monitor', None) is not None and self.active:
            self.left_monitor.update(self.states, time.monotonic())
        if getattr(self, 'mit_center_return', None) is not None and self.active and self.fresh():
            command = self.mit_center_return.update(self.states['right', 7][0], time.monotonic())
            self.set_right_joint7_mit(command.position, command.velocity,
                                      command.kp, command.kd, command.torque)
        if (self.strict_center_relax or self.require_center_before_relax) and self.fresh():
            now = time.monotonic()
            q = np.array([self.states['right', i][0] for i in range(1, 8)])
            self.center_history.append((now, q))
            while self.center_history and now-self.center_history[0][0] > .65:
                self.center_history.popleft()

    def begin_mit_center_return(self, bias=0.):
        if not self.require_center_before_relax or not self.active or not self.fresh():
            raise RuntimeError('Fresh active normal hardware required for MIT center return')
        if self.right_joint7_session is not None:
            raise RuntimeError('Join J7 worker before transferring center ownership')
        if self.right_joint7_mode_readback() != 0:
            raise RuntimeError('MIT mode must be confirmed before center return')
        self.mit_center_return = MitCenterReturn(self.states['right', 7][0],
                                                self.center_goal[6], bias, time.monotonic())

    def set_positions(self, joints):
        if getattr(self, 'mit_center_return', None) is None:
            return super().set_positions(joints)
        if not self.active or len(joints) != 7:
            raise RuntimeError('Invalid center return command')
        # J7 remains powered in MIT until the WHOLE arm is centered. Its slow
        # controller runs on each poll, not the 200 ms inherited CSP cadence.
        for i, q in enumerate(joints[:6], 1):
            self.send_control(parameter(i, 0x7016, joint_to_motor('right', i, q)))

    def center_evidence(self):
        if not self.fresh() or len(self.center_history) < 2:
            raise RuntimeError('REFUSED relax: fresh settled center is not verified')
        now = time.monotonic()
        history = self.center_history
        values = np.array([q for _, q in history])
        error = float(np.max(np.abs(values-self.center_goal)))
        span = float(np.max(np.ptp(values, axis=0)))
        current = np.array([self.states['right', i][0] for i in range(1, 8)])
        if (now-history[-1][0] > .1 or history[-1][0]-history[0][0] < .6
                or error > math.radians(.20) or span > math.radians(.12)
                or np.max(np.abs(current-self.center_goal)) > math.radians(.20)):
            raise RuntimeError('REFUSED relax: arm has not settled at center for 0.6 seconds')
        return dict(t=now, goal_rad=self.center_goal.tolist(), actual_rad=current.tolist(),
                    max_error_deg=math.degrees(error), span_deg=math.degrees(span),
                    settled_s=history[-1][0]-history[0][0])

    def _send(self, side, frame):
        cid, _, data = FRAME.unpack(frame)
        if self.strict_center_relax and ((cid >> 24) & 31 == 1 or
                ((cid >> 24) & 31 == 18 and data[:2] == b'\x05\x70' and data[4] != 5)):
            raise RuntimeError('Recording-only mode forbids MIT/strike mode commands')
        if self.strict_center_relax and not self.emergency_relax_reason and (cid >> 24) & 31 == 4:
            state = self.states.get((side, cid & 255))
            # Initial CSP setup may send disable to an ALREADY relaxed motor.
            if state is None or state[1] != 0:
                if self.playback_session is not None:
                    raise RuntimeError('REFUSED relax: playback still owns the arm')
                self.last_center_evidence = self.center_evidence()
                if self.audit_directory:
                    with open(self.audit_directory/'disable_audit.jsonl', 'a') as out:
                        out.write(json.dumps(dict(motor=cid & 255, **self.last_center_evidence))+'\n')
        if (self.require_center_before_relax and not self.emergency_relax_reason
                and (cid >> 24) & 31 == 4):
            state = self.states.get((side, cid & 255))
            if state is None or state[1] != 0:
                self.center_evidence()
        return super()._send(side, frame)

    def relax(self):
        guarded = self.strict_center_relax or getattr(self, 'require_center_before_relax', False)
        emergency = getattr(self, 'emergency_relax_reason', None)
        if guarded and self.active and not emergency:
            if self.playback_session is not None:
                raise RuntimeError('REFUSED relax: playback still owns the arm')
            self.last_center_evidence = self.center_evidence()
            print('CENTER VERIFIED BEFORE RELAX: '+json.dumps(self.last_center_evidence), flush=True)
        if emergency:
            print('MAJOR FAULT EMERGENCY RELAX: '+emergency, flush=True)
        # Keep the left native position hold through right-arm centering and
        # worker shutdown. Only an accepted whole-program relax releases it.
        failures = []
        try:
            result = super().relax()
        except Exception as exc:
            failures.append(exc)
        if getattr(self, 'left_hold_enabled', False):
            try:
                self.left_drive.relax()
            except Exception as exc:
                failures.append(exc)
        self.left_monitor = None
        if failures:
            self.active = True  # Never advertise all-disabled after partial delivery.
            raise RuntimeError('Arm disable delivery failed; use physical power cutoff: '
                               + '; '.join(map(str, failures)))
        self.mit_center_return = None
        return result

    def close(self):
        if self.audit_directory:
            (self.audit_directory/'transport.json').write_text(json.dumps({
                side: dict(retries=s.retries, maximum_wait_s=s.maximum_wait_s)
                for side, s in self.sockets.items()}, indent=2)+'\n')
        super().close()
