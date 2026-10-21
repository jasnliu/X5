"""Finite hi-hat angle search. No Tk, CAN, notification or model dependencies.

Only timestamped hi-hat HITs select an angle. Encoder feedback verifies travel,
not sound. Every trial finishes its full hold/open cycle before selection.
"""
from __future__ import annotations

import math

from .hihat import CALIBRATION_MIN_DEGREES, CALIBRATION_MAX_DEGREES

ANGLES = tuple(range(CALIBRATION_MIN_DEGREES, CALIBRATION_MAX_DEGREES + 1, 5))
CLOSE_SECONDS = 2.0
OPEN_SECONDS = 2.0
MOVEMENT_TIMEOUT = 2.5
ACK_TIMEOUT = 1.0
FEEDBACK_TIMEOUT = .6
AUDIO_BACKLOG_TIMEOUT = 3.0
REACHED_COUNTS = 2


class HiHatCalibration:
    def __init__(self, controller, log=lambda row: None):
        self.controller = controller
        self.log = log
        self.phase = 'WAITING'
        self.angle = None
        self.selected_angle = None
        self.failure = None
        self.close_at = None
        self.open_at = None
        self.hit = None
        self.finalized_at = None
        self.progress_at = None
        self.command_at = None
        self.reached_at = None
        self.hold_until = None
        self.deadline = None
        self.open_settle_position = None

    @property
    def ready(self):
        return self.phase == 'READY'

    @property
    def busy(self):
        return self.phase not in {'WAITING', 'READY', 'FAULT'}

    def event(self, kind, now, **extra):
        self.log(dict(kind=kind, at=now, phase=self.phase, angle=self.angle, **extra))

    def start(self, now):
        if self.phase != 'WAITING' or self.failure:
            raise RuntimeError('Hi-hat calibration may only start once per program launch')
        self.controller.calibrated = False
        self._open(now, 'INITIAL OPENING')

    def _open(self, now, phase='OPENING'):
        self.controller.calibration_edge(False)
        self.command_at = now
        self.open_at = now
        self.reached_at = None
        self.open_settle_position = None
        self.deadline = now + MOVEMENT_TIMEOUT
        self.phase = phase
        self.event('open', now)

    def _configure(self, now, angle):
        if angle not in ANGLES:
            raise RuntimeError('Attempt outside the approved 90..115 degree search')
        self.angle = angle
        self.close_at = self.open_at = None
        self.hit = None
        self.controller.set_calibration_angle(angle)
        self.command_at = now
        self.deadline = now + ACK_TIMEOUT
        self.phase = 'CONFIGURING'
        self.event('configure', now)

    def feed(self, messages, now):
        if not self.busy or self.failure:
            return
        for message in messages:
            if message.get('instrument') != 'hihat':
                continue
            if message.get('kind') == 'progress':
                value = message.get('finalized_at')
                if (isinstance(value, (int, float)) and not isinstance(value, bool)
                        and math.isfinite(value) and value <= now + .05
                        and (self.finalized_at is None or value >= self.finalized_at)):
                    self.finalized_at, self.progress_at = value, now
            elif message.get('kind') == 'hit' and self.close_at is not None:
                onset, received = message.get('event_at'), message.get('detected_at')
                if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                           and math.isfinite(v) for v in (onset, received)):
                    continue
                if (self.close_at <= onset <= now + .05 and onset <= received + .05
                        and received <= now + .05
                        and (self.open_at is None or onset < self.open_at)):
                    if self.hit is None:
                        self.hit = dict(message)
                        self.event('hit', now, onset=onset, detected_at=received)
                else:
                    self.event('ignored_hit', now, onset=onset,
                               reason='outside this closure; opening/older hits cannot select an angle')

    def abort(self, reason, now):
        if self.failure is not None:
            return
        self.failure = str(reason)
        self.controller.calibrated = False
        self.event('fault', now, detail=self.failure)
        if self.phase in {'WAITING', 'READY'}:
            self.phase = 'FAULT'
            return
        try:
            if not self.controller.ready():
                raise RuntimeError('Controller fault prevents a supervised return')
            self._open(now, 'FAULT OPENING')
        except Exception:
            # This is ONLY the separate hi-hat output, never the airborne arm.
            self._release_after_fault(now)

    def _release_after_fault(self, now):
        # Even an unplugged serial cable must complete the software fault path
        # so the arm is told to center, rather than waiting forever on cleanup.
        self.phase = 'FAULT'
        try:
            self.controller.release_all()
        except Exception as exc:
            self.event('release_unconfirmed', now, detail=str(exc),
                       recovery='ESP32 heartbeat watchdog; inspect hardware before restart')

    def _fresh_feedback(self, now):
        c = self.controller
        return (c.telemetry is not None and c.telemetry_at is not None
                and 0 <= now - c.telemetry_at <= FEEDBACK_TIMEOUT)

    def _at_target(self, target, now, *, released=False):
        c = self.controller
        return (self._fresh_feedback(now) and c.telemetry_at >= self.command_at
                and c.telemetry['target'] == target
                and ((c.telemetry['released'] == 1 and c.telemetry.get('opened') == 1)
                     if released else abs(c.telemetry['pos'] - target) <= REACHED_COUNTS))

    def tick(self, now, *, audio_ready=True):
        if not self.busy:
            return
        try:
            self._tick(now, audio_ready)
        except Exception as exc:
            if self.failure is None:
                self.abort(str(exc), now)
            else:
                # Failed return or stalled hi-hat: do not drive it indefinitely.
                self._release_after_fault(now)
                self.event('return_failed', now, detail=str(exc))

    def _tick(self, now, audio_ready):
        c = self.controller
        if not c.ready():
            raise RuntimeError('Hi-hat controller fault: ' + c.detail)
        if not self._fresh_feedback(now):
            raise RuntimeError('Hi-hat encoder feedback became stale')
        if not self.failure and not audio_ready:
            raise RuntimeError('Hi-hat detector stopped or became stale during calibration')

        if self.phase == 'CONFIGURING':
            ack = c.angle_ack
            if ack is not None and ack[0] == self.angle and ack[2] >= self.command_at:
                c.calibration_edge(True)
                self.command_at = self.close_at = now
                self.open_at = None
                self.reached_at = None
                self.deadline = now + MOVEMENT_TIMEOUT
                self.phase = 'CLOSING'
                self.event('close', now, target_counts=ack[1])
            elif now >= self.deadline:
                raise RuntimeError('Hi-hat angle acknowledgment timed out; no closure issued')
        elif self.phase == 'CLOSING':
            if self._at_target(c.counts_for(self.angle), now):
                self.reached_at = now if self.reached_at is None else self.reached_at
                if now - self.reached_at >= .1:
                    self.hold_until = now + CLOSE_SECONDS
                    self.phase = 'HOLDING CLOSED'
                    self.event('closed', now, hold_seconds=CLOSE_SECONDS, pos=c.telemetry['pos'])
            else:
                self.reached_at = None
            if self.phase == 'CLOSING' and now >= self.deadline:
                raise RuntimeError('Hi-hat failed to reach its closure target')
        elif self.phase == 'HOLDING CLOSED':
            if abs(c.telemetry['pos'] - c.counts_for(self.angle)) > 8:
                raise RuntimeError('Hi-hat moved away from its commanded closed position')
            if now >= self.hold_until:
                self._open(now)
        elif self.phase in {'INITIAL OPENING', 'OPENING', 'FAULT OPENING'}:
            if self._at_target(0, now, released=True):
                # Firmware releases on crossing its zero deadband. The passive
                # mechanism then coasts: do not demand that an unpowered motor
                # continue holding exactly zero. Require the real return latch
                # AND a stationary released encoder before the 2 s open dwell.
                position = c.telemetry['pos']
                if self.open_settle_position is None or abs(position-self.open_settle_position) > REACHED_COUNTS:
                    self.open_settle_position, self.reached_at = position, now
                if now-self.reached_at < .2:
                    if now >= self.deadline:
                        raise RuntimeError('Hi-hat did not settle after its open return')
                    return
                if self.phase == 'FAULT OPENING':
                    self._release_after_fault(now)
                    self.event('fault_open_verified', now, pos=c.telemetry['pos'])
                else:
                    initial = self.phase == 'INITIAL OPENING'
                    self.hold_until = now + OPEN_SECONDS
                    self.deadline = self.hold_until + AUDIO_BACKLOG_TIMEOUT
                    self.phase = 'INITIAL OPEN' if initial else 'HOLDING OPEN'
                    self.event('opened', now, open_seconds=OPEN_SECONDS, pos=c.telemetry['pos'])
            elif now >= self.deadline:
                raise RuntimeError('Hi-hat failed to return to open zero')
            else:
                self.open_settle_position = self.reached_at = None
        elif self.phase in {'INITIAL OPEN', 'HOLDING OPEN'}:
            if not self._at_target(0, now, released=True):
                raise RuntimeError('Hi-hat lost its verified open/released state while waiting')
            if now < self.hold_until:
                return
            if self.phase == 'INITIAL OPEN':
                self._configure(now, ANGLES[0])
            elif self.hit is not None:
                self.selected_angle = self.angle
                c.target_degrees = self.angle
                c.calibrated = True
                self.phase = 'READY'
                self.event('selected', now, onset=self.hit['event_at'])
            elif (self.finalized_at is not None and self.finalized_at >= self.open_at
                  and self.progress_at is not None and now - self.progress_at <= 2):
                self.event('miss', now, finalized_at=self.finalized_at)
                if self.angle == ANGLES[-1]:
                    self.abort('No hi-hat HIT detected at any angle from 90° through 115°; calibration failed', now)
                else:
                    self._configure(now, self.angle + 5)
            elif now >= self.deadline:
                raise RuntimeError('Hi-hat audio processing did not finalize the closure interval; not treating it as a miss')
