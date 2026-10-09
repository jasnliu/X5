"""Hi-hat-only acoustic phase servo; no ride controller or motor imports.

All times are host monotonic seconds. Detector delivery times NEVER stand in
for acoustic onsets. Positive advance sends earlier. The ride grid is read-only.
"""
from collections import deque
from dataclasses import dataclass
import math
import statistics


MAX_ADVANCE = .250
MAX_STEP = .010
DEADBAND = .015
RIDE_GATE = .140
CLOSURE_LATENCY_MAX = .450
MAX_LATENESS = .080
MAX_PAIR_AGE = 6.


class TimingFault(RuntimeError):
    pass


@dataclass
class Closure:
    index: int
    target: float
    commanded: float
    advance: float


class HiHatSynchronizer:
    """Deterministic scheduler + conservative event pairing, tested without I/O.

    send(closed) executes ONE edge, returning its actual monotonic send time.
    A CLOSE and following OPEN share a latched advance, preserving closed time.
    Updates never re-time an already queued edge. Open duration changes by at
    most MAX_STEP between successive pairs. No catch-up commands are sent.
    """
    def __init__(self, send, log=lambda row: None, period=.6):
        if not math.isfinite(period) or period < 2*MAX_ADVANCE:
            raise ValueError('Unsupported hi-hat beat period')
        self.send, self.log, self.period = send, log, period
        self.active = False
        self.advance = 0.
        self.epoch = None
        self.index = 0
        self.pair_advance = 0.
        self.events = {'ride': deque(maxlen=512), 'hihat': deque(maxlen=512)}
        self.watermarks = {'ride': None, 'hihat': None}
        self.progress_at = {'ride': -math.inf, 'hihat': -math.inf}
        self.pending = deque()
        self.estimates = deque(maxlen=5)
        self.last_estimate_at = None
        self.last_pair_advance = 0.
        self.started = 0.
        self.last_commanded = None
        self.matches = self.skips = 0

    def start(self, now):
        self.active = True
        self.started = now
        self.log(dict(kind='start', at=now, advance=0., period=self.period,
                      close_beats=[2, 4], open_beats=[1, 3]))

    def stop(self, now):
        if self.active:
            self.log(dict(kind='stop', at=now, advance=self.advance,
                          matches=self.matches, skips=self.skips))
        self.active = False
        self.pending.clear()

    def set_epoch(self, epoch, now):
        if self.epoch is None and self.active:
            if not math.isfinite(epoch) or epoch < now-MAX_LATENESS:
                raise TimingFault('Hi-hat grid arrived too late; refusing catch-up')
            self.epoch = epoch
            self.log(dict(kind='grid', at=now, beat1=epoch))

    def feed(self, instrument, messages, now):
        if not self.active or instrument not in self.events:
            return
        for m in messages:
            if m.get('instrument', 'ride') != instrument:
                continue
            if m.get('kind') == 'progress':
                value = m.get('finalized_at')
                old = self.watermarks[instrument]
                if (isinstance(value, (int, float)) and math.isfinite(value)
                        and value <= now+.05 and (old is None or value >= old)):
                    self.watermarks[instrument] = value
                    self.progress_at[instrument] = now
                continue
            if m.get('kind') != 'hit':
                continue
            at = m.get('event_at')
            if (not isinstance(at, (int, float)) or not math.isfinite(at)
                    or not self.started <= at <= now+.05 or now-at > MAX_PAIR_AGE):
                continue
            if any(abs(e['event_at']-at) < .003 for e in self.events[instrument]):
                continue
            event = dict(m)
            self.events[instrument].append(event)
            self.log(dict(kind='onset', instrument=instrument, event_at=at,
                          received_at=now, score=m.get('score')))

    def _within(self, instrument, lo, hi):
        return [e for e in self.events[instrument] if lo <= e['event_at'] <= hi]

    def _skip(self, c, reason, now):
        self.skips += 1
        # Do not combine old calibration with a later sparse/new sound regime.
        self.estimates.clear()
        self.log(dict(kind='skip', index=c.index, target=c.target, at=now, reason=reason))

    def evaluate(self, now, available):
        """Consume each closure only after BOTH acoustic windows are finalized."""
        while self.pending:
            c = self.pending[0]
            if now-c.target > MAX_PAIR_AGE:
                self.pending.popleft()
                self._skip(c, 'detector timeout', now)
                continue
            end = max(c.target+self.period*2/3+RIDE_GATE,
                      c.commanded+CLOSURE_LATENCY_MAX)
            if not all(available.get(k, False) and self.watermarks[k] is not None
                       and self.watermarks[k] >= end and now-self.progress_at[k] <= 2.
                       for k in self.events):
                break
            self.pending.popleft()
            rides = self._within('ride', c.target-RIDE_GATE, c.target+RIDE_GATE)
            hats = self._within('hihat', c.commanded, c.commanded+CLOSURE_LATENCY_MAX)
            # Check another ride-only grid position, not just a possible hi-hat
            # cross-response. Do not veto genuine simultaneous detections.
            neighbors = (self._within('ride', c.target-self.period-RIDE_GATE,
                                      c.target-self.period+RIDE_GATE)
                         + self._within('ride', c.target+self.period*2/3-RIDE_GATE,
                                        c.target+self.period*2/3+RIDE_GATE))
            if len(rides) != 1 or len(hats) != 1 or not neighbors:
                self._skip(c, f'ambiguous/missing: ride={len(rides)}, hihat={len(hats)}, '
                           f'neighbor_ride={len(neighbors)}', now)
                continue
            ride, hat = rides[0]['event_at'], hats[0]['event_at']
            error = hat-ride
            actual_advance = c.target-c.commanded
            estimate = actual_advance+error
            if abs(estimate) > MAX_ADVANCE:
                self._skip(c, 'required advance outside signed safety bound', now)
                continue
            if self.last_estimate_at is not None and now-self.last_estimate_at > MAX_PAIR_AGE:
                self.estimates.clear()
            self.last_estimate_at = now
            self.estimates.append(estimate)
            before = self.advance
            if len(self.estimates) >= 3:
                desired = statistics.median(self.estimates)
                residual = desired-self.advance
                if abs(residual) > DEADBAND:
                    self.advance = max(-MAX_ADVANCE, min(MAX_ADVANCE,
                        self.advance+max(-MAX_STEP, min(MAX_STEP, .35*residual))))
            self.matches += 1
            self.log(dict(kind='match', at=now, index=c.index, target=c.target,
                          ride_onset=ride, hihat_onset=hat, error=error,
                          commanded=c.commanded, advance_used=c.advance,
                          actual_advance=actual_advance, estimate=estimate,
                          advance_before=before, advance=self.advance))

    def tick(self, now, available):
        if not self.active:
            return
        self.evaluate(now, available)
        if self.epoch is None:
            return
        target = self.epoch+self.index*self.period
        advance = 0. if self.index == 0 else self.pair_advance
        due = target-advance
        if now < due:
            return
        if now-due > MAX_LATENESS:
            raise TimingFault('Hi-hat deadline missed; refusing catch-up burst')
        closed = self.index % 2 == 1  # OPEN 1/3, CLOSE 2/4, no pickup command.
        actual = self.send(closed)
        if not math.isfinite(actual) or actual < now-.001 or actual-due > MAX_LATENESS:
            raise TimingFault('Hi-hat serial delivery missed deadline')
        self.log(dict(kind='command', at=actual, index=self.index,
                      beat=self.index % 4+1, closed=closed, target=target,
                      due=due, advance=advance, lateness=actual-due))
        if closed:
            self.pending.append(Closure(self.index, target, actual, advance))
        else:
            # Lock the NEXT close and its following open together. Even if
            # several delayed observations arrive together, edge timing moves
            # by no more than 10 ms per pair.
            self.pair_advance = self.last_pair_advance+max(-MAX_STEP,
                min(MAX_STEP, self.advance-self.last_pair_advance))
            self.last_pair_advance = self.pair_advance
        self.index += 1
        self.last_commanded = actual
