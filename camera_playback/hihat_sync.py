"""Hi-hat scheduler and shared acoustic pairing; no ride controller/motor imports.

All times are host monotonic seconds. Detector delivery times NEVER stand in
for acoustic onsets. Positive advance sends earlier. The ride grid is read-only.
"""
from collections import deque
from dataclasses import dataclass
import math
import statistics

from .tempo import SYNC_BPM, SYNC_PERIOD


MAX_ADVANCE = .250
MAX_STEP = .150
MIN_PERIOD = .020
ESTIMATE_AGREEMENT = .020
DEADBAND = .015
LOCK_TOLERANCE = .015
LOCK_MATCHES = 2
RIDE_GATE = .140
CLOSURE_LATENCY_MAX = .450
MAX_LATENESS = .080
MAX_PAIR_AGE = 6.
CALIBRATION_MAX_STEP = .200
CALIBRATION_TIMEOUT = 45.


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
    Updates never re-time an already queued edge. Separate 80 BPM calibration
    uses every quarter note, half-beat reopen, and immediate bounded estimates.
    The main runtime supplies fixed_advance for swing, disabling adaptation.
    No catch-up commands are sent.
    """
    def __init__(self, send, log=lambda row: None, period=None, *, calibration=False,
                 fixed_advance=None, instrument='hihat', nominal_lead=0.):
        period = (SYNC_PERIOD if calibration else .6) if period is None else period
        if instrument not in ('hihat', 'snare') or not math.isfinite(nominal_lead) or nominal_lead < 0:
            raise ValueError('Invalid synchronization target')
        self.instrument, self.nominal_lead = instrument, nominal_lead
        if not math.isfinite(period) or period <= MIN_PERIOD:
            raise ValueError('Unsupported hi-hat beat period')
        self.send, self.log, self.period = send, log, period
        self.calibration = calibration
        if calibration and abs(period-SYNC_PERIOD) > 1e-9:
            raise ValueError(f'Synchronization calibration must run at {SYNC_BPM:g} BPM')
        # Preserve the 100 BPM bounds. At faster tempos, keep the advance
        # within half a beat and onset gates away from neighboring ride notes.
        self.max_advance = min(MAX_ADVANCE, period / 2.)
        # A large confirmed correction must still leave at least 3/4 of a
        # quarter note open before the next close. Closed duration is unchanged.
        self.max_step = min(MAX_STEP, period / 4.)
        if calibration:
            # Close/open are half a beat apart. A 200 ms step retains at
            # least 175 ms scheduled open time at the fixed 80 BPM grid.
            self.max_step = CALIBRATION_MAX_STEP
        self.ride_gate = min(RIDE_GATE, period / 3.)
        self.active = False
        self.advance = 0.
        self.epoch = None
        self.index = 0
        self.pair_advance = 0.
        self.events = {'ride': deque(maxlen=512), instrument: deque(maxlen=512)}
        self.watermarks = {k: None for k in self.events}
        self.progress_at = {k: -math.inf for k in self.events}
        self.pending = deque()
        self.estimates = deque(maxlen=3)
        self.last_estimate_at = None
        self.last_pair_advance = 0.
        self.started = 0.
        self.last_commanded = None
        self.matches = self.skips = 0
        self.locked = False
        self.good_matches = 0
        self.last_good_index = None
        self.last_good_advance = None
        if fixed_advance is not None:
            if (calibration or not math.isfinite(fixed_advance)
                    or abs(fixed_advance) > self.max_advance):
                raise ValueError('Invalid calibrated hi-hat advance for this tempo')
            self.advance = self.pair_advance = self.last_pair_advance = fixed_advance
            self.locked = True

    @property
    def complete(self):
        # Finish the latched OPEN before handing off; never start another
        # closure once the two acoustic observations have locked the offset.
        return self.calibration and self.locked and self.index % 2 == 0

    def start(self, now):
        self.active = True
        self.started = now
        self.log(dict(kind='start', at=now, advance=self.advance, period=self.period,
                      mode='calibration' if self.calibration else 'swing',
                      **(dict(hit_beats='every quarter', full_return=True)
                         if self.instrument == 'snare' else
                         dict(close_beats='every quarter' if self.calibration else [2, 4],
                              open_beats='halfway between' if self.calibration else [1, 3])),
                      correction_policy=('fixed' if self.locked else
                          'each_valid_absolute' if self.calibration else 'confirmed_absolute_until_locked'),
                      max_pair_step=self.max_step, estimate_agreement=ESTIMATE_AGREEMENT,
                      lock_tolerance=LOCK_TOLERANCE, lock_matches=LOCK_MATCHES,
                      lock_sequence=('paired_detections' if self.calibration and self.instrument == 'hihat'
                                     else 'consecutive_closures')))

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

    def _skip(self, c, reason, now, *, preserve_good_matches=False):
        self.skips += 1
        # Discard unconfirmed adaptation estimates without changing the offset.
        self.estimates.clear()
        # Finalized one-sided hi-hat calibration windows provide no evidence
        # for or against synchronization. Other invalid observations still reset.
        if not self.locked and not preserve_good_matches:
            self.good_matches = 0
            self.last_good_index = None
            self.last_good_advance = None
        self.log(dict(kind='skip', index=c.index, target=c.target, at=now, reason=reason,
                      good_matches=self.good_matches, lock_streak_preserved=preserve_good_matches))

    def _check_lock(self, closure, error, now):
        if self.locked:
            return
        # Delayed good hits only prove the offset actually used for those hits,
        # not a newer correction. Both must validate today's applied/target
        # value; this also avoids locking while a large change is still ramping.
        good = (abs(error) <= LOCK_TOLERANCE
                and abs(closure.advance-self.advance) <= 1e-9
                and abs(closure.advance-self.pair_advance) <= 1e-9)
        if not good:
            self.good_matches = 0
            self.last_good_index = None
            self.last_good_advance = None
            return
        # Calibration counts consecutive *paired detections*, not adjacent
        # commanded beats: finalized windows missing either onset are ignored.
        # Keep the legacy swing/snare rules and reject repeated/older closures.
        follows_good = (self.last_good_index is not None
                        and (closure.index > self.last_good_index
                             if self.calibration and self.instrument == 'hihat'
                             else self.last_good_index == closure.index-2))
        self.good_matches = (self.good_matches+1
                             if follows_good
                             and self.last_good_advance is not None
                             and abs(self.last_good_advance-closure.advance) <= 1e-9 else 1)
        self.last_good_index = closure.index
        self.last_good_advance = closure.advance
        if self.good_matches >= LOCK_MATCHES:
            self.locked = True
            self.log(dict(kind='locked', at=now, index=closure.index,
                          advance=self.advance, advance_ms=1000*self.advance,
                          tolerance_ms=1000*LOCK_TOLERANCE,
                          consecutive_matches=self.good_matches))

    def evaluate(self, now, available):
        """Consume each closure only after BOTH acoustic windows are finalized."""
        while self.pending:
            c = self.pending[0]
            if now-c.target > MAX_PAIR_AGE:
                self.pending.popleft()
                self._skip(c, 'detector timeout', now)
                continue
            end = max(c.target+self.period*2/3+self.ride_gate,
                      c.commanded+CLOSURE_LATENCY_MAX)
            if self.calibration:
                end = max(c.target+self.ride_gate, c.commanded+CLOSURE_LATENCY_MAX)
            if not all(available.get(k, False) and self.watermarks[k] is not None
                       and self.watermarks[k] >= end and now-self.progress_at[k] <= 2.
                       for k in self.events):
                break
            self.pending.popleft()
            rides = self._within('ride', c.target-self.ride_gate, c.target+self.ride_gate)
            hats = self._within(self.instrument, c.commanded, c.commanded+CLOSURE_LATENCY_MAX)
            # Check another ride-only grid position, not just a possible hi-hat
            # cross-response. Do not veto genuine simultaneous detections.
            neighbors = (self._within('ride', c.target-self.period-self.ride_gate,
                                      c.target-self.period+self.ride_gate)
                         + self._within('ride', c.target+self.period*2/3-self.ride_gate,
                                        c.target+self.period*2/3+self.ride_gate))
            if len(rides) != 1 or len(hats) != 1 or (not self.calibration and not neighbors):
                self._skip(c, f'ambiguous/missing: ride={len(rides)}, {self.instrument}={len(hats)}, '
                           f'neighbor_ride={len(neighbors)}', now,
                           preserve_good_matches=(self.calibration and self.instrument == 'hihat'
                                                  and (not rides or not hats)))
                continue
            ride, hat = rides[0]['event_at'], hats[0]['event_at']
            error = hat-ride
            actual_advance = c.target-c.commanded-self.nominal_lead
            estimate = actual_advance+error
            if abs(estimate) > self.max_advance:
                self._skip(c, 'required advance outside signed safety bound', now)
                continue
            if self.last_estimate_at is not None and now-self.last_estimate_at > MAX_PAIR_AGE:
                self.estimates.clear()
                # Expire adaptation estimates, not the good-pair count: sparse
                # calibration detections may be far apart. Legacy swing lock
                # continuity still requires consecutive closure IDs above.
            self.last_estimate_at = now
            self.estimates.append(estimate)
            before = self.advance
            self._check_lock(c, error, now)
            confirmed = (self.calibration or (len(self.estimates) >= 2 and
                         abs(self.estimates[-1]-self.estimates[-2]) <= ESTIMATE_AGREEMENT))
            desired = None
            if confirmed and not self.locked:
                desired = estimate if self.calibration else statistics.median(self.estimates)
                residual = desired-self.advance
                if abs(residual) > DEADBAND:
                    # This is an ABSOLUTE latency estimate, not accumulated
                    # error. Apply it in full; delayed feedback cannot add the
                    # same correction repeatedly. Rate-limit only at the
                    # scheduling boundary below, not once per delivered event.
                    self.advance = max(-self.max_advance, min(self.max_advance, desired))
            self.matches += 1
            self.log(dict(kind='match', at=now, index=c.index, target=c.target,
                          ride_onset=ride, **{self.instrument+'_onset': hat}, error=error,
                          commanded=c.commanded, advance_used=c.advance,
                          actual_advance=actual_advance, estimate=estimate,
                          advance_before=before, advance=self.advance,
                          estimate_confirmed=confirmed, desired_advance=desired,
                          locked=self.locked, good_matches=self.good_matches))

    def tick(self, now, available):
        if not self.active:
            return
        self.evaluate(now, available)
        if self.calibration:
            if now-self.started > CALIBRATION_TIMEOUT and not self.complete:
                raise TimingFault('Ride/hi-hat synchronization did not converge within 45 seconds')
            self._tick_calibration(now)
            return
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
            # several delayed observations arrive together, apply at most one
            # bounded change per pair, leaving a useful open interval. Never
            # re-time the close/open pair already in flight.
            self.pair_advance = self.last_pair_advance+max(-self.max_step,
                min(self.max_step, self.advance-self.last_pair_advance))
            self.last_pair_advance = self.pair_advance
        self.index += 1
        self.last_commanded = actual

    def _tick_calibration(self, now):
        if self.epoch is None or self.complete:
            return
        closed = self.index % 2 == 0
        target = self.epoch+(self.index//2)*self.period+(0. if closed else self.period/2)
        advance = self.pair_advance
        due = target-advance
        if now < due:
            return
        if now-due > MAX_LATENESS:
            raise TimingFault('Calibration hi-hat deadline missed; refusing catch-up burst')
        actual = self.send(closed)
        if not math.isfinite(actual) or actual < now-.001 or actual-due > MAX_LATENESS:
            raise TimingFault('Calibration hi-hat serial delivery missed deadline')
        self.log(dict(kind='command', at=actual, index=self.index,
                      beat=self.index//2+1, closed=closed, target=target,
                      due=due, advance=advance, lateness=actual-due))
        if closed:
            self.pending.append(Closure(self.index, target, actual, advance))
        elif not self.locked:
            self.pair_advance += max(-self.max_step, min(self.max_step, self.advance-self.pair_advance))
            self.last_pair_advance = self.pair_advance
        self.index += 1
        self.last_commanded = actual
