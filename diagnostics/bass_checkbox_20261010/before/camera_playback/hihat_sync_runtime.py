"""Shared ESP32 beat adapter: calibrated hi-hat and uncalibrated quarter-note bass.

Never commands or retimes the ride arm. Both use its grid, not each other's
serial command times; the hi-hat's learned advance never moves the bass.
"""
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

from .hihat_sync import HiHatSynchronizer, TimingFault
from .bass_drum import BassDrumScheduler
from .tempo import SYNC_PERIOD


class HiHatSyncRuntime:
    def __init__(self, app):
        self.app = app
        self.engine = None
        self.bass_drum = None
        self.after_id = None
        self.stream = None
        self.directory = None
        self.last_observed = None
        self.calibrated_advance = None

    @property
    def active(self):
        return self.engine is not None and self.engine.active

    def log(self, row):
        if self.stream is not None:
            try:
                self.stream.write(json.dumps(row, allow_nan=False)+'\n')
            except OSError:
                # Diagnostics must not disrupt motor supervision.
                self.stream.close()
                self.stream = None
        if row['kind'] in {'start', 'grid', 'match', 'skip', 'locked', 'stop', 'fault'}:
            print('HIHAT SYNC: '+json.dumps(row, allow_nan=False), flush=True)

    def send(self, closed):
        self.app.hihat.send_state(closed)
        return time.monotonic()

    def send_kick(self):
        self.app.hihat.send_kick()
        return time.monotonic()

    def start(self, period):
        if self.calibrated_advance is None:
            raise TimingFault('Ride/hi-hat timing must be calibrated before swing')
        self._start(period, fixed_advance=self.calibrated_advance)

    def start_calibration(self, epoch):
        self.calibrated_advance = None
        self._start(SYNC_PERIOD, calibration=True)
        self.engine.set_epoch(epoch, time.monotonic())

    def finish_calibration(self):
        if self.engine is None or not self.engine.complete:
            raise TimingFault('Ride/hi-hat timing calibration is not complete')
        self.calibrated_advance = self.engine.advance
        self.stop()
        return self.calibrated_advance

    @property
    def calibration_complete(self):
        return self.engine is not None and self.engine.complete

    def _start(self, period, **options):
        self.stop()
        self.directory = Path(__file__).resolve().parents[1]/'playback_results'/'hihat_sync'/(
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
        try:
            self.directory.mkdir(parents=True)
            self.stream = (self.directory/'timeline.jsonl').open('w', buffering=1)
        except OSError as exc:
            print('HIHAT SYNC: timeline file unavailable: '+str(exc), flush=True)
        self.engine = HiHatSynchronizer(self.send, self.log, period, **options)
        if not self.engine.calibration:
            self.bass_drum = BassDrumScheduler(self.send_kick, period, self.log)
        self.engine.start(time.monotonic())
        self.last_observed = time.monotonic()
        self.after_id = self.app.root.after(5, self.tick)

    def observe(self, status, now):
        if not self.active or not status.swing:
            return
        self.last_observed = now
        if self.engine.calibration:
            return  # Shared future deadline was supplied before either command.
        if self.engine.epoch is None:
            if status.count == 1 and status.bottoms == 1 and status.bottom_at is not None:
                # Unchanged worker establishes its grid at the pickup bottom.
                # The following beat 1 is one triplet subdivision later.
                self.engine.set_epoch(status.bottom_at+self.engine.period/3, now)
            elif status.main_at is not None and status.main_count:
                self.engine.set_epoch(status.main_at-(status.main_count-1)*self.engine.period, now)

    def feed(self, instrument, messages):
        if self.active:
            self.engine.feed(instrument, messages, time.monotonic())

    def tick(self):
        self.after_id = None
        if not self.active:
            return
        a, now = self.app, time.monotonic()
        try:
            if not a.continuous_strike_active or a.continuous_stop_requested:
                self.stop()
                return
            if now-self.last_observed > .15:
                raise TimingFault('Ride status stale; cancelling future hi-hat/bass commands')
            monitor = getattr(a, 'hihat_sound_monitor', None)
            receiver = getattr(monitor, 'receiver', None)
            available = dict(ride=a.audio_receiver.ready(now),
                             hihat=receiver is not None and receiver.ready(now))
            if self.engine.calibration and not all(available.values()):
                raise TimingFault('Ride/hi-hat detector unavailable during timing calibration')
            self.engine.tick(now, available)
            if self.bass_drum is not None:
                self.bass_drum.tick(time.monotonic(), self.engine.epoch)
        except Exception as exc:
            self.log(dict(kind='fault', at=now, detail=str(exc)))
            self.stop()
            # Existing stop finishes the hybrid return then verifies center;
            # no disable/relax is performed here.
            a._request_continuous_stop('Automatic ESP32 beat timing fault: '+str(exc))
            return
        if self.active:
            self.after_id = a.root.after(5, self.tick)

    def stop(self):
        # No new K commands after stop. A pulse already sent coasts after its
        # firmware-owned 50 ms; emergency release_all still cuts both outputs.
        self.bass_drum = None
        if self.after_id is not None:
            self.app.root.after_cancel(self.after_id)
            self.after_id = None
        if self.engine is not None:
            self.engine.stop(time.monotonic())
        if self.stream is not None:
            self.stream.close()
            self.stream = None
