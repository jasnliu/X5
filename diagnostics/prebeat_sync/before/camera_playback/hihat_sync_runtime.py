"""Nonvisual Tk background adapter. Never commands or retimes the ride arm."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

from .hihat_sync import HiHatSynchronizer, TimingFault


class HiHatSyncRuntime:
    def __init__(self, app):
        self.app = app
        self.engine = None
        self.after_id = None
        self.stream = None
        self.directory = None
        self.last_observed = None

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

    def start(self, period):
        self.stop()
        self.directory = Path(__file__).resolve().parents[1]/'playback_results'/'hihat_sync'/(
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
        try:
            self.directory.mkdir(parents=True)
            self.stream = (self.directory/'timeline.jsonl').open('w', buffering=1)
        except OSError as exc:
            print('HIHAT SYNC: timeline file unavailable: '+str(exc), flush=True)
        self.engine = HiHatSynchronizer(self.send, self.log, period)
        self.engine.start(time.monotonic())
        self.last_observed = time.monotonic()
        self.after_id = self.app.root.after(5, self.tick)

    def observe(self, status, now):
        if not self.active or not status.swing:
            return
        self.last_observed = now
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
                raise TimingFault('Ride status stale; cancelling future hi-hat edges')
            monitor = getattr(a, 'hihat_sound_monitor', None)
            receiver = getattr(monitor, 'receiver', None)
            available = dict(ride=a.audio_receiver.ready(now),
                             hihat=receiver is not None and receiver.ready(now))
            self.engine.tick(now, available)
        except Exception as exc:
            self.log(dict(kind='fault', at=now, detail=str(exc)))
            self.stop()
            # Existing stop finishes the hybrid return then verifies center;
            # no disable/relax is performed here.
            a._request_continuous_stop()
            return
        if self.active:
            self.after_id = a.root.after(5, self.tick)

    def stop(self):
        if self.after_id is not None:
            self.app.root.after_cancel(self.after_id)
            self.after_id = None
        if self.engine is not None:
            self.engine.stop(time.monotonic())
        if self.stream is not None:
            self.stream.close()
            self.stream = None
