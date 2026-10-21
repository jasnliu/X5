"""Control-only snare socket and timing adapter; graph delivery stays optional."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

from .audio import AudioReceiver
from .hihat_sync import TimingFault
from .snare_sync import SnareSynchronizer, SnareCalibrationTimeout
from . import snare_workflow as snare


class SnareSyncRuntime:
    def __init__(self, app, socket_path):
        self.app = app
        self.receiver = AudioReceiver(socket_path, 'snare') if socket_path else None
        self.engine = self.after_id = self.stream = None
        self.calibrated_advance = None
        self.last_observed = None
        self.outcome = None
        self.ended_at = None

    @property
    def active(self):
        return self.engine is not None and self.engine.active

    @property
    def calibration_complete(self):
        return self.engine is not None and self.engine.complete

    def log(self, row):
        if self.stream is not None:
            try:
                self.stream.write(json.dumps(row, allow_nan=False)+'\n')
            except OSError:
                self.stream.close()
                self.stream = None
        if row['kind'] in {'start', 'grid', 'match', 'skip', 'locked', 'stop', 'fault', 'timeout'}:
            print('SNARE SYNC: '+json.dumps(row, allow_nan=False), flush=True)

    def start_calibration(self, epoch):
        self.stop()
        self.calibrated_advance = None
        self.outcome = self.ended_at = None
        worker = snare.session(self.app)
        if worker is None or self.receiver is None:
            raise TimingFault('Snare worker and sound detector are required for timing calibration')
        self.receiver.poll()
        if not self.receiver.ready(time.monotonic()):
            raise TimingFault('Snare detector is not ready for timing calibration')
        directory = Path(__file__).resolve().parents[1]/'playback_results'/'snare_sync'/(
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
        try:
            directory.mkdir(parents=True)
            self.stream = (directory/'timeline.jsonl').open('w', buffering=1)
        except OSError as exc:
            print('SNARE SYNC: timeline file unavailable: '+str(exc), flush=True)
        self.engine = SnareSynchronizer(worker.template.curve.down, self.log)
        now = time.monotonic()
        self.engine.start(now)
        self.engine.set_epoch(epoch, now)
        self.last_observed = now
        worker.start_calibration(epoch)
        self.app.bus.left_monitor = None
        self.app.bus.left_playback_active = True
        self.after_id = self.app.root.after(5, self.tick)

    def observe(self, status, now):
        if self.active:
            self.last_observed = now

    def feed(self, instrument, messages):
        if self.active:
            self.engine.feed(instrument, messages, time.monotonic())

    def poll(self):
        # Drain during centering/playback/hi-hat calibration too. Otherwise a
        # full Unix datagram queue can retain startup messages and drop every
        # later ready heartbeat before the snare stage is reached.
        if self.receiver is not None:
            self.feed('snare', self.receiver.poll())

    def tick(self):
        self.after_id = None
        if not self.active:
            return
        a, now = self.app, time.monotonic()
        try:
            if not a.continuous_strike_active or a.continuous_stop_requested:
                self.stop()
                return
            self.poll()
            # poll() timestamps newly received heartbeats. Compare them with a
            # clock read AFTER receipt, not the earlier callback-entry time.
            # Keep the strict nonnegative age and existing freshness limits.
            now = time.monotonic()
            worker = snare.session(a)
            state = worker.status
            if state.error or now-state.at > .15:
                raise TimingFault('Snare worker fault/stale during timing calibration')
            if now-self.last_observed > .15:
                raise TimingFault('Ride status stale during snare calibration')
            available = dict(ride=a.audio_receiver.ready(now), snare=self.receiver.ready(now))
            if not all(available.values()):
                receivers = dict(ride=a.audio_receiver, snare=self.receiver)
                detail = '; '.join(f'{name}: state={receivers[name].state}, '
                                   f'detail={receivers[name].detail}'
                                   for name, ready in available.items() if not ready)
                raise TimingFault('Ride/snare detector unavailable during timing calibration: '+detail)
            self.engine.observe(state)
            self.engine.tick(now, available)
            worker.set_calibration_advance(self.engine.advance)
        except SnareCalibrationTimeout as exc:
            self.outcome, self.ended_at = 'timeout', now
            self.log(dict(kind='timeout', at=now, elapsed=now-self.engine.started,
                          detail=str(exc), calibrated=False))
            self.stop()
            a._request_continuous_stop('Automatic snare calibration timeout: '+str(exc))
            return
        except Exception as exc:
            self.outcome, self.ended_at = 'fault', now
            self.log(dict(kind='fault', at=now, detail=str(exc)))
            self.stop()
            a._request_continuous_stop('Automatic snare calibration fault: '+str(exc))
            return
        if self.active:
            self.after_id = a.root.after(5, self.tick)

    def finish_calibration(self):
        if not self.calibration_complete:
            raise TimingFault('Ride/snare timing calibration is not complete')
        self.calibrated_advance = self.engine.advance
        self.outcome, self.ended_at = 'locked', time.monotonic()
        self.stop()
        return self.calibrated_advance

    def stop(self):
        if self.after_id is not None:
            self.app.root.after_cancel(self.after_id)
            self.after_id = None
        if self.active:
            snare.finish(self.app)
            self.engine.stop(time.monotonic())
        if self.stream is not None:
            self.stream.close()
            self.stream = None

    def close(self):
        self.stop()
        if self.receiver is not None:
            self.receiver.close()
