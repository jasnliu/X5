"""Latest-only selected-arm feedback; reuse the recorder's query-only observer."""
from dataclasses import dataclass
import threading
import time

from motion_recording.recording import NOMINAL_SAMPLE_RATE_HZ, utc_now
from safe_zone.encoder import SingleArmObserver

FRESH_SECONDS = .3


@dataclass(frozen=True)
class Snapshot:
    state: dict
    completed: float
    captured_at_utc: str
    batch_duration_s: float


class Feedback:
    def __init__(self, side, bus, observer_factory=None):
        self.side, self.bus = side, bus
        self.observer_factory = observer_factory or SingleArmObserver
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.latest = None
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def read(self):
        with self.lock:
            return self.latest, self.error

    def run(self):
        while not self.stop.is_set():
            observer = None
            try:
                observer = self.observer_factory(self.side, self.bus)
                while not self.stop.is_set():
                    began = time.monotonic()
                    state = observer.sample()
                    ended = time.monotonic()
                    snapshot = Snapshot(state.copy(), ended, utc_now(), ended - began)
                    with self.lock:
                        self.latest, self.error = snapshot, None
                    self.stop.wait(max(0., 1. / NOMINAL_SAMPLE_RATE_HZ - (ended - began)))
            except Exception as exc:
                with self.lock:
                    self.latest, self.error = None, str(exc)
                print(f'Destination feedback unavailable (will retry): {exc}', flush=True)
            finally:
                if observer is not None:
                    observer.close()  # socket only; never sends disable/relax
            self.stop.wait(1.)

    def close(self):
        self.stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.)


def fresh(snapshot, now=None):
    now = time.monotonic() if now is None else now
    return snapshot is not None and 0 <= now - snapshot.completed < FRESH_SECONDS
