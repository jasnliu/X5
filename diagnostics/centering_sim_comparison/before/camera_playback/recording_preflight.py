"""Background, calculation-only recording validation; no GUI or device access.

The model/zone are read-only inputs. Progress and completion cross a queue;
only the GUI thread may install the result or update widgets. Cancelling a job
discards its result, never starts a recording, and never controls any motor.
"""
import queue
import threading

from .recording_cache import load_cached_smooth_recording as load_smooth_recording


class RecordingPreflight:
    def __init__(self, *args, playback_speed, loader=load_smooth_recording):
        self.messages = queue.SimpleQueue()
        self.cancelled = threading.Event()
        self.thread = threading.Thread(
            target=self._work, args=(loader, args, playback_speed),
            name='recording-preflight', daemon=True)
        self.thread.start()

    def _send(self, kind, value):
        if not self.cancelled.is_set():
            self.messages.put((kind, value))

    def _work(self, loader, args, playback_speed):
        try:
            result = loader(*args, playback_speed=playback_speed,
                            progress=lambda text: self._send('progress', text))
        except Exception as exc:
            self._send('error', str(exc))
        else:
            self._send('result', result)

    def poll(self):
        rows = []
        while not self.cancelled.is_set():
            try:
                rows.append(self.messages.get_nowait())
            except queue.Empty:
                break
        return rows

    def cancel(self):
        self.cancelled.set()
