"""Right can0 only. A final byte allow-list sits immediately before socket.send."""
from collections import deque
import json
import threading
import time

import numpy as np

from safe_zone.encoder import SingleArmObserver, request_frame
from .core import JOINTS

ALLOWED = frozenset(request_frame(m) for m in range(1, 9))


class QuerySocket:
    def __init__(self, sock, log):
        self.sock, self.log = sock, log

    def send(self, data):
        if data not in ALLOWED:
            raise RuntimeError('Blocked non-query CAN packet')
        self.log.write(json.dumps(dict(monotonic=time.monotonic(), interface='can0',
                                       packet_hex=data.hex(), operation='state_request_only')) + '\n')
        self.log.flush()
        return self.sock.send(data)

    def __getattr__(self, name):
        return getattr(self.sock, name)


class Observer:
    def __init__(self, folder):
        self.folder = folder; self.error = None
        self.history = deque(maxlen=250); self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True); self.thread.start()

    def _run(self):
        observer = None
        try:
            with (self.folder / 'can_query_audit.jsonl').open('a') as log:
                observer = SingleArmObserver(side='right', bus='can0')
                observer.socket = QuerySocket(observer.socket, log)
                while not self.stop.is_set():
                    start = time.monotonic(); q = observer.sample(); end = time.monotonic()
                    sample = dict(start=start, end=end, joints={k: q[k] for k in JOINTS})
                    with self.lock:
                        self.history.append(sample)
                    self.stop.wait(max(0, .05 - (end-start)))
        except Exception as e:
            self.error = str(e)
        finally:
            if observer:
                observer.close()

    def samples(self):
        with self.lock:
            return list(self.history)

    def close(self):
        self.stop.set(); self.thread.join(timeout=2)


def stationary_capture(observations, joints, start, end):
    """Reject moving/stale captures. Returns a central frame and nearby measured q.

    This is deliberately stationary capture, NOT hardware timestamp synchronization.
    Frames must be bracketed by multiple fresh, quiet encoder batches.
    """
    frames = [f for f in observations if start <= f['header']['received_monotonic'] <= end]
    qs = [q for q in joints if start-.2 <= q['start'] and q['end'] <= end+.2]
    if len(frames) < 5 or len(qs) < 10:
        raise ValueError('Not enough fresh board/encoder samples; keep the board visible and still')
    ft = np.array([f['header']['received_monotonic'] for f in frames])
    qt = np.array([(q['start']+q['end'])/2 for q in qs])
    if ft[-1]-ft[0] < 1 or qt[-1]-qt[0] < 1 or max(np.diff(ft)) > .35 or max(np.diff(qt)) > .2:
        raise ValueError('Capture has frame/encoder gaps; try again without other camera applications')
    if qt[0] > ft[0] or qt[-1] < ft[-1]:
        raise ValueError('Encoder measurements do not bracket camera frames; hold still and retry')
    values = np.array([[q['joints'][k] for k in JOINTS] for q in qs])
    spread = np.degrees(np.ptp(values, axis=0))
    if np.max(spread) > .35:
        raise ValueError(f'Arm moved {np.max(spread):.2f} degrees during capture; support it more steadily')
    common = set(frames[0]['detection']['corner_ids'])
    for f in frames[1:]:
        common &= set(f['detection']['corner_ids'])
    if len(common) < 8:
        raise ValueError('Board visibility changed during capture')
    common = sorted(common)
    tracks = []
    for f in frames:
        lookup = dict(zip(f['detection']['corner_ids'], f['detection']['corners_px']))
        tracks.append([lookup[i] for i in common])
    tracks = np.asarray(tracks)
    movement = float(np.max(np.linalg.norm(tracks - np.median(tracks, axis=0), axis=2)))
    if movement > 2.5:
        raise ValueError(f'Board/image moved {movement:.1f}px; wait for table/board vibration to settle')
    frame = frames[len(frames)//2]
    nearest = int(np.argmin(np.abs(qt - frame['header']['received_monotonic'])))
    skew = float(abs(qt[nearest] - frame['header']['received_monotonic']))
    if skew > .1:
        raise ValueError('Camera/encoder observations too far apart')
    evidence = dict(duration_s=float(ft[-1]-ft[0]), joint_spread_deg=spread.tolist(),
                    board_motion_px=movement, nearest_query_skew_s=skew,
                    frame_headers=[f['header'] for f in frames], encoder_batches=qs,
                    method='stationary window; host receipt timestamps; no hardware synchronization')
    return frame, qs[nearest]['joints'], evidence
