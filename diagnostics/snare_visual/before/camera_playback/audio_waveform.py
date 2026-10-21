"""Optional TONOR debug window in its own process; never imports arm controls.

The independent capture stream uses the same ADC-monotonic clock convention
as ST7. Detector messages are read-only copies, never inputs to calibration.
No audio is saved. Closing/failing this process cannot stop the beat program.
"""
import argparse
import json
import os
import queue
import signal
import subprocess
import time
import tkinter as tk

import numpy as np

from .audio import AudioReceiver
from .audio_bridge import (DEFAULT_TONOR_SOURCE, DEFAULT_AUDIO_DEVICE,
                           capture_sources, choose_tonor_source)
from .waveform import WaveformTimeline, WaveformWindow, capture_start, HISTORY_SECONDS


class MicrophoneCapture:
    def __init__(self, device=DEFAULT_AUDIO_DEVICE):
        self.device = int(device) if str(device).isdecimal() else device
        self.blocks = queue.Queue(maxsize=100)
        self.stream = None
        self.dropped_blocks = 0
        self.error = None
        self.captured_frames = 0

    def callback(self, data, frames, timing, status):
        try:
            if status:
                raise ValueError('Microphone capture gap: ' + str(status))
            started = capture_start(time.monotonic(), timing.currentTime,
                                    timing.inputBufferAdcTime)
            item = (started, np.array(data[:, 0], dtype=np.float32, copy=True))
            # Prefer current audio if drawing fell behind. Absolute ADC times
            # expose the missing interval rather than compressing the timeline.
            if self.blocks.full():
                try:
                    self.blocks.get_nowait()
                    self.dropped_blocks += 1
                except queue.Empty:
                    pass
            self.blocks.put_nowait(item)
            self.captured_frames += frames
        except (ValueError, queue.Full) as exc:
            self.error = str(exc)
            self.dropped_blocks += 1

    def start(self):
        # Delay PortAudio import until PULSE_SOURCE is set for this process.
        import sounddevice as sd
        self.stream = sd.InputStream(device=self.device, samplerate=16000,
                                     channels=1, dtype='float32', blocksize=320,
                                     callback=self.callback)
        self.stream.start()

    def poll(self):
        blocks = []
        for _ in range(100):
            try:
                blocks.append(self.blocks.get_nowait())
            except queue.Empty:
                break
        return blocks

    def close(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None


class WaveformApp:
    def __init__(self, root, ride_socket, hihat_socket, *, device=DEFAULT_AUDIO_DEVICE,
                 history=HISTORY_SECONDS):
        self.root = root
        self.timeline = WaveformTimeline(history)
        self.window = WaveformWindow(root, self.timeline)
        self.receivers = {}
        self.errors = []
        for instrument, path in (('ride', ride_socket), ('hihat', hihat_socket)):
            try:
                self.receivers[instrument] = AudioReceiver(path, instrument=instrument)
            except Exception as exc:
                self.errors.append(f'{instrument} markers unavailable: {exc}')
        self.capture = MicrophoneCapture(device)
        self.stopping = self.closed = False
        self.after_id = None
        root.protocol('WM_DELETE_WINDOW', self.request_close)

    def request_close(self, *_):
        self.stopping = True

    def start(self):
        try:
            self.capture.start()
        except Exception as exc:
            self.errors.append('TONOR capture unavailable: ' + str(exc))
        self.tick()

    def tick(self):
        if self.closed:
            return
        if self.stopping:
            self.root.quit()
            return
        for started, samples in self.capture.poll():
            self.timeline.add_audio(started, samples)
        # Poll both streams before rendering; delayed events are inserted at
        # event_at, never notification time or the graph's right edge.
        for receiver in self.receivers.values():
            for message in receiver.poll():
                self.timeline.add_hit(message, time.monotonic())
        now = time.monotonic()
        details = []
        if self.errors:
            details.append('; '.join(self.errors))
        if self.timeline.latest_sample is None:
            details.append('Waiting for TONOR audio')
        elif now-self.timeline.latest_sample > .5:
            details.append('TONOR audio stale — capture gap')
        else:
            details.append('TONOR live')
        for instrument, receiver in self.receivers.items():
            state = 'ready' if receiver.ready(now) else receiver.state
            if state == 'ready' and not receiver.ready(now):
                state = 'stale'
            details.append(f'{instrument}: {state}')
        if self.capture.dropped_blocks:
            details.append(f'visual capture gaps: {self.capture.dropped_blocks}')
        self.window.render(now, ' · '.join(details))
        self.after_id = self.root.after(50, self.tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
            except tk.TclError:
                pass
        try:
            self.capture.close()
        finally:
            for receiver in self.receivers.values():
                receiver.close()


def configure_capture_source(preferred):
    # No set-default-source command: do not change routing for other programs.
    result = subprocess.run(['pactl', 'list', 'short', 'sources'], check=True,
                            capture_output=True, text=True)
    source = choose_tonor_source(capture_sources(result.stdout), preferred)
    os.environ['PULSE_SOURCE'] = source
    os.environ['PIPEWIRE_PROPS'] = json.dumps(
        {'application.name': 'X5-audio-waveform', 'target.object': source})
    return source


def main():
    p = argparse.ArgumentParser(description='Visual-only rolling TONOR waveform with ST7 onset markers')
    p.add_argument('--ride-socket', required=True)
    p.add_argument('--hihat-socket', required=True)
    p.add_argument('--source', default=DEFAULT_TONOR_SOURCE)
    p.add_argument('--device', default=DEFAULT_AUDIO_DEVICE)
    p.add_argument('--history-seconds', type=float, default=HISTORY_SECONDS)
    args = p.parse_args()
    root = tk.Tk()
    app = WaveformApp(root, args.ride_socket, args.hihat_socket,
                      device=args.device, history=args.history_seconds)
    signal.signal(signal.SIGINT, app.request_close)
    signal.signal(signal.SIGTERM, app.request_close)
    try:
        try:
            source = configure_capture_source(args.source)
            print('WAVEFORM: visual-only TONOR capture on ' + source, flush=True)
            app.start()
        except Exception as exc:
            # Keep an informative window without selecting a different mic.
            app.errors.append(str(exc))
            app.tick()
        root.mainloop()
    finally:
        app.close()
        root.destroy()


if __name__ == '__main__':
    main()
