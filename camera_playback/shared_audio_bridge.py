"""One ST7 encoder/capture process, unchanged isolated instrument sockets."""
import argparse
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

from .audio_bridge import (DEFAULT_AUDIO_DEVICE, DEFAULT_TONOR_SOURCE,
                           HEARTBEAT_SECONDS, LISTENER_WARMUP_SECONDS, MODEL_VERSIONS,
                           detector_command, forward_audio_message, route_tonor,
                           validate_instrument)
from .audio_visual import MirroredAudioSender


def shared_command(root, device, instruments):
    root = Path(root).resolve()
    command = [str(root/'.venv/bin/python'), '-m', 'cymbal_detector.shared_listener',
               '--device', device]
    if not (root/'cymbal_detector/shared_listener.py').is_file():
        raise RuntimeError('ST7 shared listener is missing; update the sibling ST7 checkout')
    for instrument in instruments:
        single = detector_command(root, device, instrument)
        command.extend(['--model', instrument, single[single.index('--config')+1],
                        single[single.index('--checkpoint')+1],
                        single[single.index('--normality-artifact')+1]])
    return command


class Router:
    def __init__(self, senders):
        self.senders = senders
        self.ready_at = {}
        self.progress_at = {}

    def status(self, state, detail):
        for sender in self.senders.values():
            sender.status(state, detail)

    def consume(self, message, now):
        if message.get('status') == 'stopped':
            return  # Child exit is supervised, never reported as ready.
        instrument = message.get('instrument')
        if instrument not in self.senders:
            raise RuntimeError('Shared audio message has an unknown instrument')
        validate_instrument(message, instrument)
        if message.get('status') == 'listening':
            if (message.get('timestamp_clock') != 'portaudio_adc_monotonic' or
                    message.get('normality', {}).get('enabled') is not True):
                raise RuntimeError('Shared listener requires ADC timestamps and normality')
            self.ready_at[instrument] = now + LISTENER_WARMUP_SECONDS
        elif forward_audio_message(message, self.senders[instrument], now, instrument):
            if instrument not in self.ready_at:
                raise RuntimeError('Audio progress arrived before listener readiness')
            self.progress_at[instrument] = now

    def heartbeat(self, now):
        for instrument, sender in self.senders.items():
            ready = (now >= self.ready_at.get(instrument, float('inf')) and
                     now-self.progress_at.get(instrument, -float('inf')) < 2.)
            sender.status('ready' if ready else 'starting',
                          f'ST7 {MODEL_VERSIONS[instrument]}; shared encoder; '
                          + ('TONOR ready' if ready else 'loading/warming up'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--channel', action='append', nargs=3, required=True,
                        metavar=('INSTRUMENT', 'CONTROL_SOCKET', 'VISUAL_SOCKET'))
    parser.add_argument('--device', default=DEFAULT_AUDIO_DEVICE)
    parser.add_argument('--source', default=DEFAULT_TONOR_SOURCE)
    args = parser.parse_args()
    names = [c[0] for c in args.channel]
    if len(set(names)) != len(names) or any(n not in MODEL_VERSIONS for n in names):
        parser.error('Each channel must name a unique known instrument')
    senders = {}
    child = selector = None
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True
        if child is not None and child.poll() is None:
            child.terminate()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        for name, control, visual in args.channel:
            senders[name] = MirroredAudioSender(control, name, visual or None)
        router = Router(senders)
        router.status('starting', 'Starting shared ST7 audio encoder')
        command = shared_command(args.root, args.device, names)
        source = route_tonor(args.source, change_default=False)
        env = os.environ.copy()
        env.update(X5_AUDIO_TIMESTAMPS='1', PULSE_SOURCE=source,
                   PIPEWIRE_PROPS=json.dumps({'application.name': 'X5-shared-audio',
                                               'target.object': source}))
        for key in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS'):
            env[key] = '1'
        for key in ('PYTHONPATH', 'LD_LIBRARY_PATH'):
            env.pop(key, None)
        if stopping:
            return 0
        print('ST7 shared audio: '+', '.join(MODEL_VERSIONS[n] for n in names), flush=True)
        child = subprocess.Popen(command, cwd=str(args.root.resolve()), env=env,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)
        selector = selectors.DefaultSelector()
        selector.register(child.stdout, selectors.EVENT_READ)
        os.set_blocking(child.stdout.fileno(), False)
        pending = b''
        next_heartbeat = 0.
        while child.poll() is None:
            for key, _ in selector.select(timeout=.1):
                chunk = os.read(key.fileobj.fileno(), 65536)
                if not chunk:
                    selector.unregister(key.fileobj)
                    continue
                pending += chunk
                if len(pending) > 1024*1024:
                    raise RuntimeError('Shared audio output exceeded the bounded line buffer')
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    text = line.decode(errors='replace').strip()
                    try:
                        message = json.loads(text)
                    except json.JSONDecodeError:
                        print(text, flush=True)
                        continue
                    router.consume(message, time.monotonic())
                    if message.get('event') != 'AUDIO_PROGRESS':
                        print(text, flush=True)
            now = time.monotonic()
            if now >= next_heartbeat:
                router.heartbeat(now)
                next_heartbeat = now + HEARTBEAT_SECONDS
        if stopping:
            router.status('stopped', 'Shared audio listener stopped')
            return 0
        raise RuntimeError(f'Shared ST7 listener exited with status {child.returncode}')
    except Exception as exc:
        detail = 'Shared ST7 audio failed: ' + str(exc)
        print(detail, file=sys.stderr, flush=True)
        for sender in senders.values():
            sender.status('error', detail)
        return 1
    finally:
        if selector is not None:
            selector.close()
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        for sender in senders.values():
            sender.close()


if __name__ == '__main__':
    raise SystemExit(main())
