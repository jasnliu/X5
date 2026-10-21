#!/usr/bin/env python3
"""Run one pinned ST7 model on TONOR, on an instrument-specific UI socket."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

from .audio import AudioSender
from .audio_visual import MirroredAudioSender


DEFAULT_TONOR_SOURCE = (
    "alsa_input.usb-TONOR_TONOR_TD510_Dynamic_Mic_0000KT5a300000135-00.analog-stereo"
)
DEFAULT_AUDIO_DEVICE = "pipewire"
ST7_MODEL_VERSION = "v2"
HIHAT_MODEL_VERSION = "hihat_v1"
MODEL_VERSIONS = {"ride": ST7_MODEL_VERSION, "hihat": HIHAT_MODEL_VERSION}
HEARTBEAT_SECONDS = 0.5
LISTENER_WARMUP_SECONDS = 2.0


def capture_sources(output: str) -> list[str]:
    """Return non-monitor source names from ``pactl list short sources``."""
    sources = []
    for line in output.splitlines():
        columns = line.split("\t")
        if len(columns) < 2:
            columns = line.split()
        if len(columns) >= 2 and not columns[1].endswith(".monitor"):
            sources.append(columns[1])
    return sources


def choose_tonor_source(sources: list[str], preferred: str) -> str:
    if preferred in sources:
        return preferred
    matches = [source for source in sources if "tonor" in source.lower()]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise RuntimeError("no TONOR capture source is connected")
    raise RuntimeError(
        "multiple TONOR capture sources found; pass --source with one of: "
        + ", ".join(matches)
    )


def route_tonor(preferred: str) -> str:
    result = subprocess.run(
        ["pactl", "list", "short", "sources"],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    source = choose_tonor_source(capture_sources(result.stdout), preferred)
    subprocess.run(
        ["pactl", "set-default-source", source],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return source


def detector_command(root: Path, device: str, instrument: str = "ride") -> list[str]:
    """Pin checkpoint, config and normality to the same ST7 model version."""
    root = root.resolve()
    python = root / ".venv/bin/python"
    program = root / "cymbal.py"
    version = MODEL_VERSIONS[instrument]
    model = root / "models" / version
    config = model / "config.yaml"
    checkpoint = model / "best.pt"
    normality = model / "normality/reference.npz"
    missing = [
        path for path in (python, program, config, checkpoint, normality)
        if not path.is_file()
    ]
    if missing:
        raise RuntimeError(f"ST7 {version} is incomplete; missing " + ", ".join(map(str, missing)))
    return [str(python), str(program), "--config", str(config), "listen",
            "--checkpoint", str(checkpoint), "--normality-artifact", str(normality),
            "--device", device]


def validate_instrument(message, instrument):
    """Ride's legacy runtime is untagged; hi-hat must identify its own target."""
    if instrument == 'hihat':
        valid = message.get('instrument') == 'hihat' and message.get('label') == 'hihat_close'
    else:
        valid = (message.get('instrument', 'ride') == 'ride'
                 and message.get('label', 'cymbal_hit') == 'cymbal_hit')
    if not valid:
        raise RuntimeError(f'ST7 {instrument} listener reported a different instrument/target')


def forward_audio_message(message, sender, now, instrument="ride"):
    """Never reconstruct an ADC timestamp from process startup/pipe receipt."""
    if message.get('event') == 'HIT':
        validate_instrument(message, instrument)
        values = [message.get(k) for k in ('event_monotonic', 'score', 'normality_score')]
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in values):
            raise RuntimeError('ST7 HIT lacks a valid ADC-monotonic onset; update its listener')
        onset, score, normality = values
        if onset > now+.05:
            raise RuntimeError('ST7 audio timestamp is in the future')
        sender.hit(onset, now, score, normality)
        return False
    if message.get('event') == 'AUDIO_PROGRESS':
        values = [message.get(k) for k in ('finalized_monotonic', 'captured_monotonic')]
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in values):
            raise RuntimeError('ST7 progress lacks valid capture timestamps')
        finalized, captured = values
        if not finalized <= captured <= now+.05:
            raise RuntimeError('ST7 progress has inconsistent capture timestamps')
        sender.progress(finalized, captured)
        return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Bridge one ST7 instrument to its isolated X5 socket")
    parser.add_argument("--socket", required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--source", default=DEFAULT_TONOR_SOURCE)
    parser.add_argument("--device", default=DEFAULT_AUDIO_DEVICE)
    parser.add_argument("--instrument", choices=tuple(MODEL_VERSIONS), default="ride")
    parser.add_argument('--visual-socket', help='optional one-way event copy for the waveform window')
    args = parser.parse_args()

    version = MODEL_VERSIONS[args.instrument]
    sender = (MirroredAudioSender(args.socket, instrument=args.instrument, visual_path=args.visual_socket)
              if args.visual_socket else AudioSender(args.socket, instrument=args.instrument))
    child = None
    selector = None
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True
        if child is not None and child.poll() is None:
            child.terminate()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    sender.status("starting", "Selecting the connected TONOR microphone")
    try:
        command = detector_command(args.root.resolve(), args.device, args.instrument)
        print(f"ST7 {args.instrument} audio model {version}: "
              f"{args.root.resolve() / 'models' / version} "
              "(matching config, checkpoint and normality reference)", flush=True)
        source = route_tonor(args.source)
        environment = os.environ.copy()
        environment['X5_AUDIO_TIMESTAMPS'] = '1'
        environment['PULSE_SOURCE'] = source
        environment['PIPEWIRE_PROPS'] = json.dumps({
            'application.name': f'X5-{args.instrument}-detector', 'target.object': source})
        # Independent listeners isolate visual-only failures from ride control.
        # Bound CPU thread pools so their encoders cannot oversubscribe the host.
        for key in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS'):
            environment[key] = '1'
        environment.pop("PYTHONPATH", None)
        environment.pop("LD_LIBRARY_PATH", None)
        child = subprocess.Popen(
            command,
            cwd=str(args.root.resolve()),
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=0,
        )
        selector = selectors.DefaultSelector()
        selector.register(child.stdout, selectors.EVENT_READ)
        os.set_blocking(child.stdout.fileno(), False)
        pending = b''
        progress_at = None
        listener_started_at = None
        ready_at = None
        last_detail = f"ST7 model {version} loading; TONOR source {source}"
        last_line = ""
        next_heartbeat = 0.0

        while child.poll() is None:
            now = time.monotonic()
            for _key, _mask in selector.select(timeout=0.1):
                chunk = os.read(child.stdout.fileno(), 65536)
                pending += chunk
                # Drain all complete lines, including ones buffered in a single
                # read; readline()+select can strand a hit until another write.
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    last_line = line.decode(errors='replace').strip()
                    try:
                        message = json.loads(last_line)
                    except json.JSONDecodeError:
                        print(last_line, flush=True)
                        continue
                    if message.get('event') != 'AUDIO_PROGRESS':
                        print(last_line, flush=True)
                    if message.get("status") == "listening":
                        validate_instrument(message, args.instrument)
                        normality = message.get("normality")
                        if not isinstance(normality, dict) or normality.get("enabled") is not True:
                            raise RuntimeError("ST7 listener did not enable its normality scorer")
                        if message.get('timestamp_clock') != 'portaudio_adc_monotonic':
                            raise RuntimeError('ST7 listener needs ADC-monotonic audio timestamps')
                        listener_started_at = time.monotonic()
                        ready_at = listener_started_at + LISTENER_WARMUP_SECONDS
                        normality_version = normality.get("version", "unknown")
                        last_detail = (f"ST7 model {version}; normality format v{normality_version} "
                                       f"warming up on TONOR source {source}")
                    elif forward_audio_message(message, sender, time.monotonic(), args.instrument):
                        progress_at = time.monotonic()

            now = time.monotonic()
            if (ready_at is not None and now >= ready_at
                    and progress_at is not None and now-progress_at < 2.):
                last_detail = (
                    f"ST7 model {version} and normality scorer ready; "
                    f"listening to TONOR source {source}"
                )
                state = "ready"
            else:
                state = "starting"
            if now >= next_heartbeat:
                sender.status(state, last_detail)
                next_heartbeat = now + HEARTBEAT_SECONDS

        return_code = child.wait()
        if stopping:
            sender.status("stopped", f"ST7 {args.instrument} listener stopped")
            return 0
        detail = f"ST7 {args.instrument} listener exited with status {return_code}"
        if last_line:
            detail += ": " + last_line[:300]
        sender.status("error", detail)
        return return_code or 1
    except Exception as exc:
        detail = "ST7/TONOR startup failed: " + str(exc)
        print(detail, file=sys.stderr, flush=True)
        sender.status("error", detail)
        return 1
    finally:
        if selector is not None:
            selector.close()
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        sender.close()


if __name__ == "__main__":
    raise SystemExit(main())
