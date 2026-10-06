#!/usr/bin/env python3
"""Run ST7 on the TONOR input and forward its HIT events to the arm UI."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

from .audio import AudioSender


DEFAULT_TONOR_SOURCE = (
    "alsa_input.usb-TONOR_TONOR_TD510_Dynamic_Mic_0000KT5a300000135-00.analog-stereo"
)
DEFAULT_AUDIO_DEVICE = "pipewire"
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


def detector_command(root: Path, device: str) -> list[str]:
    python = root / ".venv/bin/python"
    program = root / "cymbal.py"
    config = root / "configs/baseline.yaml"
    checkpoint = root / "runs/cymbal-fmn-tcn/best.pt"
    normality = root / "runs/cymbal-normality/reference.npz"
    missing = [
        path for path in (python, program, config, checkpoint, normality)
        if not path.is_file()
    ]
    if missing:
        raise RuntimeError("ST7 is incomplete; missing " + ", ".join(map(str, missing)))
    return [str(python), str(program), "listen", "--device", device]


def main() -> int:
    parser = argparse.ArgumentParser(description="Bridge ST7 cymbal HIT events to X5")
    parser.add_argument("--socket", required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--source", default=DEFAULT_TONOR_SOURCE)
    parser.add_argument("--device", default=DEFAULT_AUDIO_DEVICE)
    args = parser.parse_args()

    sender = AudioSender(args.socket)
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
        source = route_tonor(args.source)
        command = detector_command(args.root.resolve(), args.device)
        environment = os.environ.copy()
        environment.pop("PYTHONPATH", None)
        environment.pop("LD_LIBRARY_PATH", None)
        child = subprocess.Popen(
            command,
            cwd=str(args.root.resolve()),
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        selector = selectors.DefaultSelector()
        selector.register(child.stdout, selectors.EVENT_READ)
        listener_started_at = None
        ready_at = None
        last_detail = f"ST7 loading; TONOR source {source}"
        last_line = ""
        next_heartbeat = 0.0

        while child.poll() is None:
            now = time.monotonic()
            for _key, _mask in selector.select(timeout=0.1):
                line = child.stdout.readline()
                if not line:
                    continue
                last_line = line.strip()
                print(last_line, flush=True)
                try:
                    message = json.loads(last_line)
                except json.JSONDecodeError:
                    continue
                if message.get("status") == "listening":
                    normality = message.get("normality")
                    if not isinstance(normality, dict) or normality.get("enabled") is not True:
                        raise RuntimeError(
                            "ST7 listener did not enable its normality scorer"
                        )
                    listener_started_at = time.monotonic()
                    ready_at = listener_started_at + LISTENER_WARMUP_SECONDS
                    version = normality.get("version", "unknown")
                    last_detail = (
                        f"ST7 normality v{version} warming up on TONOR source {source}"
                    )
                elif message.get("event") == "HIT" and listener_started_at is not None:
                    event_time = message.get("time_since_start")
                    score = message.get("score")
                    normality_score = message.get("normality_score")
                    if (isinstance(event_time, (int, float))
                            and isinstance(score, (int, float))
                            and isinstance(normality_score, (int, float))):
                        detected_at = time.monotonic()
                        sender.hit(
                            listener_started_at + float(event_time),
                            detected_at,
                            float(score),
                            float(normality_score),
                        )

            now = time.monotonic()
            if ready_at is not None and now >= ready_at:
                last_detail = (
                    f"ST7 normality scorer ready; listening to TONOR source {source}"
                )
                state = "ready"
            else:
                state = "starting"
            if now >= next_heartbeat:
                sender.status(state, last_detail)
                next_heartbeat = now + HEARTBEAT_SECONDS

        return_code = child.wait()
        if stopping:
            sender.status("stopped", "ST7 cymbal listener stopped")
            return 0
        detail = f"ST7 cymbal listener exited with status {return_code}"
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
