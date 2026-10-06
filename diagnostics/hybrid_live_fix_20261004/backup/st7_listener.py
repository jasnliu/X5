from __future__ import annotations

import json
import queue
import signal
import time
from dataclasses import dataclass

import numpy as np
import sounddevice as sd
import torch

from .detector import CymbalDetector
from .normality import NormalityScorer


@dataclass
class AudioBlock:
    samples: np.ndarray
    overflowed: bool


def list_devices() -> str:
    return str(sd.query_devices())


def listen(
    config: dict,
    detector: CymbalDetector,
    normality: NormalityScorer,
    device: str | int | None = None,
) -> None:
    runtime = config["runtime"]
    sample_rate = detector.sample_rate
    block_samples = round(float(runtime["block_seconds"]) * sample_rate)
    update_samples = round(float(runtime["update_seconds"]) * sample_rate)
    window_samples = round(float(runtime["window_seconds"]) * sample_rate)
    lookahead = float(runtime["lookahead_seconds"])
    queue_blocks = max(2, round(float(runtime["queue_seconds"]) * sample_rate / block_samples))
    audio_queue: queue.Queue[AudioBlock] = queue.Queue(maxsize=queue_blocks)
    stop = False
    capture_start = time.monotonic()
    samples_seen = 0
    samples_since_update = 0
    ring = np.zeros(0, dtype=np.float32)
    last_emitted = -float("inf")
    last_finalized = 0.0

    def handle_signal(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    def callback(indata, frames, _time_info, status) -> None:
        nonlocal stop
        if status.input_overflow:
            stop = True
        block = AudioBlock(np.array(indata[:, 0], dtype=np.float32, copy=True), bool(status.input_overflow))
        try:
            audio_queue.put_nowait(block)
        except queue.Full:
            stop = True

    old_int = signal.signal(signal.SIGINT, handle_signal)
    old_term = signal.signal(signal.SIGTERM, handle_signal)
    selected = device
    if isinstance(device, str) and device.isdecimal():
        selected = int(device)
    print(
        json.dumps(
            {
                "status": "listening",
                "sample_rate": sample_rate,
                "device": device if device is not None else "default",
                "window_seconds": runtime["window_seconds"],
                "lookahead_seconds": lookahead,
                "decoder": detector.decoder.as_dict(),
                "normality": {
                    "enabled": True,
                    "version": normality.metadata["normality_version"],
                    "meaning": "0-100 similarity to reference normal hits",
                },
            }
        ),
        flush=True,
    )
    try:
        with sd.InputStream(
            samplerate=sample_rate,
            channels=1,
            dtype="float32",
            blocksize=block_samples,
            device=selected,
            callback=callback,
        ):
            while not stop:
                try:
                    block = audio_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                if block.overflowed:
                    raise RuntimeError("Microphone input overflow; refusing to process discontinuous audio")
                ring = np.concatenate((ring, block.samples))[-window_samples:]
                samples_seen += len(block.samples)
                samples_since_update += len(block.samples)
                if samples_since_update < update_samples or len(ring) < min(window_samples, sample_rate * 2):
                    continue
                samples_since_update = 0
                global_end = samples_seen / sample_rate
                global_start = global_end - len(ring) / sample_rate
                finalized = global_end - lookahead
                waveform = torch.from_numpy(ring.copy())
                events = detector.detect_waveform(waveform)
                for event in events:
                    event_time = global_start + event["time"]
                    if event_time <= last_finalized or event_time > finalized:
                        continue
                    if event_time - last_emitted < detector.decoder.min_gap_seconds:
                        continue
                    last_emitted = event_time
                    normality_result = normality.score_waveform(waveform, event["time"])
                    print(
                        json.dumps(
                            {
                                "event": "HIT",
                                "time_since_start": round(event_time, 6),
                                "score": round(event["score"], 6),
                                "normality_score": round(normality_result.score, 1),
                                "notification_monotonic": time.monotonic(),
                            }
                        ),
                        flush=True,
                    )
                last_finalized = finalized
    finally:
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        print(json.dumps({"status": "stopped", "captured_seconds": samples_seen / sample_rate}), flush=True)
