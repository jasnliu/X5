from __future__ import annotations

import json
import os
import queue
import signal
import time
from dataclasses import dataclass

import numpy as np
import torch

from .detector import CymbalDetector
from .normality import NormalityScorer
from .audio_clock import block_start_monotonic, event_monotonic
from .audio_input import microphone_input


@dataclass
class AudioBlock:
    samples: np.ndarray
    overflowed: bool
    started_at: float


def list_devices() -> str:
    import sounddevice as sd
    return str(sd.query_devices())


def listen(
    config: dict,
    detector: CymbalDetector,
    normality: NormalityScorer,
    device: str | int | None = None,
) -> None:
    with microphone_input(device) as (selected, source):
        _listen(config, detector, normality, selected,
                requested_device=device, capture_source=source)


def _listen(config, detector, normality, device, *, requested_device, capture_source):
    # PipeWire routing must be set before PortAudio is initialized.
    import sounddevice as sd

    runtime = config["runtime"]
    # X5 opts in; standalone ST7 retains its original console contract.
    clock_reporting = os.environ.get('X5_AUDIO_TIMESTAMPS') == '1'
    sample_rate = detector.sample_rate
    block_samples = round(float(runtime["block_seconds"]) * sample_rate)
    update_samples = round(float(runtime["update_seconds"]) * sample_rate)
    window_samples = round(float(runtime["window_seconds"]) * sample_rate)
    lookahead = float(runtime["lookahead_seconds"])
    queue_blocks = max(2, round(float(runtime["queue_seconds"]) * sample_rate / block_samples))
    audio_queue: queue.Queue[AudioBlock] = queue.Queue(maxsize=queue_blocks)
    stop = False
    clock_error = None
    samples_seen = 0
    samples_since_update = 0
    ring = np.zeros(0, dtype=np.float32)
    last_emitted = -float("inf")
    last_finalized = 0.0

    def handle_signal(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    def callback(indata, frames, _time_info, status) -> None:
        nonlocal stop, clock_error
        try:
            started_at = (block_start_monotonic(time.monotonic(),
                                              _time_info.currentTime,
                                              _time_info.inputBufferAdcTime)
                          if clock_reporting else 0.)
        except ValueError as exc:
            clock_error = str(exc)
            stop = True
            return
        if status.input_overflow:
            stop = True
        block = AudioBlock(np.array(indata[:, 0], dtype=np.float32, copy=True),
                           bool(status.input_overflow), started_at)
        try:
            audio_queue.put_nowait(block)
        except queue.Full:
            stop = True

    old_int = signal.signal(signal.SIGINT, handle_signal)
    old_term = signal.signal(signal.SIGTERM, handle_signal)
    selected = device
    if isinstance(device, str) and device.isdecimal():
        selected = int(device)
    listening_status = {
        "status": "listening",
        **detector.event_metadata,
        "sample_rate": sample_rate,
        "device": device if device is not None else "default",
        **({"requested_device": requested_device, "capture_source": capture_source}
           if capture_source is not None else {}),
        "window_seconds": runtime["window_seconds"],
        "lookahead_seconds": lookahead,
        **({"timestamp_clock": "portaudio_adc_monotonic"} if clock_reporting else {}),
        "decoder": detector.decoder.as_dict(),
        "normality": {
            "enabled": True,
            "version": normality.metadata["normality_version"],
            "meaning": "0-100 similarity to reference normal hits",
        },
    }
    try:
        with sd.InputStream(
            samplerate=sample_rate,
            channels=1,
            dtype="float32",
            blocksize=block_samples,
            device=selected,
            callback=callback,
        ):
            # Do not claim to be listening until the microphone stream opens.
            print(json.dumps(listening_status), flush=True)
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
                block_end = block.started_at + len(block.samples) / sample_rate
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
                                **detector.event_metadata,
                                "time_since_start": round(event_time, 6),
                                **({"event_monotonic": event_monotonic(
                                    block_end, samples_seen, sample_rate, event_time)}
                                   if clock_reporting else {}),
                                "score": round(event["score"], 6),
                                "normality_score": round(normality_result.score, 1),
                                "notification_monotonic": time.monotonic(),
                            }
                        ),
                        flush=True,
                    )
                last_finalized = finalized
                # Sent AFTER all hits in this finalized interval: downstream
                # may finish a no-hit attempt only once this watermark passes it.
                if clock_reporting:
                    print(json.dumps({
                        "event": "AUDIO_PROGRESS",
                        "finalized_monotonic": block_end - lookahead,
                        "captured_monotonic": block_end,
                        "notification_monotonic": time.monotonic(),
                    }), flush=True)
            if clock_error:
                raise RuntimeError(clock_error)
    finally:
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        print(json.dumps({"status": "stopped", "captured_seconds": samples_seen / sample_rate}), flush=True)
