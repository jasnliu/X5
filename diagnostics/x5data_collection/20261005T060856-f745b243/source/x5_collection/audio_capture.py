"""Standalone TONOR WAV writer. No model, labels, training, or motor imports.

Run with ST7's existing Python environment for sounddevice. Capture-only ADC
timestamps are logged to match encoder times later; onsets are NOT guessed here.
"""
import argparse
import json
import math
from pathlib import Path
import queue
import signal
import time
import wave


def write_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2)+'\n')
    temporary.replace(path)


def main():
    import sounddevice as sd
    p = argparse.ArgumentParser()
    p.add_argument('directory', type=Path)
    args = p.parse_args()
    directory = args.directory
    stop = False
    errors = []
    blocks = queue.Queue(maxsize=200)

    def stopped(*_):
        nonlocal stop
        stop = True

    def callback(data, frames, timing, status):
        now = time.monotonic()
        age = timing.currentTime-timing.inputBufferAdcTime
        if status or not math.isfinite(age) or not -.01 <= age <= 1.:
            errors.append('Audio status/ADC clock: '+str(status)+' age='+str(age))
        try:
            blocks.put_nowait((bytes(data), frames, now-age, now))
        except queue.Full:
            errors.append('Audio queue overflow')

    signal.signal(signal.SIGINT, stopped)
    signal.signal(signal.SIGTERM, stopped)
    total = 0
    started = None
    last_status = 0.
    try:
        with wave.open(str(directory/'raw.wav'), 'wb') as wav, (directory/'audio_blocks.jsonl').open('w') as timestamps:
            wav.setparams((1, 2, 16000, 0, 'NONE', 'not compressed'))
            with sd.RawInputStream(device='pipewire', samplerate=16000, channels=1,
                                   dtype='int16', blocksize=320, callback=callback):
                while not stop and not errors:
                    try:
                        data, frames, adc, received = blocks.get(timeout=.5)
                    except queue.Empty:
                        errors.append('Microphone stopped supplying samples')
                        break
                    if started is None:
                        started = adc
                    wav.writeframesraw(data)
                    timestamps.write(json.dumps(dict(frame=total, frames=frames,
                        adc_monotonic=adc, received_monotonic=received))+'\n')
                    total += frames
                    if received-last_status >= .25:
                        write_json(directory/'audio_status.json', dict(ready=True,
                            frames=total, first_adc_monotonic=started,
                            heartbeat=received, errors=errors))
                        last_status = received
    except BaseException as exc:
        errors.append(str(exc))
    result = dict(sample_rate=16000, channels=1, sample_width=2, frames=total,
                  duration_s=total/16000, first_adc_monotonic=started, errors=errors)
    write_json(directory/'audio_result.json', result)
    print(json.dumps(result), flush=True)
    return int(bool(errors))


if __name__ == '__main__':
    raise SystemExit(main())
