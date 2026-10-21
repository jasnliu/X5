# Bass drum dataset — collected 2026-10-10

Data collection only. **No training was performed.**

- 60 real recordings: `recordings/r1.wav` through `r60.wav`.
- 60 matching annotations: `timestamps/t1.csv` through `t60.csv`.
- 222 `bass_hit` annotations; 480.04325 seconds total (~8 seconds per clip).
- Mono 16 kHz, 16-bit PCM. Samples are unchanged slices of the continuous captures.
- No clipped samples; maximum absolute sample level 0.7031 of full scale.
- Fixed bass strike: existing ESP `K`, clockwise PWM 180/255 at 20 kHz for 50 ms,
  then coast. No bass calibration, strike-force change, or firmware flashing.

| Recorded context | Clips |
|---|---:|
| Bass only (6 singles, 6 quarter-note sequences) | 12 |
| Bass + hi-hat | 12 |
| Bass + ride | 12 |
| Bass + snare | 8 |
| Bass + ride + hi-hat | 8 |
| Bass + ride + hi-hat + snare | 8 |

Tempos: 80, 100 and 120 BPM. Four-hit sequences strike bass on every quarter note;
accompaniment is on beats 2 and 4 with nominal offsets 0 and +/-75 ms. These are
command-schedule offsets, not measurements of acoustic simultaneity.

## Labels and limitations

CSV columns: `recording_file,hit_time_seconds,label`. Times are seconds from the
start of the corresponding WAV. Only **bass** is exhaustively annotated here.
Hi-hat, ride and snare are accompaniment context in the manifest, **not**
exhaustively labeled. Missing labels for those instruments are **not negatives**
for their models when merging datasets in the future.

All clips received assistant visual waveform/spectrogram review, with enlarged
review of ambiguous attacks. An isolated 22 ms bass waveform was used for onset
alignment, with a 40–300 Hz filtered comparison for the main batches. Command
logs only located broad search windows; timestamps are not command times or a
constant command-time offset. Diagnostic accompaniment proposals were not
accepted or exported. No learned detector was trained or used for annotation.
These labels have not received independent human or auditory verification;
fine onset timing in overlapping hits remains an annotation limitation.

`manifest.json` records source captures, acoustic labels, hashes, pulse settings,
context, review method and physical-session groups. The 36/12/12 train/validation/
test assignments are future-use suggestions only. There are five physical
capture groups, with no group spanning splits, but all were collected on the
same day, in the same room and microphone setup. This is not evidence of
cross-room or cross-day generalization. No dedicated all-negative clips were
collected; natural background and decay remain within the clips.

## Physical safety and evidence

For all collection after the updated safety instruction (r3–r60), the left arm
reversed its played recording, verified its start, centered, verified center,
and only then relaxed. The r1–r2 pilot had already ended before that instruction
and used the earlier direct-center return. All five successful sessions verified
both arms centered before relaxation and all 16 motors disabled/fault-free after.
No hardware emergency relaxation occurred.

Raw audio, timing, review images, passive camera/CAN evidence and full results:
`../../diagnostics/bass_data_collection/` (relative to this folder).
See `RESULTS.md` and `final_verification.json` there. Two earlier unsuccessful
startup attempts remain in diagnostics and contribute no exported clips.

From the X5 project root, read-only verification:

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m bass_collection.export --verify
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. /usr/bin/python3 diagnostics/bass_data_collection/verify_final.py
```

The first command verifies the 60 pairs, hashes, formats, labels and grouping.
The second also checks raw-slice identity, saved safety evidence and the 1,072
protected pre-existing files. Neither command opens hardware or trains models.
