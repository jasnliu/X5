# Snare-hit audio dataset

**60 WAV/CSV pairs, 40 positive clips, 20 negatives, 45 snare onset labels.**
Captured with the TONOR microphone on 2026-10-07 local time (2026-10-08 UTC).

- `recordings/r1.wav` through `recordings/r60.wav`
- `timestamps/t1.csv` through `timestamps/t60.csv`
- `manifest.json`: pairing, category, source session, split, hashes and review provenance
- WAV: 16 kHz mono PCM16, unmodified source PCM; about 394.05 seconds total.
- CSV: `recording_file,hit_time_seconds,label`. Positive rows use `snare_hit`;
  no-snare files contain only the header. Times are relative to each paired WAV.

| Category | Clips | Snare labels |
|---|---:|---:|
| Snare alone (15 singles + 5 doubles) | 20 | 25 |
| Snare + ride | 8 | 8 |
| Snare + hi-hat | 8 | 8 |
| Snare + both | 4 | 4 |
| Ride only | 6 | 0 |
| Hi-hat only | 6 | 0 |
| Ride + hi-hat | 4 | 0 |
| Background/motor hold noise | 4 | 0 |

Every new snare strike commanded 11 degrees. Hi-hat used an acknowledged
90-degree target; ride used the existing collector's 10–12-degree variations.
Existing ride/hi-hat datasets were not modified.

## Splits and one replacement

`r1`–`r48` are development data; `r49`–`r60` come from a separate test capture.
Keep the test group out of fitting and threshold tuning. These sessions share
room, kit, microphone position and day; they are not broad domain-shift evidence.

**59 exported clips are newly captured; one is a preserved existing ride-only
negative.** The new block-41 ride attempt was too weak for a clear acoustic ride
verification. `r41.wav` is therefore a byte-identical copy of
`../ride/recordings/r1.wav`, with a new empty snare-label CSV. Its original ride
session group and hash are retained in the manifest. Do not copy its positive
`cymbal_hit` CSV into this target's annotations. The weak original capture remains
in diagnostics; it was not silently called a verified ride sound.

When combining instrument datasets, group this copied audio with its original
ride session to prevent duplicate-audio leakage across splits.

## Annotation quality

All 60 exported examples were visually reviewed using waveforms/spectrograms,
including enlarged windows for all 45 snare attacks. Three mixture onsets were
refined by matching the isolated snare attack and visually checking it again.
Motor command times only located search windows; labels are audio sample times.
No trained sound detector created the labels. **No auditory or independent human
annotation audit was performed**; manifest status is `draft_visual_signal_review`.
Six decimal places do not claim sub-millisecond annotation certainty.

No snare model has been trained or integrated by this collection task.
See [verification and source-session evidence](../../diagnostics/snare_data_collection/RESULTS.md).
