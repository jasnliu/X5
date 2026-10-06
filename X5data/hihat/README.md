# Hi-hat closure dataset

Collected October 5, 2026 using the TONOR microphone. This is a **separate
hi-hat target**, not an extension of the ST7 ride-cymbal labels. No training or
model inference was performed.

## Contents

| Pairs | Contents | Closure labels |
|---|---|---:|
| r1/t1 through r20/t20 | 16 single-closure and 4 two-closure clips | 24 |
| r21/t21 through r30/t30 | Real hi-hat/ride mixtures | 10 |
| r31/t31 through r40/t40 | Background, no closure | 0 |
| r41/t41 through r60/t60 | Copies of 20 existing ride-only clips | 0 |

60 pairs, 30 positive clips and 30 negative clips; 34 onset labels and about
343 seconds of exported audio. Filenames are `recording/rN.wav` and
`timestamp/tN.csv`. WAVs are 16 kHz, mono, signed PCM16. CSV columns are:

```csv
recording_file,hit_time_seconds,label
```

Positive rows use `hihat_close`; negative CSVs contain only the header. Times
are seconds relative to the paired WAV. A ride hit overlapping a closure does
**not** remove the closure label. Ride-only negative timestamps are newly empty
hi-hat annotations, not copies of the original ride-positive labels.

## Capture and annotation

The existing swing hi-hat settings were used without firmware/gain changes:
110-degree close, normal open/return after 0.6 seconds, pairs 1.2 seconds apart.
Hi-hat-only/background capture occurred with the arm disabled. Mixed examples
used center + close gripper, **record3**, no camera, and one hybrid J7 attempt
per example. Commanded displacements were 10, 10.5, 11, 11.5 and 12 degrees,
each twice; measured peaks were 10.067-11.385 degrees.

All 30 positive waveforms/spectrograms and 34 enlarged onset windows were
visually reviewed. High-frequency energy-rise proposals locate the main
broadband closure attack; motor command times only guide the review windows.
Background and copied ride negatives were also visually reviewed. **No auditory
review was possible in this interface.** These are assistant visual/signal-review
draft labels (`human_verified: false`), not independently verified ground truth.
Review before training; decimal precision is not a claim of sub-millisecond
semantic accuracy. The beginning of quiet contact/motor noise may precede the
chosen main closure attack by several milliseconds.

New clips retain unfiltered, unnormalized raw PCM. Positive clips have varied
1.5-2.0 second pre-onset context and 3.5 seconds after the last onset. No clipping
or audio-capture errors were found. The 20 copied ride WAVs are byte-identical
to their originals. Original ride WAVs/CSVs and ST7 data/models are unchanged.

## Hardware shutdown

All capture blocks finished before shutdown problems. The automatic arm return
tripped a joint-limit check on a small J7 overshoot near center; relaxation was
refused. The hi-hat parent-heartbeat watchdog also fired during arm return,
then completed its O/1.5-second wait/S release-acknowledgement sequence.

After another ntfy warning, bounded near-center recovery verified stationary
center before arm relaxation. A fresh audited query confirmed all 16 motors
disabled and fault-free. **No labeling was done with the arm powered.** The
hi-hat firmware supplies command/release acknowledgement, not independent
encoder-position proof. The unsuccessful original result is preserved alongside
the later recovery evidence; this was not a fault-free automatic shutdown.

## Provenance and future evaluation

`manifest.json` includes hashes, clip offsets, label status, commanded/measured
ride depths, source sessions and shutdown notes. All new hi-hat examples come
from one session; copied ride negatives retain their original session group.
Splits are unassigned. Do not use random clips from this same capture as an
independent held-out-session evaluation. Mixed depth and timing offset were
paired, not varied independently. More session diversity may be needed later.

The unmodified ST7 data loader accepted all 60 pairs and 34 labels, including
empty negative CSVs and hash checks. This verifies file-format compatibility
only, not semantic accuracy or performance of a future hi-hat model.

- [Collector and export usage](../../HIHAT_DATA_COLLECTION.md)
- [Verification report](../../diagnostics/hihat_collection/VERIFICATION.md)
- Raw capture, plots, `review.json`, encoder traces and recovery evidence:
  `../../diagnostics/hihat_collection/20261005T175549-54ef005a/`.
