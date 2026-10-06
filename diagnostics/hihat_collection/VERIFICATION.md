# Hi-hat collection verification — 2026-10-05

## Delivered

- `collect_hihat_data.sh` / `hihat_collection`: separate finite collector.
- `label_hihat_data.py`: offline reviewed-onset exporter; no model dependency.
- Existing 30 ride pairs and manifest moved to `X5data/ride`, byte-preserved.
- `X5data/hihat`: 60 WAV/CSV pairs; 30 positive clips, 30 negatives, 34 labels.
- 40 newly captured clips and 20 byte-identical ride copies, about 343.0395 s.
- ST7 datasets/models, saved arm recordings and existing camera/firmware code
  unchanged (382 protected-file hashes checked).

## Physical evidence and limitations

Session: `20261005T175549-54ef005a`.

Two successful ntfy notices preceded hi-hat and arm movement respectively.
TONOR routing was verified in `microphone.json`. The arm remained disabled for
the 20 hi-hat-only and 10 background clips. It then centered/closed the gripper,
played only record3, and completed all ten mixed hybrid strokes. Record3 endpoint
maximum error was 0.021979 degrees, settled for 0.628671 s. J7 measured peaks
ranged from 10.066537 to 11.385297 degrees, within the 12-degree corridor.
No camera, live detection, training, force tuning or firmware changes were used.

**All acquisition completed, but automatic shutdown failed.** The last capture
ended at monotonic 563387.546322. During return, the hi-hat parent-heartbeat
watchdog fired at 563389.547536; its normal O/wait/S path obtained release ACK
at 563391.055266. Arm settling then encountered a near-center joint-limit
violation at 563400.076289: J7 was approximately 0.1973 degrees beyond the
1.4 rad center. No unconditional/midair relax was issued.

A further ntfy warning preceded bounded near-center recovery. The recorded
center proof at 563444.042035 had maximum historical error 0.198312 degrees,
span 0.109897 degrees and settled duration 0.646425 s. Only then was relaxation
sent. `recovery_verified.json` and the subsequent independently audited
`query_final_disabled.json` confirm all 16 motors disabled; the latter also
checks fault-free responses. Powered time through center recovery was about
141 seconds. Analysis and export started only after shutdown.

`result.json` remains `success:false`, `relaxed_verified:false`. The additional
`post_capture_recovery.json` documents why completed audio remains usable;
it does not rewrite history. Hi-hat open/release evidence is commands plus ACK,
not an encoder-position measurement.

## Cleanup changes after the run

The new collector now requests accessory shutdown asynchronously before arm
centering. It also recognizes the already-existing bounded near-center region
when the arm enters it during return, rather than only at return start. Normal
strike geometry, motor settings and strict measured-center-before-relax gates
are unchanged. These changes passed offline tests; **the updated automatic
end-to-end path was not physically rerun** just to retest it.

## Offline validation

- 18 hi-hat/controller/export tests passed, including fake-PTY finite closure
  cycles, asynchronous stop, parent failure recovery, center-region bounds and
  rejection of unsafe/in-capture-error export.
- 13 existing ride collection/export regression tests passed.
- Offline `collect_hihat_data.sh --check` validates record3 and bounded plan.
- Raw capture: 302.96 s, 16 kHz mono PCM16, no capture errors or clipped samples.
- 60 paired WAV/CSV hashes, numbering, recording references, bounds, onset
  ordering, positive/negative counts and raw PCM preservation verified.
- Unmodified ST7 loader accepted 60 pairs/34 labels/30 empty negative CSVs with
  hashes verified. This was format checking, not model execution or training.
- 61 original ride audio/CSV/manifest files and 382 protected files unchanged.

Logs are in `setup_20261005/`; dataset checks are
`20261005T175549-54ef005a/{dataset_verification,loader_verification}.json`.

## Label evidence

Six positive waveform/spectrogram overview pages, five detailed onset pages,
one background page and two ride-negative pages were visually reviewed. All 34
closure onsets are signal-derived, not command timestamps or model predictions.
There was no auditory review or independent human review. Manifest entries are
explicitly marked visual/signal-review drafts. All new hi-hat material is one
session; paired ride-depth/timing settings are not a full factorial design.
No claims about trained-model accuracy or generalization are supported yet.
