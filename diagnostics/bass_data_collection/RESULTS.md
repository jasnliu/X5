# Bass collection result — 2026-10-10

## Completed

60 WAV/CSV pairs exported under `X5data/bass`, r1/t1 through r60/t60.
222 reviewed bass impacts; 480.04325 seconds, mono 16 kHz PCM16.
No clipping, duplicate WAV hashes, missing pairs or audio-capture errors.
Every exported WAV is byte-identical to its corresponding raw capture slice.
No training, model inference, bass calibration, force adjustment or firmware flash.

All 1,072 protected existing files match their original SHA-256 hashes, including
prior instrument data/models, saved arm recordings, configuration, shared
controllers, ESP source and original launchers. See `preservation_final.json`.

## Successful physical groups

| Session | Clips | Bass impacts | Left return |
|---|---:|---:|---|
| 20261010T232155-train-63812811 | r1–r2 | 5 | Pilot finished before reverse-first instruction; center verified before relax |
| 20261010T232632-train-65a1ae8b | r3–r4 | 5 | Reverse, verify start, center, verify, relax |
| 20261010T232851-train-4e7f0cb9 | r5–r36 | 122 | Reverse, verify start, center, verify, relax |
| 20261010T233449-validation-5fd0ac49 | r37–r48 | 45 | Reverse, verify start, center, verify, relax |
| 20261010T233755-test-9e4798d8 | r49–r60 | 45 | Reverse, verify start, center, verify, relax |

All groups completed with strict dual-arm center-before-disable evidence and a
query-only final check proving all 16 motors disabled and fault-free. Full left
reverse duration was 11.665686732 seconds. Last left center maximum error was
0.032969 degrees and right 0.066436 degrees, both with >=0.6 seconds settled
history. Relaxation may subsequently allow passive joint movement; center proof
refers to the instant before relaxation, not a claim of powered position hold
while disabled. No emergency off-center relaxation was used.

Passive CAN audits found no motor fault frames and no active disable without
fresh center evidence. The largest observed motor temperature across audits was
38 C. `physical_main/exit.json` confirms exit 0. Camera/CAN capture processes and
the motion collector exited; no collection controller was left running.

## Startup attempts retained for transparency

- `20261010T231707-train-8f29f616`: accessory preflight stopped before arm enabling
  or bass kicks. The old check incorrectly required the coasted encoder position
  to stay near zero after the firmware's verified open-zero crossing.
- `20261010T231820-train-91a0fe74`: left endpoint settling stopped before any clips
  or bass kicks. Both arms returned to verified center and relaxed. The bass-only
  transport then adopted the main beat's existing corrected J6 path/hold gains
  and same-sample drive velocity, preserving bounds and strike settings.

Both occurred before the later reverse-first instruction. No failed-attempt
clips were exported. Shared controller code was not changed.

## Review and verification

Final CSVs contain `bass_hit` only. Every full clip was visually reviewed using
waveforms and spectrograms; low-correlation or unusual-latency attacks received
enlarged review. Raw-template proposals were crosschecked with filtered waveform
alignment for r5–r60; all comparisons agreed within 3 ms (agreement is not an
independent absolute-error bound). No command-time labels were substituted.
Other-instrument proposals are explicitly unreviewed diagnostics and are not
part of the dataset. See each successful session's `review.json`.

Labels are assistant visual signal review, not independent human or auditory
verification. Accompaniment is not exhaustively labeled; do not use absent
accompaniment labels as negative ground truth in future multiclass merging.
Train/validation/test assignments are not training and do not establish model
quality or independence across environments.

Final offline regression: **34 tests passed** (`final_tests.log`), including
complete and interrupted left reverse, rejection of direct-center fallback after
reverse failure, fixed bass pulse/quarter-note timing through a fake serial PTY,
center guards, annotation scope, numeric pairing and no-overwrite export.
Physical evidence is separate from those offline tests.

Machine-readable combined audit: `final_verification.json`.
