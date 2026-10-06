# X5 hybrid-hit audio dataset

**Status: collected and exported; onset labels are drafts, not audited ground
truth. No model training or model inference was performed.**

- Source session: `20261005T060856-f745b243` (UTC identifier; October 4 locally).
- Microphone: TONOR TD510, verified PipeWire source routing.
- 30 WAV/CSV pairs: 20 isolated examples and 10 double-hit examples.
- 40 draft `cymbal_hit` onset labels; about 182 seconds of exported audio.
- WAV only: 16 kHz, mono, signed PCM16, no clipping detected.
- CSV: `recording_file,hit_time_seconds,label`, with one row per onset.
- Filenames: `recordings/r1.wav` through `recordings/r30.wav`, paired with
  `timestamps/t1.csv` through `timestamps/t30.csv` in collection order. Matching
  numbers identify a pair. Seconds are relative to each WAV, not to the original
  recording or motor clock. CSV recording references and manifest entries use
  these names; each manifest `source_id` retains the original session/block ID.

## Motion diversity and safety evidence

Commanded J7 downward displacement from the **record3 endpoint** was 10, 10.5,
11, 11.5, or 12 degrees. Each setting contributed four singles and two pairs.
The existing experiment hybrid method was used, including partial returns
within pairs and full returns between blocks. Six-second ringdown gaps
separated blocks. Measured stroke peaks were 10.001–11.407 degrees; larger
commanded depths were not assumed to have been reached exactly. Pair audio
onsets were about 202–215 ms apart.

The successful batch ended with verified center **before** relaxation, followed
by disabled/fault-free readback of all 16 motors. No further hardware is needed
to review or use these files. This collection does not demonstrate improved ST7
detection performance.

## Annotation provenance and limitation

All waveforms/spectrograms were visually reviewed against existing ST7 cymbal
examples (`r1` and `r10`). High-frequency onset analysis aided placement;
second-hit timestamps were refined to distinguish a new attack from the first
hit's ringing. Encoder release times only located review windows. There was
**no auditory review**: this assistant interface could not listen to the audio.
Consequently all entries are explicitly `label_status: "draft"` and
`human_verified: false`; review before training or claiming labeling accuracy.

PCM samples were copied directly from the raw capture, without normalization,
denoising, or filtering. Clips have 1.5–2.5 seconds of pre-onset audio and four
seconds after the last labeled onset, broadly matching the existing ST7 clip
style. The filters used for onset analysis were not applied to the saved WAVs.

The unmodified ST7 data loader accepted all 30 pairs and 40 labels, including
hash verification. Format compatibility is **not** semantic label validation.
All files are from one physical session, so their manifest group is shared and
`split` is `unassigned`. Do not treat random clips from this session as an
independent held-out microphone/session evaluation. No artificial variants or
sub-10-degree attempts were added.

## Audit files

- `manifest.json`: per-file hashes, onset times, clip offsets, actual peak
  displacement, commanded displacement, group, and annotation status.
- `../../diagnostics/x5data_collection/20261005T060856-f745b243/raw.wav`: original
  continuous capture, retained outside this dataset to avoid duplicate samples.
- The same session directory contains `review.json`, onset review plots,
  encoder traces, capture clocks, and safe-shutdown evidence.
- [Verification report](../../diagnostics/x5data_collection/VERIFICATION.md).
- [Collector usage](../../X5_DATA_COLLECTION.md).

Existing ST7 data, models, and X5 saved recordings were left unchanged.
