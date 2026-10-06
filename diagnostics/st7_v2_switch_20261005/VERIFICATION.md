# X5 ST7 model V2 selection — 2026-10-05

## Runtime change

All audio-enabled modes of `start_beat.sh` / `launch_right_camera_playback.py`
now use sibling `/home/jason/Proyectos3/st7`, with the shared audio bridge
explicitly passing:

- `--config /home/jason/Proyectos3/st7/models/v2/config.yaml`
- `--checkpoint /home/jason/Proyectos3/st7/models/v2/best.pt`
- `--normality-artifact /home/jason/Proyectos3/st7/models/v2/normality/reference.npz`

The launcher and bridge share the same asset preflight. Missing V2 files stop
startup rather than silently selecting baseline/V1 or the old `/home/jason/st7`
checkout. Startup/status text identifies the model as V2 and distinguishes the
model version from the normality artifact's format version.

This covers default camera playback, `--hardware`, and `--hardwaretest`. Pure
`--test` still does not require or launch ST7. The capture-only X5data collector
does not load a detection model, so it needed no changes. Already running X5
processes must be restarted to load the new selection.

## Verification

- **100 focused offline tests passed:** 5 model-selection tests, 90 camera
  playback tests, and 5 audio-timing tests. Logs are in this directory.
- Launch-description checks used mocked ROS process constructors: verified
  sibling-root selection in all audio-enabled modes, no ST7 in pure simulation,
  and failure before process construction when V2 is unavailable.
- Missing-asset tests verified no fallback even when old artifacts are present.
- Parsed the exact command with ST7's real CLI and confirmed config/override
  paths agree for the checkpoint and normality reference.
- Ran ST7's actual file-inference command on `r56.wav` and `r76.wav`: one and two
  detected events respectively, with finite normality scores in the 0–100 range.
  These existing training examples are integration smoke checks, **not**
  independent accuracy measurements. See `model_load_verification.json` and
  `r56_v2_file_inference.json` / `r76_v2_file_inference.json`.
- Loaded checkpoint SHA-256 matches V2's completion report:
  `ea4038cca1db1a1005fc3bd4e51ec69617bdc3345cfe6efeddba2047662e5565`.
- All **421 protected files** retain their original hashes, covering both model
  versions, ST7/X5 datasets, saved arm recordings, ST7's standalone default
  config/launcher, and X5's shell launchers. See `verification.json`.

No arm commands, live microphone capture, training, calibration fitting, model
copying, or detector threshold tuning were performed. Y2 camera selection,
robot motion, audio timestamp forwarding, and fault handling were not changed.

## Known V2 model limitation

The existing ST7 `models/v2/TRAINING_REPORT.md` reports 40/40 detections on the new
hybrid session used for training, but original held-out test precision/recall
of 66.7%/66.7% versus V1's 100%/100%. It identifies regressions on older rapid/
multiple-hit recordings. This requested version switch does not establish that
V2 is universally better, and no new physical or unseen-session claim is made.
