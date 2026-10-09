# Hi-hat offset checkpoint and random eighth-note snare

## Behavior

- **Hi-hat:** the fast existing acquisition loop continues until two consecutive
  valid closure/ride onset errors are within **15 ms**, at the same currently
  applied offset. It then logs `kind: locked` and freezes that offset for the
  rest of the run. Later drift or missing/ambiguous detections do not unlock it.
  New runs construct a new synchronizer and learn again from zero.
- Old delayed observations cannot certify a newer offset or a still-ramping
  pair. Invalid/missing pairs and out-of-tolerance errors break the pre-lock
  streak. Consecutiveness uses closure IDs, not notification spacing, so 20 BPM
  still locks when consecutive closures arrive slightly over six seconds apart.
- **Snare:** each unchanged four-quarter-note measure has eight straight eighth
  positions. Random count is 1 or 2 with equal probability; a random feasible
  combination is then chosen. Distinct notes are guaranteed; no bar is omitted.
- Spacing includes the complete unchanged **11°** powered strike/return,
  settling and the existing 30 ms margin, including at measure boundaries.
  Minimum reference spacing is **0.4374633435 s**: 0.1474633435 s down + 0.220 s
  return + 0.040 s settling + 0.030 s margin. The next actual strike must still
  pass the measured-return/settling guard; no motion checks were relaxed.
- At 100 BPM, the selected grid spacing is at least **0.600 s**; at 120 BPM,
  **0.500 s**. Adjacent eighth notes become eligible at sufficiently low tempos
  (68 BPM gives 0.441176 s). Tempo and bar length are unchanged.

## Verification (offline only)

- `verification.json`: 2,000 random bars at each of 20, 60, 68, 100 and 120 BPM.
  Every tempo exercised all eight positions and both hit counts. All gaps,
  including cross-bar gaps, met the full-return minimum.
- Positive, negative and delayed-delivery hi-hat simulations each logged exactly
  one lock and retained an exactly constant scheduled offset afterward. Unit
  tests also cover later drift, outliers, missed hits, different historical
  offsets, a new run, and slow-tempo delivery jitter.
- `ui_test.log` / `simulation_ui_trace.json`: actual Tk/ROS **test-mode** app ran
  eight bars with **15 simulated snares**, covering every eighth-note slot.
  Stopping during the second hit of the final bar completed the unchanged return,
  reversed the left recording, centered both simulated arms, and relaxed only
  after both were centered. Camera/audio/serial/CAN device access was prohibited.
- Production worker tests use quantized fake J6 feedback and CAN-queue pressure;
  two-hit bars finish without overlap or missed returns. These are not hardware
  tests or proof of real acoustic strike alignment.
- **82 focused tests passed. All 808 repository tests passed** (236.05 s).
  Results are in `targeted_tests.log` and `full_tests.log`.

`unchanged.sha256` verifies unchanged standalone snare launcher/motion/tuning,
hi-hat angle/calibration, ride trajectory/workflow, recording-cache code, and the
selected left/right recording files. No physical movement or firmware flashing
was performed. Stop the running program normally and restart to load this code.
The existing broad cache fingerprint will require a new preflight after these
code edits; cache behavior itself was intentionally not changed.
