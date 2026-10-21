# Snare ownership and physical playback verification — PASS

Verified October 9, 2026, on the real robot, with final source:

```bash
./start_beat.sh --hardware --camera /dev/video4 --verify-playback
```

## Final physical run (run5)

The normal main app used real CAN feedback, camera, shared three-model audio,
and hi-hat startup calibration. Its existing opt-in playback verifier invoked
the real Run button, forbade strikes, and requested Center + Relax after playback.

1. Both arms centered and grippers closed.
2. Left `left_recordings/record1.json` played completely.
3. Right `recordings/record3.json` played completely.
4. Beat/search skipped; left recording reversed completely while right centered.
5. Left completed its center leg; each side relaxed only after its own fresh,
   measured settled-center proof.
6. All 16 motors reported disabled, and every launched process exited cleanly.

Evidence: `/home/jason/Proyectos3/X5/playback_results/dual_playback_verification/20261009T213737-c5abb609`.

- Application/control faults: **0**.
- Motor fault frames: **0**, across **85,362** feedback frames.
- Off-center/unproven active disable commands: **0**, independently audited.
- Verified recording endpoints: **3** (left forward, right forward, left reverse),
  retaining the original **0.05 degree** precision checks.
- Maximum joint error at disable: left **0.05495 degrees**,
  right **0.03297 degrees**.
- Existing center gates retained: 0.20 degree error, 0.12 degree span, 0.6 s settling.
- Beat/snare strikes executed: **0**. This is not full-band/strike verification.
- Maximum observed motor temperature: **37.0 C**.
- Independent post-exit query: **all 16 disabled, no fault bits**;
  48 state-request packets, no movement/enable/disable/mode writes.
- No fault recovery, emergency stop, or manual intervention was needed in run5.

## Changes

- `SnareSession.finished` now means no pending/active stroke owns the path,
  rather than requiring the arm to sit at the snare recording endpoint.
  Fresh running feedback, worker health and full-stroke return checks remain.
  A pending shared-clock start also blocks handoff.
- Latched dual-arm faults and requested shutdown no longer repeatedly re-enter
  motor recovery merely because the audio listener is stopping. Audio error
  display and active-playback audio-loss protection remain.
- Physical trials exposed left-J6 path/hold oscillation with a fixed commanded
  endpoint. That path now uses the drive's same-sample reported velocity and
  path-only gains Kp=160, Kd=4, load-estimator gain=40 (formerly 40/1.8/60).
  Motion references and the existing 3 Nm calculated net-torque cap are unchanged.
  The right controller retains its original defaults. The powered strike
  trajectory, strike gains, 11-degree depth and 0.220 s return are not modified.
- No endpoint, center, joint-limit or safety acceptance threshold was widened.

## Regression and preservation

- **905 tests passed in 238.57 s** on final, physically exercised source.
- New coverage includes idle vs strike-anchor ownership, pending starts, full
  returns, stale/faulted/disabled feedback, shutdown error churn, same-sample
  velocity, unchanged right defaults/reference path and the net-torque cap.
- `tested_source_final.json` hashes match the physically tested source afterward.
- All 7 paths in the initial protection manifest remain unchanged (recordings,
  snare motion reference, right strike controller and left center definitions).
- Model files, datasets, timing-calibration settings and audio optimization were
  not edited by this fix.

## Earlier failed trials — retained, not counted as passes

- run1: original ownership bug fixed; left endpoint oscillation caused soft-hold
  timeout and incomplete gain restoration. A separate exclusive recovery restored
  the exact saved volatile gains and centered both powered arms before disabling.
- run2: reported velocity alone did not cure the endpoint oscillation. Normal
  Center + Relax recovered both arms successfully after the failed playback.
- run3: slower bias adaptation alone still failed the strict endpoint check;
  normal Center + Relax recovered both arms.
- run4: both forward recordings passed with the damped path hold, but reverse
  ended 0.066 degrees off target (0.05 required). Right had already centered and
  relaxed; a separate exclusive recovery centered and then relaxed the left.
- run5: final load-estimator rate plus damped hold passed every requested stage.

Trials 1 and 4 required stopping the old writers while retaining their stationary
firmware-held targets, then acquiring both CAN locks for the recovery controller.
No enable or mode change was sent during takeover; no off-center relax was used.
All failed-trial and recovery wire audits report **zero motor fault frames and
zero unproven/off-center active disable commands**. Their errors are preserved.

Unrelated nonfatal Qt font, camera low-light-control and RViz stereo warnings
remain; they are not motor/control faults and were not changed in this task.

## Files

- `run5/app.log`, `run5/summary.json`, `run5/can_audit.json`: final acceptance.
- `run5/can.log`: independent wire trace.
- `run5/final_query_disabled.json`: independent disabled-state confirmation.
- `full_tests_verified.log`: final regression output.
- `before/`, `changes.diff`, `protected.json`, `tested_source_final.json`: audit.
- `run1` through `run4`: failed runs and any separate recovery evidence.
