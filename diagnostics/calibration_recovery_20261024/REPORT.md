# October 24 calibration recovery

## Result

**Passed with the original accuracy thresholds and no discarded active poses.**
The new default for all `plan_drums.sh` calibrated modes is:

`calibration_sessions/20261024T211629Z-1227b7b1-reprocessed/calibration.json`

SHA256: `7b7ed0d43f35994dec53792f72eeef0b9659ed7452751f47b478bce4711c943d`.

All 40 active observations remain: 35 training and the original five reserved
validation IDs 1–5. Capture 30 was already undone by the user; its image also
remains intact. No new exclusion, threshold change, intrinsics adjustment,
board-size adjustment, encoder-zero change or shared-URDF edit was made.

## Why the original attempt failed

The old rigid fit gave 4.726 mm / 1.317 degrees / 4.530 pixels RMS. Its existing
bounded local-kinematic refinement improved that to 2.504 mm / 0.964 degrees /
2.189 pixels, but one maximum orientation error was **2.1177 degrees**, above
the unchanged 2-degree limit. The error display misleadingly reported the
discarded rigid fit after refinement failed; it now retains the actual refined
metrics and separately records the rigid baseline.

The accepted IPPE image poses had not received a nonlinear per-image corner
refinement. The previous robot/camera fit minimized pose residuals, not the
measured corner residuals themselves. A bounded training-only study compared
eight variants; the chosen fixed fallback combines independently refined image
poses with both corner and pose residuals in the same 17-parameter model
(camera transform, unknown board mounting transform, bounded effective J2–J6
offsets). Arbitrary rigid board mounting orientation is permitted by that model.
The result establishes that these observations can pass the existing checks;
there was no evidence requiring rejection of the user's captures or board mount.
It does not prove a specific mechanical source for every residual error.

Each per-image LM step uses only that image's unchanged measured corners, with
fixed factory intrinsics, 100 iterations and 1e-12 stopping tolerance. The joint
fit uses scales of 5 mm, 1 degree and 1 pixel, linear least squares, the original
five-degree offset bound, convergence/rank checks and maximum 350 evaluations.
No fit, pose weighting or policy was tuned against reserved validation outcomes.
The candidate was frozen in `frozen_selection.json` before validation evaluation.
The regular calibration utility now tries exactly this fallback only after its
original methods fail, not the eight-variant research sweep.

## Accuracy results

| Check | Position RMS / max, mm | Orientation RMS / max, degrees | Pixel RMS / max |
|---|---:|---:|---:|
| All 35 training observations | 3.445535 / 7.831282 | 0.977180 / 1.941217 | 1.788365 / 3.970828 |
| Five-fold training-only predictions | 3.858381 / 8.456311 | 0.994690 / 1.929486 | 2.256927 / 4.882474 |
| Reserved five validation observations | **3.581183 / 5.699054** | **0.967404 / 1.471525** | **1.783288 / 2.966540** |
| Unchanged permitted thresholds | 5 / 10 | 1 / 2 | 4 / 8 |

The training-only normalized crossvalidation error ratio is about 0.444 versus
its rigid baseline (required <0.8). Rotation is close to the existing RMS limit;
these results are not a claim of large statistical margin or universal accuracy.
Independent evaluation using `safe_zone.geometry.Model`, rather than the
solver's batched FK, reproduces the held-out results.

The production command completed successfully using system Python:

```bash
./calibrate_camera.sh solve calibration_sessions/20261024T211629Z-1227b7b1 \
  --output calibration_sessions/20261024T211629Z-1227b7b1-reprocessed
```

The output now exists; use a different NEW output directory for a repeat solve.
See `reprocess.log`. Original images/corners/poses/session/report and the entire
previous approved calibration remain byte-identical to the before snapshot.
There is one retained source copy of the raw images in the Git artifact policy;
the duplicate reprocessed image copies remain local and reproducible.

## Planner integration and actual live checks

Only the calibration default changed in `drum_planner/common.py`; a normalized
whole-file hash test verifies this. Timing, path generation, acquisition,
camera FOV, transforms' conventions, transport and source robot geometry were
not changed. The matching local kinematic correction is embedded by the existing
model generator, never sent as motor zero changes. Both right/left generated
models agree with independent FK; the left joint geometry remains unchanged.

CLI routing tests cover both arms of experiment, hardware, sparse hardware and
compatibility aliases, plus camera, plan, preview, hardware-check, hardware-view,
planned-view and live. Hardware routing uses mocks, not physical motor owners.
Demo also takes the shared default; pure saved-trajectory simulation does not
acquire or reinterpret camera data. Existing resumed sessions retain their bound
calibration rather than altering an already owned motor session; start a new
session to use the new calibration. Archived scenes/plans require their original
explicit `--calibration PATH` and are not silently relabeled.

Actual ordinary experiment launches were run serially:

```bash
./plan_drums.sh experiment --right --seconds 8 \
  --output diagnostics/calibration_recovery_20261024/live_right
./plan_drums.sh experiment --left --seconds 12 \
  --output diagnostics/calibration_recovery_20261024/live_left
```

Both displayed native windows, remained destination-unselected, captured five
fresh native-depth frames plus both-arm feedback, and loaded the new calibration
hash. The snapshots were frozen and devices released before preview. Acquisition
took **3.62 s right / 3.49 s left**. Both views were visually inspected; screenshots
are retained locally. Query wire audits contain **1,168 / 1,120** state-query
frames respectively, all exact allowlisted packets. Every received motor status
reported disabled and no fault. **No movement, torque, enable/disable, gripper,
centering or relax commands were sent.** Both processes exited cleanly.

An unconstrained dominant-plane fit to fresh depth points measured the likely
floor normal at **0.77 degrees from robot +Z** (4.35 mm plane RMS on quantized
occupied points). The same fresh points would be 3.49 degrees off with the old
transform. This is a coarse independent leveling check, not an absolute physical
alignment measurement. It must not be compared directly with the earlier
25-degree scene, which was taken at a different camera placement.

No physical trajectory was run and no clearance/safety certification is claimed.
The calibration is valid for the camera/base mounting used in these captures;
subsequent movement requires a new check/calibration. Depth intrinsics/factory
color-to-depth extrinsics were preserved and checked by the normal acquisition.

## Regression checks

- `final_calibration_tests.log`: **179 passed**; calibration math, old successful branches,
  fixed fallback, negative cases, heldout isolation, immutable copy/reload,
  both-arm default routing/model checks and acquisition routing.
- `planner_tests.log`: initial broader run, 210 passing; archived-fixture errors
  correctly rejected attempts to pair old scenes with the new default.
- `planner_recheck.log`: after binding those historical fixtures explicitly to
  their original calibration, **162 passed**, with **three pre-existing dated
  hash-only failures** for `destinations/right/destinationR1.json`. That file
  already differed from the October 21/22 references before this task, and was
  left untouched. The historical assertions were not weakened to conceal it.
- `verification.json` / `verification.log`: original data/source preservation,
  all 41 original/copied image hashes, fresh-scene calibration identity and
  read-only CAN packet audit. `verify.py` repeats these checks from saved inputs.

No dependencies or hardware firmware/settings were modified. Production fitting
uses saved observations and no camera/CAN access; all live access above was
limited to the separately identified query-only integration check.

Reference: [OpenCV solvePnPRefineLM](https://docs.opencv.org/4.11.0/d5/d1f/calib3d_solvePnP.html).
