# Fast workspace experiment (both arms)

```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh experiment --right
./plan_drums.sh experiment --left
```

Each normal launch captures **one new camera scene and new poses of BOTH arms**.
The scene retains the existing five-native-depth-frame fusion; it is not a single
depth image. The existing query-only acquisition brackets those frames with fresh
encoder feedback. Only the selected arm is required to remain stationary during
capture; passive-arm sway is not a stillness veto. Both arms must already report
disabled motors, and the Gemini camera and encoder buses must be available. The
program never enables, disables, holds, relaxes or moves motors/grippers.

After startup acquisition, the camera and encoder connections close. The scene
and poses then remain frozen throughout calculation/playback. No continuous
tracking or hardware readiness is implied. The **Recapture scene** button remains
an explicit camera-only refresh using the same startup arm poses; relaunch to
refresh the arm poses as well. Capture failure stops startup with a clear error;
there is **no automatic fallback to an old scene or pose**.

For reproducible software-only studies with the robot/camera disconnected:

```bash
# The bundled historical study scenes were captured before the camera moved:
OLD_CAL=calibration_sessions/20261017T211949Z-762f9908-reprocessed/calibration.json
./plan_drums.sh experiment --right --offline --calibration "$OLD_CAL"
./plan_drums.sh experiment --left --offline --calibration "$OLD_CAL"
# Explicit saved pair also selects offline mode:
./plan_drums.sh experiment --right --state /path/state.json --scene /path/scene.json
```

Normal fresh launches on **both arms** now use
`calibration_sessions/20261024T211629Z-1227b7b1-reprocessed/calibration.json`.
An explicit saved pair must match the chosen calibration; old scenes are never
silently relabeled or used as evidence of the current camera pose.

Offline launches are clearly labeled **OFFLINE SAVED INPUTS**. Partial saved-input
overrides require `--offline`; they cannot silently mix new and old acquisition.
The old `--capture-start` switch is now a redundant alias for normal fresh startup,
not a selected-arm-only refresh. Low-level `app.prepare()` study callers retain
their device-free default; the normal CLI explicitly requests `fresh=True`.

Each fresh session saves `acquisition/acquisition_report.json`, both-arm measured
state, the new depth frames/scene, host timestamp bracketing evidence and the
query-only CAN audit. Acquisition is isolated in a bounded job; only that job may
open CAN. All calculation/preview workers keep their existing CAN prohibition.

## Current workflow

The GUI starts with no destination selected and no calculation running. Select
from `destinations/right` or `destinations/left` to calculate one **fast
workspace** result. **Calculate fresh** repeats the complete calculation: it does
not restore a saved trajectory, timing solution or CPU reference. A finished
result stays in memory for playback; playing it is not another calculation.

The former five-method comparison is no longer the visible experiment menu.
Its implementations and historical reports are retained for offline comparison.
`--method workspace_curve` is the current pipeline; `--method all --headless`
remains an alias for the single current pipeline. Hardware modes and the original
CPU timing policies are unchanged. Hardware still uses its original Workspace
curve generator, not the new experiment orchestration.

**Add checkpoint** permits up to three ordered destinations. Each leg starts at
the preceding endpoint, stops and holds for two seconds. Multiple checkpoints
finish with an independently generated and checked return to original saved rest.
A single destination retains exact reverse playback. The frozen scene/self-mask
always belongs to the original saved snapshot, not the simulated next-leg start.
Changing selections cancels/reaps the old job before starting the new one.
Cancel, changing targets, removing checkpoints and Calculate fresh remain usable
following failure. No partial mission is presented as a complete trajectory.

## What changed

- Endpoint-only Cartesian workspace fits retain broad route alternatives and
  stick-tip/posture regularization. No recorded interior movement is reused.
- Seven moderate-prior proposals are tried first. Multiple surviving shapes
  trigger the stronger-prior family before ranking; if the only survivor fails
  final validation, that family is also tried. This rule is identical for both
  arms and does not recognize destination names.
- Coarse collision screens only reject proposals. **They never certify a route.**
  Continuous joint bounds, unchanged nominal collision geometry/margins, zones,
  2 ms final trajectory samples and 0.003 rad L1 edge checks remain mandatory.
- Geometry and original soft-clearance scores rank multiple surviving routes.
  Clearance probes are batched, and redundant soft scoring is omitted if there
  is only one candidate. This does not omit any hard check or increase a margin.
- Timing now intentionally uses the **CPU**: a fresh 24-control seed initializes
  a bounded 96-control sparse IPOPT solve. All seven joints constrain velocity,
  acceleration and jerk. It is not the expensive historical CPU-reference solve.
- Batched polynomial stationary-root checks cover every spline interval and both
  knot sides. This is the same continuous extrema mathematics as the retained
  original checker, not a coarse sampling replacement. A homogeneous clock
  correction preserves velocity **0.6 rad/s**, acceleration **1.2 rad/s²**, jerk
  **6 rad/s³**, rest endpoints and the existing 0.1% reserve.
- The exact previously generated polynomial path stays byte-for-byte unchanged;
  only its positive time density changes. Playback uses that exact law. No
  corner rounding, waypoint deletion, path refit or new internal stops.
- There is normally one sparse solve, with at most one extra extrema refinement
  for a meaningful between-sample violation. Each solve has 90 iterations and a
  one-second solver wall-time bound; initialization and verification also cost
  time. A checked incumbent is retained if a candidate is invalid or slower.
- One native frozen world is shared **only within one fresh checkpoint job**,
  after checking identical models, scene, tools, passive arm and policies. It is
  destroyed afterward. No previous route or scene certificate is reused.

The old mandatory CPU reference and strict per-request **1%** gate remain absent.
There is no global-optimality claim. The new policy was selected by movement
quality versus fresh calculation cost, comparing multiple clock representations,
CPU/GPU solves and higher-resolution offline references. References are **never
loaded by the runtime optimizer**. The previous GPU implementation remains intact
for comparison/rollback; neither hardware timing dispatcher is changed.

## Intentional CPU backend and explicit fallback

Normal logs and the preview say **CPU sparse timing — intentional CPU backend**.
GPU discovery/availability is no longer required by these two experiment modes.
CasADi/IPOPT is already installed in the planner environment; this change installs
or upgrades nothing. It does not promote the separate sparse hardware fork.

If sparse refinement is unavailable, fails, or does not beat its checked seed,
the logs and preview explicitly say **CPU TIMING FALLBACK**, with the reason and
selected checked seed/uniform clock. Nonconvergence is reported, not described as
an optimum. No unverified solver iterate is played. If the batched eigensolver
fails, the original continuous root checker is used and logged explicitly.

Reports include the requested/actual backend, selected stage, fallback reason,
implementation identity, solver status/iterations, continuous extrema, and
verification/calculation times. `fast_timing_report.json`,
`experiment_report.json`, `selected_candidate.json` and the exact `plan.json`
remain beside each leg. The isolated 3 GiB job cap, bounded serial work and
process-tree cancellation remain. No additional workers or persistent result
caches are introduced.

## Measured balance (2026-10-24, saved inputs)

Right saved-rest → R1 → R4 → independently planned return:

- Movement, excluding unchanged holds: **10.768 → 10.427 s** (3.16% shorter).
- Fresh full-calculation median: **7.060 → 7.002 s** over three runs each.
- Current cold launch including saved-scene/session preparation: **8.239 s**.
- Left nominal movement: **2.980 → 2.857 s** (4.13% shorter); full calculation
  **6.371 → 6.824 s** in the final before/after comparison.

All geometric coefficients and limits stay identical. Doubling timing resolution
saved just 26 ms more across the right mission while adding about 1.65 s of timing
computation, so it is not the default. These are software measurements, not a
proof of global time optimality or physical arm performance. The final suite has
672 passing tests; both-arm native playback/checkpoint checks also passed.
Detailed provenance and alternatives are in the local diagnostic report below.

## Inputs and interpretation

Offline study states/scenes are identified by hashes in
`drum_planner/experiment/default_inputs.json` and `default_left_inputs.json`.
Right uses zone3; left uses zone1. The original gripper TCP, not the stick tip,
defines zone membership. Custom tools/sticks still participate in collision
checks. The opposite arm remains fixed in the simulation. Unseen space is not
certified clear; this remains frozen-scene planning, not live obstacle avoidance.

`--budget` / **Search cap per leg** bounds proposal fitting, not a requested
calculation duration. Gesture settings expose only the joint-travel and
clearance ranking weights used by this pipeline. The original full settings
schema remains accepted for compatibility; legacy corridor/seed settings do not
control this deterministic workspace fitter.

Cold timings, discarded alternatives, both-arm/generalization checks and
preservation evidence: `diagnostics/timing_speed_20261024/REPORT.md`.
The preceding speed-first GPU study is in `diagnostics/fast_gpu_20261024/REPORT.md`.
The superseded five-method/strict-1% documentation is preserved at
`diagnostics/fast_gpu_20261024/before/DRUM_PLANNER_EXPERIMENTS.md`.
Software tests and previews are not physical validation or motor authority.
