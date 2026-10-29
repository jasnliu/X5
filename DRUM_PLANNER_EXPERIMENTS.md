# Workspace curve experiment: editable joint timing and simulated starts

## Right J2 operating-position correction (2026-10-28)

Right J2 uses a **0° interior operating floor**, not a hard mechanical bound
on measured rest. The original camera/acquired pose may be between the reported
-1° stop and 0°: keep it exact, with continuous monotone departure/return inside
that margin. Do not command lower than the acquired rest, invent negative goals,
or dip below zero between positive checkpoints. A simulated spawn does not
replace the original camera-mask/rest reference. DT1 goals still require +1°.
Other bounds, left behavior, geometry, timing limits, calibration and drive
settings remain unchanged. Native endpoint arithmetic gets only 1e-12 rad of
roundoff allowance, never joint clipping. See the
[software-only rest-transition study](diagnostics/hardware_rest_transition_20261028/REPORT.md).

## 2026-10-26 hardware promotion (supersedes historical hardware exclusions below)

Normal `--hardware --right/--left` now reuses the current experiment path/recovery
and configurable CPU timing engine. Its own Calculate acquires and validates fresh
physical inputs; it never imports an experiment plan's eligibility. Right hardware
also offers fresh **dt1**. Both hardware and experiment keep their original distinct
selection/Calculate/Start workflows. Hardware does not expose simulated teleport.
`--hardware-sparse` remains the older opt-in fork. See [hardware controls](BASELINE_HARDWARE.md)
and [verification](diagnostics/hardware_current_20261026/REPORT.md).
All experiment algorithm and default-policy behavior is unchanged by this promotion;
only the paired-scene renderer was factored for shared use.


```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh experiment --right
./plan_drums.sh experiment --left
```

Each normal launch captures **one new camera scene and new poses of BOTH arms**.
Right experiments now capture synchronized RGB alongside native depth for the
optional **dt1** target. Left acquisition remains depth-only, unchanged.
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

If startup reports **Camera is busy (UVC error -6)**, close the other application
using the calibrated Orbbec (for example DT1), then run the planner again. The
error now includes the SDK's cause, the absolute `depth_stderr.log` path and,
when visible, processes holding this camera. Identification uses its calibrated
serial and current USB/video paths; reboot-time device numbering is not assumed.
The planner never kills another camera program, resets USB, changes permissions,
or substitutes cached inputs. A missing owner listing does not prove the camera
is free: another user's process or a driver may hold the interface. Other SDK
failures retain their actual error rather than being mislabeled as a busy camera.

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
from `destinations/right` (plus **dt1**) or `destinations/left` to calculate one **fast
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

## Camera-generated dt1 destination (right only)

The shared cymbal generator enforces a **3° position reserve at every joint
bound except J2 lower, which has 1° above its new 0° floor**, on newly generated dt1 endpoints. The green target and orientation rules
are unchanged. This endpoint-only rule also applies to the standalone destination
program and normal hardware dt1; it does not restrict the measured/simulated
start except for the explicitly checked original-rest transition. See [goal-reserve details](CYMBAL_DESTINATION.md#generated-goal-joint-position-reserve).

Run `./plan_drums.sh experiment --right` and select **dt1** in a checkpoint
dropdown. No separate DT1 window or prior destination-generation run is needed.
Keep other camera applications closed during startup, as before.

- Startup retains five distinct synchronized native-depth frames, with the same
  three-of-five confirmation, 5 mm occupied voxels, calibration and margins.
  The final frame's RGB/depth pair is saved in that same acquisition. It is not a
  second scene or an old destination from another launch.
- Selecting **dt1** first runs the existing DT1 perception and cymbal-prime
  endpoint solver on the frozen pair. The camera stays off. The unchanged
  constraints include the exact green target, above/near-parallel shaft,
  perpendicular J7 strike plane and **inward aim within ±25°**.
- Candidate endpoints and approach feasibility are checked against the
  experiment's actual five-frame collision world, current simulated start and
  fixed opposite arm. The perception self-mask remains at the acquired pose.
  The ordinary workspace path and selected timing policy then calculate the
  full route. Destination geometry is not substituted for route validation.
- The generated destination stays private to the experiment session under
  `dt1_goals/`; nothing is added to or overwritten in `destinations/`. Each
  complete selection/Calculate fresh generates anew, without a saved-goal
  fallback. It supports mixed saved/dt1 checkpoints, using the preceding
  endpoint to rank a later dt1 goal. It is not a new simulated-start option.
- Cancel/switch reaps the previous contained worker before another starts.
  Detection or IK failure leaves no playable stale route; select a saved target
  or Calculate fresh to retry. Recapture scene regenerates selected dt1 targets
  against the replacement scene, retaining the original saved arm poses.
- Left/hardware/`--fast` destination selectors, standalone DT1/cymbal generator,
  path/timing algorithms, joint limits and physical execution are unchanged.

Optional CLI selection, still simulation-only:

```bash
./plan_drums.sh experiment --right --destinations dt1
./plan_drums.sh experiment --right --headless --destinations dt1
./plan_drums.sh experiment --right --destinations R1 dt1
```

Explicit offline replays can use a previously captured **paired experiment**
scene and matching state/calibration. Older depth-only study scenes still work
with saved destinations; selecting dt1 reports that RGB is unavailable rather
than acquiring devices implicitly or using an old target. The paired scene uses
an experiment-only schema/reader; hardware depth-scene acceptance is unchanged.
Evidence: [`diagnostics/experiment_dt1_20261026/REPORT.md`](diagnostics/experiment_dt1_20261026/REPORT.md).

## Failed-route recovery (2026-10-26 UTC)

Successful original Workspace curves are retained, unchanged. If both original
proposal families fail, **normal experiments only** now try bounded quintic
B-splines. Fitting and playback use the identical polynomial: bounded control
points enclose the entire curve, avoiding the previous fit-then-interpolate
joint-limit overshoot. Three fixed controls at each end preserve the exact
start/goal and zero endpoint velocity/acceleration, with no internal corner stops.

If needed, symmetric distal-joint posture guides shape the departure. These are
optimization guides, **not extra destinations**. The final three controls are
restored to the actual selected endpoint, then the actual curve is checked.
Bounded late-control blends can remove unnecessary steering without changing the
collision-critical departure. When forward constructions fail, a fresh fit from
the other endpoint is tried and parameterized in the requested direction. A
checkpoint return is still independently generated; no previous leg is replayed
or used as a template. Every search retains the original acquired camera mask.

All recovery paths are ranked by the existing geometry/clearance objectives and
pass continuous position extrema, the unchanged per-joint timing checks, and
final dense full-arm/tool collision and zone validation. Invalid endpoints remain
invalid: this does **not** clip joint positions, widen limits, delete depth points,
change calibration, or ignore the opposite arm. Fitting remains serial/bounded
(20-second recovery search cap within the per-leg search budget); mandatory
validation is separate. No result caching or recorded-route imitation is used.

Implementation: `experiment/workspace_recovery.py`, invoked lazily by `fast.py`.
Shared `workspace_curve.py`, hardware planners and the separate `--fast` mode
are unchanged. Real-camera offline reproduction and quality evidence:
[`diagnostics/workspace_routes_20261026/REPORT.md`](diagnostics/workspace_routes_20261026/REPORT.md).

## Simulated starting position and per-joint limits (2026-10-25)

These controls belong to **normal `experiment --right/--left`**, not `--fast`.
The controls update left path generation/ranking unchanged. The later recovery
update below extends only failed experiment searches. Robot geometry, calibration,
joint-position bounds, zones and collision margins remain unchanged.

- **Simulated start** defaults to **Default (captured start)**. Choose a saved
  destination to teleport only the displayed selected arm to that exact joint
  pose. No approach from measured rest is inserted. The opposite arm stays at
  its captured pose. The normal return ends at this selected simulated start.
- The real acquired poses remain separately recorded. Camera robot self-masking
  always uses the acquired selected-arm pose, including all later checkpoint
  legs and the independently planned return. Teleporting does not reorient,
  recapture, remesh, recalibrate or reinterpret the camera scene. Explicit
  camera-only Recapture also retains the acquired pose for masking.
- **Joint limits / profiles…** edits seven independent values for each of
  velocity **rad/s**, acceleration **rad/s²** and jerk **rad/s³**. Values must
  be positive and finite. Invalid input is rejected, not clipped. There are no
  position-bound, current, torque, temperature or firmware controls here.
- **Original defaults** restores 0.6 / 1.2 / 6 for every joint. Save/load JSON
  profiles with explicit arm and units; the file dialog initially offers
  `config/experiment_motion_profiles/right` or `left`. Opposite-arm profiles
  are rejected. Higher custom limits are prominently **simulation-only and
  not hardware-validated**. Normal new launches retain original limits unless
  a profile is explicitly supplied; there is no hidden last-used override.
- **Timing effort** offers Fast, Balanced (default) and Thorough. Every level
  retains its fastest continuously checked incumbent. Balanced spends extra
  effort when the first solve is unfinished or needs refinement; Thorough also
  tries a 192-control clock. No global-optimum claim is made. Calculation logs
  distinguish solver stops, checked-seed fallback and actual accepted gains.
- The main view shows the highest-used constraint. Reopening Joint limits shows
  the achieved V/A/J peaks and utilization for all joints, aggregated across
  checkpoint and return legs. These are calculated peaks, not motor telemetry.

Changing start, limits or timing effort invalidates the displayed plan, cancels
and reaps obsolete work, and recalculates the currently selected target sequence.
There is still no trajectory calculation until a destination is selected.
If start equals target, the checked result has **zero movement time** and the
normal two-second hold; it does not fit an unnecessary loop. Unsafe/blocked
stationary poses are still rejected by the original native collision checks.

For isolated timing comparisons, use the same start, target, scene and geometric
preferences. Only timing changes with the limit profile; the generated polynomial
path is preserved exactly. All geometric proposals are still ranked as before,
not re-ranked to chase speed at the expense of the approved movement design.

CLI examples (normal launches still acquire fresh inputs):

```bash
./plan_drums.sh experiment --right --start R1 --destinations R2
./plan_drums.sh experiment --left --start default --destinations L1
./plan_drums.sh experiment --right --start R1 --destinations R2 \
  --motion-profile /path/right_limits.json --timing-quality thorough
```

For device-free headless tests, also supply `--offline --state ... --scene ...
--headless --output NEW_DIRECTORY` and the calibration matching that saved scene.
Destination names, stems and JSON paths are accepted. The selected destination
and simulated-start files are snapshotted; editing a source file mid-calculation
does not silently change the running job.

Profile format (all 21 values are explicit):

```json
{
  "schema": "x5-experiment-motion-profile-v1",
  "side": "right",
  "name": "Original limits",
  "simulation_only": true,
  "timing_quality": "balanced",
  "velocity_rad_s": [0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6],
  "acceleration_rad_s2": [1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2],
  "jerk_rad_s3": [6, 6, 6, 6, 6, 6, 6]
}
```

The implementation is `experiment/{motion_settings,motion_timing,motion_ui,scenario,
stationary}.py`. Existing `fast_timing.py`, all hardware timing implementations,
and the separate `fast_mode/` are unchanged. See
`diagnostics/experiment_controls_20261025/REPORT.md` for measured effort/quality
comparisons, unchanged-path evidence, both-arm UI tests and limitations.

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
- Timing intentionally uses the **CPU**: a fresh 24-control seed initializes
  bounded sparse IPOPT solves with 21 explicit per-joint constraints. Fast uses
  96 controls / 90 iterations / 1 s per solve; Balanced can add a 96-control /
  400-iteration / 4 s solve; Thorough can add 192 controls / 700 iterations / 8 s.
  Each stage permits at most one extra between-grid extrema refinement. A
  converged, sufficiently checked same-resolution solve is not repeated merely
  to consume the larger budget. It is not the old CPU-reference qualification.
- Batched polynomial stationary-root checks cover every spline interval and both
  knot sides. This is the same continuous extrema mathematics as the retained
  original checker, not a coarse sampling replacement. A homogeneous clock
  correction enforces the requested per-joint limits, rest endpoints and the
  existing 0.1% duration reserve. Defaults remain **0.6 / 1.2 / 6**.
- The exact previously generated polynomial path stays byte-for-byte unchanged;
  only its positive time density changes. Playback uses that exact law. No
  corner rounding, waypoint deletion, path refit or new internal stops.
- Solver wall-time bounds exclude initialization, seed and verification work;
  full cold calculation is reported separately. A checked incumbent is retained
  if any longer or higher-resolution attempt is invalid or slower.
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

## Historical measured balance (2026-10-24, saved inputs)

Right saved-rest → R1 → R4 → independently planned return:

- Movement, excluding unchanged holds: **10.768 → 10.427 s** (3.16% shorter).
- Fresh full-calculation median: **7.060 → 7.002 s** over three runs each.
- Current cold launch including saved-scene/session preparation: **8.239 s**.
- Left nominal movement: **2.980 → 2.857 s** (4.13% shorter); full calculation
  **6.371 → 6.824 s** in the final before/after comparison.

All geometric coefficients and limits stay identical. Doubling timing resolution
saved just 26 ms more across the right mission while adding about 1.65 s of timing
computation, so it is not the default. These are software measurements, not a
proof of global time optimality or physical arm performance. That dated study
reported 672 passing tests and both-arm native playback/checkpoint checks.
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
