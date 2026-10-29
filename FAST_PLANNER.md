# Separate wrist-biased `--fast` planner

```bash
./plan_drums.sh --fast --right
./plan_drums.sh --fast --left
```

This is a **software-only planning/preview mode**, not an arm executor. It does
not replace or modify `experiment`, `--hardware`, or `--hardware-sparse`.
Normal launch acquires one fresh camera scene and reads both arm poses using the
existing query-only acquisition, then closes the devices. There are no motor
enable, disable, hold, gripper, movement or relax controls in this mode.

Select one to three destinations, then click **Calculate fresh**. **Play** only
animates the model. Changing a selection invalidates the result and cancels an
active calculation; it does not queue another calculation. Cancel and retry are
available. Closing the preview reaps its disposable computation worker.

## What changes

- Destinations define the modeled **stick-tip position and orientation**, not an
  immutable seven-joint pose. Destination files themselves are never changed.
- Default **full-pose** mode preserves the complete stick pose. Optional
  **tip-axis** mode preserves the tip and stick direction but permits roll around
  the stick. That can change gripper orientation; full robot/tool collisions and
  the original gripper-reference zone are still checked. This is not an option
  to change the contact point or freely tilt the stick.
- Coupled inverse kinematics proposes joint configurations requiring less J1–J4
  movement between destinations. Combined proximal endpoint travel cannot exceed
  the captured-posture reference. No recording interiors seed the search.
- Minimum-joint-motion curves are tested first. When obstructed, the unchanged
  Workspace curve generator supplies bounded clearance alternatives. If those
  also fail, a bounded native joint-corridor search supplies new proposals;
  their continuous quintic/C3 curves are independently checked. C3 blends retain
  the established 1-degree/1-cm bounds on deviation from the searched polyline.
  A blocked route never permits weaker margins or a stop-at-every-corner fallback.
- Actual checked movement time ranks candidates. Among candidates within **5%**
  of the fastest checked transfer, prefer the least total J1–J4 path travel. This
  explicitly bounded compromise avoids sacrificing large amounts of speed to a
  geometric score. Other candidates remain fallbacks if approach/return fails.
- With two or three destinations, the optimized first playing posture is reached
  by a separately checked approach **from the captured pose**. The final return
  is independently planned. Transfer, approach and return times are reported
  separately. There is no teleportation to a convenient simulated start.

## What stays unchanged

Original joint-position limits, calibrated model, per-side tools, passive-arm
geometry, zones, collision policy and margins. Motion limits remain **0.6 rad/s,
1.2 rad/s², 6 rad/s³** for every joint. There is no installation of the higher
limits explored in earlier studies.

The existing sparse timing mathematics is reused, with a separate process-local
reference-quality budget (96 controls, 600 iterations, 8 s per sparse stage).
Comparison paths receive exactly the same timing algorithm and budget. Normal
experiment/hardware timing settings are unchanged. All outputs retain continuous
derivative checks and final dense native collision/zone checks. The optimizer is
not claimed globally optimal. Solver-stop/fallback messages remain visible.

Two-second preview holds and rest-to-rest legs remain. This mode does **not** yet
implement impacts, rebound, continuous beat execution or a physical controller.
Collision checks refer to one frozen observed scene; unseen space is not certified
clear. Joint travel is a heuristic, not a torque/inertia or thermal model.

## Offline tests and comparisons

Explicit saved state **and** scene are required; stale experiment defaults are
not silently substituted. A normal launch without them uses fresh acquisition.

```bash
./plan_drums.sh --fast --right --offline \
  --state PATH_TO_SAVED_MEASURED_START.json --scene PATH_TO_SAVED_SCENE.json \
  --destinations R1 R2 --headless --compare --output NEW_OUTPUT_DIRECTORY

./plan_drums.sh --fast --left --offline \
  --state PATH_TO_SAVED_MEASURED_START.json --scene PATH_TO_SAVED_SCENE.json \
  --destinations L1 --headless --compare --output ANOTHER_NEW_DIRECTORY
```

Names (`R1`, `R2`, `L1`), full destination stems, and explicit JSON paths are
accepted. Side/schema/model/joint-limit validation still applies. `--task tip-axis`
enables roll freedom explicitly. `--compare` additionally calculates the original
captured-posture Workspace curve; it increases calculation time and is not needed
for normal use. `--seconds N` closes only a simulation preview; `--execute` is not
supported.

Results are under the chosen session's `plans/`: `plan.json`, `fast_report.json`,
per-candidate routes, private destination snapshots, exact timing laws and native
checks. No trajectory-result cache is used. One contained process group runs at
a time, with the existing 3 GiB memory cap and no AF_CAN during planning/preview.

The study and numerical findings are recorded in
`diagnostics/fast_posture_20261024/REPORT.md`.
