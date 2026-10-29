# Normal hardware promotion and fresh DT1 — 2026-10-26

## Delivered

Normal `./plan_drums.sh --hardware --right` and `--left` now call the **same
current numerical path/timing engine** as normal experiment mode. This is not a
second implementation of the solver, nor promotion of a saved experiment plan.

- Broad Workspace curve proposals plus bounded-spline/wrist-guided recovery.
- Fast / Balanced / Thorough fixed-path **CPU** timing; Balanced default.
- Editable seven-joint velocity, acceleration and jerk constraints, load/save,
  Original defaults. Defaults remain 0.6 rad/s, 1.2 rad/s², 6 rad/s³.
- Right-only **dt1** selection, including mixed saved/DT1 checkpoints. Calculate
  generates the endpoint from its **new** paired RGB/depth capture, then plans and
  independently checks the exact route. No old generated destination fallback.
- DT1 keeps its existing ±20° inward aiming, near/above-parallel shaft and J7
  strike-plane constraints. No strike is generated or commanded.
- Hardware uses measured physical start, not the experiment's simulated teleport.
- Selection/profile edits invalidate but never calculate or move. Active runs
  retain their immutable profile/selection. Start alone authorizes execution.
- Three windows, two-second holds, independently planned multi-checkpoint return,
  selected-arm ownership, Hold, checked traveled-prefix recovery and settle/release
  remain. Default recovery retains its original conservative derivative bound;
  lower custom limits are respected, higher ones never speed up recovery.
- Existing **0.8 rad/s CSP drive cap is unchanged**. Actual trajectory peaks times
  next-Start scale must fit that cap before physical transport construction.
  Planning profiles do not change firmware or certify actual motor capabilities.
- Separate `--hardware-sparse` retains its historical planner/timing/acquisition
  and fixed original limits, without DT1 or these new controls.

## Architecture / scope

`baseline/current.py` calls `experiment.fast.plan` on Calculate's measured inputs.
`baseline/settings.py` serializes separate hash-bound hardware profiles.
`baseline/targets.py` freezes saved selections, resolves DT1 on Calculate's exact
snapshot, and validates its scene/calibration/state provenance. `planning_ui.py`
adds controls without automatic calculation or extra motion confirmation steps.

Right Calculate reuses the existing five-frame paired acquisition adapter; left
and sparse retain depth-only capture. Startup's display-only snapshot remains
separate: **DT1 always uses Calculate's snapshot**, not that earlier display.
The query-only observer monitors throughout the calculation, while heavy numerical
work runs in the planner venv in a separate process. Existing 3 GiB group,
cancellation/reaping, and passive-arm query-only transport remain.

Hardware acceptance now understands the paired scene explicitly. Full payload
hashes, calibration, original measured mask, endpoint/profile hashes, actual
continuous extrema and native collision/zone checks are retained. A camera-only
recapture or ordinary offline experiment does not become hardware eligibility.
`tasks.prepare/check_packet` still gates the exact playback law independently.

`experiment/rgbd.py` only factors its renderer for hardware reuse and updates its
docstring; its capture/fusion policy and all numerical planners remain unchanged.

## Verification performed — NO physical devices or movement

All geometry studies used the **saved actual-camera capture**:
`diagnostics/experiment_dt1_20261026/fresh_right/acquisition/`.
No new camera acquisition, live encoder query, enable/disable, hold or motor motion
was performed in this task. Fake camera/encoder tests are explicitly labelled.
Native UI Start/Hold/Reverse used only the separate simulated-motor owner.

### Native offline routes and parity

Each test generated the route from scratch and independently ran the hardware
native checker at 2 ms plus checked edges. It then reran the experiment engine
with identical scene/start/goal/profile. **Exact outward samples and durations
matched for all cases; checkpoint cycle samples matched as well.** No stored
trajectory was reused for either calculation. The reference reuses the resolved
endpoint (not its route); this is an input-controlled parity test, not a cold-start
speed benchmark.

| Case | Outward travel | Full cycle including holds | Native hardware-check samples | Result |
|---|---:|---:|---:|---|
| right DT1 | 2.788254 s | 7.576507 s | 1,396 | valid, exact parity |
| right R1 | 3.777010 s | 9.554020 s | 1,890 | valid, exact parity |
| right R2 | 3.708523 s | 9.417045 s | 1,856 | valid, exact parity |
| left L1 | 3.765846 s | 9.531693 s | 1,884 | valid, exact parity |
| right R1 → DT1 | 5.151887 s moving outbound | 12.690293 s | 6,347 | valid, exact parity |

The two-checkpoint case has a two-second intermediate hold, reaches DT1 at
7.151887 s, holds two seconds there, and follows a **newly planned 3.538406 s**
return. Its camera robot self-mask remains at the acquisition pose on every leg.
Native results are under `native_dt1_v2/`, `right_r1/`, `right_r2_v2/`, `left_l1/`,
and `right_r1_dt1/`; `summary.json` indexes their reports.

Calculation-only elapsed values are also saved. For example DT1 generation plus
path/timing took 21.91 s on the saved scene. This **excludes live acquisition and
the independent hardware preparation check** and is not a promised end-to-end
Calculate latency. Timing/path algorithms were not modified to obtain these numbers.

### Tests, UI and launchers

`verified_tests.log`: **525 passed, 3 deselected**, in 95.64 s. Coverage includes
both-arm fake transport/controller workflows, fresh acquisition routing and cleanup,
profile/import/hash/limit tampering, exact law playback, stationary holds, lower-limit
recovery, frame-bracket metadata, DT1 provenance, left/sparse isolation, CLI aliases,
camera failure diagnostics, existing experiments, UI invalidation and recovery.

The native prepared-packet regression uses actual saved geometry but **mocked
freshness in a temporary fixture**. It tests prepare → packet check → owner
preload and DT1 rejection without representing an old capture as fresh hardware
evidence. Separately, a fake camera/encoder child exercises the real capture
adapter and verifies all five frame brackets plus its measured-state hash.

Three dated source-freeze assertions were excluded because they explicitly demand
that pre-promotion hardware files remain byte-identical. They are not runtime tests:
- `test_shared_generators_timing_models_and_user_data_preserved`
- `test_original_sources_only_have_explicit_opt_in_wiring_changes`
- `test_original_hardware_fast_mode_and_path_generator_preserved`

Replacement task-specific preservation audit checked **413 pre-task files**:
390 unchanged, 23 changed within the explicitly listed scope, **zero unexpected
changes**. Numerical experiment/path/timing source, native collision backend,
physical transport, standalone cymbal generator, sibling DT1/Y2 code, and recorded
model/calibration/data files in the audit are unchanged. See `preservation_audit.json`.

`native_ui_v3/ui_report.json`: passed on the real Qt/VTK desktop with saved camera
geometry and fake motors. It exercised three windows; right DT1; no auto-calculate;
Cancel/retry through real contained workers; exact-law preview/seek/play; active
profile edit refusal; simulated Start → Hold → checked-prefix Reverse → rest/relax;
and left saved-only selection/preview. Screenshots: `controls.png`, `right_prime.png`,
`left_goal.png`. No render errors. Both actual `plan_drums.sh --hardware --simulate`
launchers also exited 0 (`launcher_right.log`, `launcher_left.log`).

### Issue found during integration

R2 exposed an endpoint-clock rounding difference: its last event sample was
2.6645e-15 seconds after the law's 3.7085225931534618-second duration. The old hardware
wrapper rejected it as out of range despite the identical endpoint. The wrapper
now normalizes only floating-point ULP-sized clock roundoff; real out-of-range
queries still fail. No joint coordinate, path, timing law, collision margin or
motion limit is clipped/relaxed. A regression test protects both behaviors.

Initial test harness issues (old mock boundaries, missing fake directories and a
Qt screenshot call on an unrelated window) were corrected and rerun. The successful
native UI report is v3; earlier failed harness logs are retained, not claimed as passes.

## Use

```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh --hardware --right
# Choose dt1 (or a saved destination), click Calculate, inspect preview, then Start.
./plan_drums.sh --hardware --left
```

Optional `--timing-quality fast|balanced|thorough` and `--motion-profile PATH.json`
apply to normal hardware. GUI edits offer the same controls. Profiles are arm-bound;
experiment numerical profiles can be explicitly imported without importing any
experiment motion authority. The original hardware motor settings remain untouched.

Physical tracking, clearance in the real environment, actual loaded motor limits,
and performance at increased limits have **not** been established by these software
checks. No physical motion is claimed.
