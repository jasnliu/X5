# Generated DT1 goal joint-position reserve

## Scope and result

Implemented only the requested generated-goal correction: **3 degrees of reserve
inside both calibrated position bounds of every right-arm joint**. The shared
cymbal solver applies it to `plan_cymbal_destination.sh`, normal hardware DT1, and
experiment DT1. It is an endpoint constraint, not a change to motor/model limits,
timing, route-generation algorithms, measured starts, or recovery.

All work and verification was **offline**. Saved real camera data/encoder poses
were replayed. No physical camera/CAN connection, enable/disable, hold, release
or arm movement was performed. The UI execution test used simulated motors only.

## Fault-capture comparison

Source: `planner_runs/20261028T002734Z-hardware-right-322aca/calculate-322a821a/`.
The old endpoint placed J7 at its upper bound and J2 at its calibrated lower
bound. The regression test reproduced these zero-clearance candidates before
the change (`regression_before.log`), then passed on the new solver.

| Quantity | Previous goal/plan | Reserved goal/plan |
| --- | ---: | ---: |
| J7 goal | 1.400000000 rad | 1.347640122 rad |
| J7 upper-bound clearance | 0 degrees | 3 degrees |
| J2 goal | -0.119486222 rad | -0.067126344 rad |
| J2 lower-bound clearance | 0 degrees | 3 degrees |
| Minimum endpoint clearance, all seven joints | 0 degrees | 3 degrees |
| Shaft-above elevation | 9.673584 degrees | 10.825483 degrees |
| Inward yaw | +20 degrees | +20 degrees |
| Planned outward duration | 2.888025 s | 3.410966 s |

Both programs independently generated the **same new joint vector** at the
unaltered green point `[0.5408726891409983, -0.378703976028645,
0.48438769130806975]`. The disc normal, inward aiming, J7 swing-plane constraint,
calibration, model and original motion limits are unchanged. Native tip error was
approximately `1.74e-14 m` (numerical model agreement, not physical accuracy).

The new hardware calculation passed native validation of **1,707 exact-curve
samples plus edges**, including arm/stick, zone and original collision padding.
Its numerical timing and samples matched an independently recalculated experiment
plan exactly. This test remained explicitly offline and was rejected as physical
hardware eligibility. Hardware calculation from saved inputs, including detection
and goal generation, took 11.82 s; acquisition time is not included.

The extra 0.523 s movement duration is a consequence of the new endpoint and the
existing path selector; timing/search policy was not retuned. The unchanged
moderate workspace proposals now found a route, whereas the old goal needed the
existing recovery proposals. This task does not optimize that tradeoff further.

`fault_input_comparison.json` verifies that the full hardware request differs
only in the goal and generated model file paths; the model bytes are identical.
Measured both-arm poses, original camera self-mask, five-frame scene, zone and
collision policy/padding are equal. In particular, starting J4 is still only
about 0.248 degrees above its original lower bound: it was not clipped or rejected
by the new endpoint rule.

A second actual saved pair from `cymbal_destination_runs/20261027T054238Z-923ea3`
also generated a valid reserved goal: 8.972355 degrees shaft elevation, +20 degrees
inward yaw, minimum joint reserve 3 degrees. Independent native FK/collision
checks passed (`previous_native/report.json`).

## Implementation contract

- `cymbal_destination/goal_limits.py` defines the endpoint policy. Calibrated
  bounds are narrowed only for IK fitting and both polishing steps. Only
  optimizer guesses are projected into that interval, never measured starts or
  accepted endpoints. Acceptance uses a `1e-10 rad` numerical roundoff tolerance,
  not an encoder/controller tolerance.
- Candidate filtering, independent final checking and destination serialization
  require the reserve. No clamping, fallback to a boundary goal, reserve reduction,
  green-target movement, orientation relaxation or model-limit widening.
- Version 5 outputs contain the reserve policy, calibrated bounds and actual
  per-joint clearances. Export, reopen, DT1 job ingestion and new hardware-goal
  validation recheck them. No false ready result/destination is saved on a
  reserve rejection. Logs and standalone preview label the endpoint-only rule.
- New Calculate operations always regenerate DT1. Old v4 standalone previews
  need recalculation into a new directory. Already-created v4 hardware missions
  remain readable under their original checks so this update does not disable
  their recovery. It does not rewrite an existing plan or motor-owner session.
- Manual saved goals, left-arm planning, sparse hardware, path/timing engines,
  controller/transport, Hold and Reverse are unchanged. The experiment DT1
  selection inherits the reserve through its already-shared solver.

Three degrees is a design reserve, not a measured upper bound on tracking error.
It exceeds the roughly 0.417-degree J7 over-bound excursion seen in retained
post-fault logs, but those logs did not retain the initial fault sample. It does
**not** resolve or certify the separate J2 tracking discrepancy, recovery from
out-of-bounds feedback, arbitrary overshoot, dynamics, or physical operation.

## Verification

- **557 selected tests passed**, 2 historical hash tests deselected, in 64.08 s
  (`regression_final_job/worker.log`). Covers both sides, per-joint edge rejection,
  nonfinite data, seed-versus-measured-pose separation, calibrated proof integrity,
  invalid-goal/no-save behavior, stale policy, DT1 integration, prepared packets
  for v5 and legacy v4, and existing timing/recovery/sparse behaviors.
- The unfiltered run had **553 passed / 2 failed** before four additional reserve
  tests were added. Both failures are old whole-source SHA snapshots:
  `test_original_sources_only_have_explicit_opt_in_wiring_changes` first flags
  `drum_planner/depth.py`; `test_original_hardware_fast_mode_and_path_generator_preserved`
  first flags `baseline/computation.py`. Both files matched this task's initial
  hashes exactly. Those historical assertions were not rewritten to force a pass.
- The only warning was SciPy clipping an internal trial iterate in a synthetic
  narrow-bounds test. Accepted endpoint values are never clipped.
- Two complete standalone launcher replays recomputed perception and IK; two
  independently initialized native endpoint checks passed. No target override.
- Native standalone Qt/VTK controls passed, including reserve text, exact target
  markers, pose switching, export/no-overwrite/tamper rejection, failure display
  and no render errors. Screenshots were visually inspected.
- Native three-window hardware UI passed offline Calculate/cancel/retry, DT1
  regeneration, profile invalidation, independent preview, and **fake-motor**
  Start/Hold/checked Reverse/settled release. Left preview has no DT1 option.
  Existing physical owners were not accessed or interrupted.
- AST, launcher shell syntax and `git diff --check` passed. The task-start audit
  checked 362 preexisting source/config/model/test files; 64 explicitly protected
  engine/controller/model/recording/launcher files were unchanged. Previous raw
  captures/results and user changes were preserved; nothing was staged or committed.

## Evidence and replay

- `standalone_fault/`, `standalone_previous/`: new standalone results.
- `standalone_native/report.json`, `previous_native/report.json`: independent checks.
- `hardware_fault/report.json`, `hardware_fault/hardware/`: current hardware
  calculation on the exact fault capture; experiment parity alongside it.
- `standalone_ui/`, `hardware_ui/`: UI reports/screenshots and simulated run.
- `fault_input_comparison.json`, `preservation_audit.json`, `before/` and job logs.

Open the checked static destination without devices:

```bash
./plan_cymbal_destination.sh --open diagnostics/cymbal_goal_reserve_20261027/standalone_fault
```

Regenerate from the same actual saved pair (choose a new output directory):

```bash
./plan_cymbal_destination.sh --headless \
  --replay planner_runs/20261028T002734Z-hardware-right-322aca/calculate-322a821a/capture/camera/pair.npz \
  --state planner_runs/20261028T002734Z-hardware-right-322aca/calculate-322a821a/capture/measured_start.json \
  --output /path/to/new-offline-run
```

For ordinary subsequent use, the existing launch commands are unchanged. The
new policy applies to a **freshly generated destination/calculation**, not a
previously prepared route.
