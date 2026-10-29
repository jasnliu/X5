# Hardware FCL startup failure: measured-rest correction

## Scope and result

**Software-only. No camera or encoder acquisition, CAN access, enable/disable,
real HOLD, or physical arm motion was performed.** Saved real camera captures
were replayed; motor tests used `SimulatedBus`. Normal `--hardware` now handles
the actual failing resting pose, with original collision/zone, V/A/jerk and drive
policies retained. Test outputs are NOT fresh physical readiness evidence.

## Confirmed cause

Both captures below contain right J2 **-0.005562371252002762 rad (-0.3187004°)**:

- `planner_runs/20261028T023205Z-hardware-right-98a266/calculate-5b1b95c4/capture`
- `planner_runs/20261028T023247Z-hardware-right-ae273b/calculate-7f013e4d/capture`

Their native `plan/result/dt1/checkpoint_1/oracle.log` reports **Saved start violates
calibrated joint limits**. The previous generated model's lower bound was 0°.
Thus the native world rejected the acquired pose before IK/path generation.
The mistake was treating the provisional interior planning margin as a physical
limit on actual resting feedback. This was not a camera/FCL-library failure.
Original evidence is retained in [original_failure.txt](original_failure.txt).

## Correction and safety distinctions

1. Keep 0° as the ordinary J2 operating floor and **DT1 goals >= +1°**. Other
   generated-goal reserves remain 3°. No target, encoder pose or trajectory is clipped.
2. Bind a new model/request to the exact original acquired full pose. If its J2
   is strictly between the reported -1° stop and 0°, permit only a continuous
   monotone departure from that rest or return to that same full pose below zero.
   No lower excursion, unrelated negative goal or negative intermediate dip is
   accepted. Roots/extrema/signs of the actual polynomial, not just waypoints,
   establish this rule. The acquired camera self-mask remains unchanged on all legs.
3. Native comparisons are exact: one legitimate R2 curve evaluated its start
   2.6e-18 rad below the stored scalar. The negative-rest model therefore carries
   an explicit **1e-12 rad (5.7e-11°) arithmetic allowance**, never pose snapping
   or a physical tracking tolerance. It cannot cross the reported stop.
4. Feedback uses the provisional reported -1° boundary separately from the
   command margin. Existing tracking, freshness, fault and stationary-HOLD checks
   are unchanged. A small measured HOLD offset below the original rest is checked
   using a private recovery model for the connector plus traveled prefix, under
   the existing TRACK_L1 allowance. The original model/plan is never rewritten;
   the connector cannot go below its measured held endpoint (except roundoff).
5. New Prepare/Start verifies the current reference-bound policy. Old v4/v5/v6
   destinations remain readable for recovery, not fresh Starts. Standalone DT1
   and hardware DT1 share v7 goal generation with the same +1° minimum.
6. Oracle startup errors now identify the joint, value and bounds in degrees.
   Native exits include their diagnostic tail and absolute log path, and failed
   constructors close their subprocess and pipes.

The -1° stop is operator-reported and approximate, not independently calibrated
or certified here. These tests do not establish physical tracking performance.

## Saved-camera native matrix

Every case passed native exact-curve 2 ms samples and connecting edges with the
original full arm/stick geometry, zones, occupied-volume and clearance policies.
Each hardware result was also compared to a fresh experiment calculation using
the same scene/start/goal. Final-run source geometry and timed samples matched
in all eight cases. Multi-checkpoint timing comparisons explicitly allow bounded
wall-time solver iterations to differ, but independently check both clocks;
this option does not change production timing or weaken path checks.

| Case | Outward seconds | Calculate wall seconds | Hardware proof samples | Result |
|---|---:|---:|---:|---|
| latest_dt1 | 2.743720 | 19.04 | 1,373 | pass |
| earlier_dt1 | 2.751148 | 19.05 | 1,377 | pass |
| latest_r1 | 3.385367 | 6.66 | 1,694 | pass |
| latest_r2 | 3.499214 | 3.95 | 1,751 | pass |
| latest_r1_r2 | 7.042351 | 18.12 | 6,272 | pass |
| latest_dt1_r2 | 7.005330 | 26.83 | 6,254 | pass |
| positive_dt1 | 2.795804 | 18.75 | 1,399 | pass |
| left_l1 | 3.764860 | 7.32 | 1,884 | pass |

For checkpoint cases, outward includes the intermediate two-second hold;
proof samples cover the complete mission including the independently planned
return. Calculation time includes DT1 perception/posture search where applicable,
not camera acquisition. Nothing is played from a plan-result cache.

See [verified/matrix_status.json](verified/matrix_status.json) and each case's
`report.json` for the full inputs, limits, native proof and parity results.
All offline results were explicitly rejected as physical hardware eligibility.

## Workflow verification

- **693 functional pytest checks passed**, 6 historical whole-source-hash tests
  deselected. Those snapshots predate authorized planner/model changes and still
  expect historical bytes; their hashes were not rewritten to manufacture passes.
  See [regression_confirmed.log](regression_confirmed.log). One SciPy warning
  concerns optimizer trial variables, not clipping captured joints or final goals.
- Real native three-window UI: dt1 selection, timing effort edits, Calculate,
  cancellation/retry, fresh computation from the saved pair, disconnected preview,
  seek/play/pause, explicit fake-motor Start, active-profile edit refusal, HOLD,
  checked-prefix Reverse and return/relax; left preview has no dt1 entry.
  [UI receipt](ui_verified/ui_report.json), [preview](ui_verified/right_prime.png),
  [controls](ui_verified/controls.png). Screenshots were visually inspected.
- Actual negative-start DT1/R1/R2/checkpoint paths completed simulated cycles at
  0.5x and 1x. Native prepared-packet checks used explicitly MOCKED freshness in
  temporary test files; they did not claim a new acquisition. Early and late HOLD
  tests injected a -0.02° J2 feedback offset, checked the native connector/prefix,
  and reached the original rest without faults or premature relaxation.
- New tests reject at/below-stop starts, altered reference context, unrelated
  negative endpoints, nonmonotone excursions and old prepared Starts. Native
  startup errors and process cleanup are tested. Existing fault/retry, passive
  sway, bus ownership, drive-cap and per-arm tests remain passing.
- Standalone destination rebuilt from the same saved pair and measured negative
  rest, then independently verified tip/orientation/goal reserve and native
  collision checks: [standalone proof](standalone_native/report.json).
- CLI smoke tests use the actual `./plan_drums.sh --hardware --right/--left`
  entrypoint with **explicit `--simulate`** and a two-second auto-close timer.
  See `cli_right.log` and `cli_left.log` (no Start or devices).

Two verification-harness assumptions were corrected rather than changing motor
behavior: compare a polynomial endpoint with numerical precision instead of
requiring snapped encoder bytes, and read the owner's final shutdown receipt
rather than its potentially stale periodic status snapshot.

## Preservation

[Source/data audit](preservation_audit.json): 441 prior files checked; no files
removed; all 108 protected files unchanged, including vendor/shared URDF,
calibration, recordings, saved destinations, native collision code, timing
solvers, physical transport/controller and prefix-recovery law.
[Numerical regression](numerical_preservation.json): the previously working
positive-start DT1 route remains **2.795804073069683 s**, with identical source
curve and timed samples; left generated URDF bytes are identical to before.
The added gates validate geometry; they do not replace the path objective or
change the timing optimizer, motor gains, drive cap or physical calibration.

## Next use

For a new run, reopen `./plan_drums.sh --hardware --right`, select the destination
and Calculate again. Do not reuse an old prepared packet. Physical Start remains
an explicit user action; these offline checks are not permission or proof of a
fault-free physical run. If an older owner is already holding an arm, preserve
that session and its existing Hold/Reverse recovery controls.
