# Right J2 operating-position correction — offline verification

## Delivered policy

- **Right J2 commanded/model lower bound: 0 degrees**, in the same
  `encoder_to_joint` coordinates displayed by `start_beat.sh`.
- The operator reports a mechanical stop near **-1 degree**. The difference is
  a provisional operating clearance, not a measured maximum tracking error or
  independently certified hardware limit.
- Apply the bound **after** calibration's nominal limit-coordinate conversion.
  Do not edit the shared/vendor URDF, calibrated joint origins, encoder zero,
  sign conversion, camera transform or motor firmware.
- J2 remains free above zero. Its upper bound, other joint bounds, velocities,
  accelerations, jerk limits, collision margins and left-arm behavior remain
  unchanged. This is not a J2 lock or a change to the path/timing algorithms.
- Newly generated DT1 endpoints use **J2 >= +1 degree**: one additional degree
  above the operating floor, approximately two above the reported stop. Other
  lower/upper endpoint reserves remain 3 degrees, including J2's upper reserve.
  Manual saved R1/R2 goals are not rewritten or subjected to the DT1-only reserve.

The generated model supplies bounds to standalone destination IK/approach checks,
experiment and hardware planners, continuous curve checks, native collision
validation and the hardware controller's existing model-bound validation. The
sparse fork also inherits the shared generated position bound without changing
its algorithms. A negative measured start is rejected with an explanation, not
clipped or automatically moved. Prepare/Start requires a current-policy model;
recalculate old right-arm plans. Legacy v4/v5 mission data remain readable for
their existing checked prefix-recovery workflow. No active motor owner was
accessed, modified, restarted or terminated.

## Safety and comparison boundary

**All tests were offline. No camera or encoder connection, CAN command, enable,
disable, hold, physical motion or physical calibration was performed.** Real saved
camera/encoder data were replayed; UI execution used fake motors only. These
results establish software feasibility, not physical tracking or collision safety
under a changed scene. No stored scene constitutes fresh hardware readiness.

The principal comparison uses exactly:

`planner_runs/20261028T011228Z-hardware-right-56a27e/calculate-20319199/capture`

Before/after scene, measured start, passive-arm pose, calibration, tool geometry,
zones, padding and timing limits are identical. The DT1 green dot remains
`[0.5454327821296912, -0.3779090345621325, 0.4795023706624437]` metres. Inward yaw
remains within +/-20 degrees, shaft elevation stays above parallel, and the
J7 swing-plane alignment is retained. Only the endpoint posture is re-solved.
Native model files differ in exactly one URDF attribute: right J2's lower bound.

## Route quality and movement times

| Same saved start/scene | Before | Corrected | Result |
| --- | ---: | ---: | --- |
| DT1 | 2.927047 s | **2.795804 s** | 4.48% shorter movement |
| Saved R1 | 3.684378 s | **3.340158 s** | 9.34% shorter movement |
| Saved R2 | 3.657196 s | **3.657216 s** | Essentially unchanged (+0.000021 s) |
| Left L1 control case | 3.765846 s | **3.765846 s** | Identical samples and timing coefficients |

These are outward movement durations, not calculation times or physical runs.
The old DT1 route is a diagnostic reference only: its J2 minimum was **-6.0530
degrees**, and its goal was **-3.8461 degrees**. The corrected DT1 route spans
approximately **+0.0078 to +1.1413 degrees**, ending at **+1 degree**. Polynomial
extrema, not only sampled waypoints, were used to verify this interval.

On the same DT1 task:

- Wrist path length: **265.96 -> 254.40 mm**.
- Stick-tip path length: **766.12 -> 765.95 mm**.
- Substantial joint reversals (2-degree threshold): **3 -> 2**.
- Backtracking measure: **54.68 -> 48.52 degree-sum**.
- Total joint travel: **195.39 -> 212.03 degree-sum**; some motion is redistributed
  into other joints. Proximal-weighted travel rises about 3.7%. Not every metric
  improves, and these are geometric heuristics, not an aesthetic guarantee.
- Shaft elevation changes only **8.444 -> 8.718 degrees**; inward yaw remains
  +20 degrees. The target/disc/stick model was not altered to obtain success.

R1's old route dipped to -1.7074 degrees; its new route stays above zero, with
less wrist travel and fewer reversals. R2 was already above zero and changes
only negligibly. The independently planned **start -> R1 -> R2 -> start** cycle
also passes: movement legs 3.340158, 2.007264 and 3.657211 seconds respectively,
with the original two-second checkpoint holds. No new internal stops were added.

Generation and planning from the saved latest DT1 capture take approximately
**18–19 seconds**, versus 11.30 seconds in the controlled before run. This includes
fresh perception/IK and path/timing calculations, but excludes live acquisition
and the additional independent native preparation check. The extra cost is the
existing bounded workspace recovery needed to respect J2's corrected interval;
no planning-result cache or old trajectory is substituted. Movement is slightly
faster, but calculation is slower. R1/R2 generation took about 6.38/2.89 seconds.

## Avoiding unnecessary restriction

Initially, the unchanged blanket 3-degree goal reserve was stacked above the new
0-degree operating floor. It succeeded on the latest capture (2.553430-second
route) but found no exact aligned goal on the earlier actual-camera fault capture:

`planner_runs/20261028T002734Z-hardware-right-322aca/calculate-322a821a/capture`

A bounded geometric study held the target, orientation constraints and all other
reserves fixed while testing additional J2 lower-goal reserves of 3, 2, 1, 0.5
and 0 degrees. On that earlier capture, the search found 0, 0, 2, 3 and 4 geometric
candidates respectively. This is a bounded-search result, not proof of global
impossibility at 2 or 3 degrees. Collision checks were not mocked as evidence of
feasibility: the selected **1-degree** policy was subsequently checked with the
real native collision world in both programs.

The fixed 1-degree additional reserve was the largest tested value that worked
on both captures. It is not a runtime fallback or an automatic safety-limit
relaxation. The **full-path 0-degree floor is never reduced**. The earlier capture
now produces a destination at the unchanged green dot, shaft elevation 11.560
degrees, plus a hardware-equivalent timed route of **3.433170 seconds**. Its
independent native check passes 1,718 samples plus edges. No need to relax J7,
inward aiming, collision margins, or the green target.

## Verification

- Selected regression suite: **576 passed, 5 historical whole-source hash tests
  deselected**. One SciPy warning concerns an internal trial iterate being brought
  back inside optimizer bounds; accepted goals/measurements are never clipped.
- The initial unfiltered run is preserved: 571 passed, 8 failed. One failure
  exposed the genuinely over-restrictive stacked goal reserve; two expected old
  native fixtures to remain eligible for a new Start. The former was fixed with
  the bounded study above. The latter now explicitly test legacy recovery loading
  plus rejection of new forward preparation, and a current-policy native fixture
  tests successful preparation. The remaining five are historical byte-hash
  assertions, not numerical failures: two flag the intentionally changed model,
  and three first flag earlier changes in depth/computation. They were not rewritten
  to hide source changes. See `unit_initial.log` and `unit_final.log`.
- Hardware/experiment exact output parity passed for DT1, R1, R2, R1/R2 cycle and
  left L1. Final-policy DT1 was also rerun after adding the position policy to
  the DT1 job identity; joint values and timed output remain identical.
- Independent native samples plus edges: latest DT1 **1,399**, R1 **1,672**,
  R2 **1,830**, R1/R2 full cycle **6,504**. All passed existing collision/zone
  margins; velocity/acceleration/jerk and continuous-position checks passed.
- Both standalone launcher replays pass independent native endpoint/FK/aiming
  checks. Standalone and hardware generate the **identical latest destination
  joint vector**, despite independent perception/solver execution.
- Native standalone UI: static start/goal, markers, new bound text, export,
  tamper/no-overwrite rejection and failure display pass without render errors.
- Native three-window hardware UI: actual offline Calculate/cancel/retry, fresh
  DT1 generation from the saved pair, disconnected preview, **fake-motor**
  Start/Hold/checked Reverse/settled release pass. Left preview has no DT1.
  Screenshots and joint-path curves were visually inspected.
- Calibration/FK preserved; left model URDF bytes and timed samples/coefficients
  are identical before/after. The study's initial left equality assertion included
  measured solver wall-clock diagnostics and therefore failed; inspection showed
  only `history` solve seconds and `calculation_seconds` differed. The corrected
  comparison verifies exact numerical laws and samples (`left_preservation.json`).
- Syntax/AST and `git diff --check` pass. A task-start audit covers 395 existing
  files; 110 protected engine/controller/model/recording files remain unchanged.
  The experiment engine change is only one explanatory log line. No files were
  staged, committed or deleted. Original captures and user changes are preserved.

## Evidence and use

- `quality_report.json`, `joint_paths.png`: exact extrema and quality comparison.
- `reserve_feasibility.json`: bounded geometric reserve study (not collision proof).
- `identity_latest/`, `hardware_earlier/`: final-policy current-hardware/experiment
  replays and independent native checks on both actual fault captures.
- `new_r1/`, `new_r2/`, `new_r1_r2/`: saved-goal and checkpoint tests.
- `standalone_latest/`, `standalone_earlier/`, `latest_native/`, `earlier_native/`:
  independent standalone launcher and native endpoint checks.
- `standalone_ui_final/`, `hardware_ui/`: UI evidence; controllers simulated only.
- `study.py`, `quality.py`, `study_results.json`, logs, `preservation_audit.json`
  and `before/`: comparison code/provenance. `baseline_fault/`, `old_*` and the
  intermediate `updated_fault/` are diagnostic references, not current motion.

Open the checked destination **without devices**:

```bash
./plan_cymbal_destination.sh --open diagnostics/right_j2_operating_limit_20261028/standalone_latest
```

For the next real use, relaunch the normal program and **Calculate a new
trajectory**, inspect it, then decide whether to Start. Do not reuse an old
prepared route. Physical tracking remains untested by this work; the software
margin is provisional, not permission to push against a mechanical stop.
