# 5 mm occupancy refinement — 2026-10-22

## Scope and outcome

Implemented 5 mm occupancy cells for new native Gemini scenes, replacing 10 mm.
The sensor still supplies full-FOV 1280×800 depth; all confirmed pixels are used.
This changes spatial grouping, **not** camera accuracy, FOV, temporal fusion,
Workspace curve generation/ranking, timing, robot/tool geometry, zones, motion
limits, collision policy, or motor controls.

**The original `nominal_endpoint_invalid` collision is fixed for the exact saved
incident. The complete incident mission is NOT fixed:** after passing the start
check, all 14 existing first-leg route proposals are rejected. Do not describe
this change as a successful rest → R1 → R2 → rest execution for that capture.

## Implementation

- `depth.py`: 5 mm capture/default voxel grouping; `refine_snapshot` derives a
  separate immutable 5 mm scene from the saved hash-verified native depth image.
  It never subdivides old occupied cubes, regenerates old temporal fusion, or
  rewrites original captures. Raw frames, fused/free-depth files and calibration
  remain unchanged. Derived metadata records the original scene hash.
- `planning.py` and the shared native MoveIt/experiment checker: honor scene
  resolution, accepting 5 mm and archived 10 mm. Old requests without resolution
  retain 10 mm; old plans retain their recorded interpretation.
- Search/final proof use the same recorded resolution; candidate cache identity
  explicitly includes it. New experiments on both arms and simulated hardware
  recalculations refine old inputs. Existing archived playback does not.
- Frozen mesh vertices/faces remain identical because their input depth image is
  unchanged. Live voxel-view labels now correctly say 5 mm.
- Requested world clearance stays **20 mm**, finite-edge allowance **5 mm**;
  effective robot padding stays **25 mm**. Stick perception mask remains **29 mm**.
  No new obstacle exemptions, removal of contacts, expanded masks or joint clipping.

## Exact incident

Source: `planner_runs/20261022T203104Z-hardware-right-5e5d2b/calculate-f2ec86b8`.
The source start, scene, calibration and R1/R2 selection were retained.

| Test | Original 10 mm | Reconstructed 5 mm |
| --- | --- | --- |
| Occupied centers before self masking | 120,527 | 329,428 |
| Effective world padding | 25 mm | 25 mm |
| Measured starting pose | collision | valid |
| R1 goal | valid | valid |

The saved map reconstruction took approximately 0.150 s on this machine.
The separately replayed fresh-capture path produced 329,429 cells: it
uses native scaled depth before archival float32 conversion, while reconstruction
uses the archived float32 image. This one-cell rounding-boundary count difference
does not change the start/goal result; neither input precision path was changed.
The offending old cell encloses four original, temporally supported returns.
The smaller actual occupied subcells pass with the same buffer; the native
regression fixture confirms none of those cells were self-filtered away.
An added actual goal obstacle still fails the native collision check at 5 mm.

A full hardware **Calculate** test used virtual encoder replies and replayed the
original five camera frames through the unchanged production acquisition,
monitor, isolated compute process and planner. The fresh 5 mm scene passes its
endpoint checks, then returns `no_valid_workspace_curve_within_budget`:

- Ten proposals violate continuous interpolated joint bounds. All ten dip below
  the unchanged right J2 lower limit by 0.00937–0.68573 degrees; two also exceed
  J4's upper limit. This is a proposed-curve issue, not an invalid encoder pose.
- Four proposals pass joint bounds but collide with `right_drumstick` along the
  route, about 8.5%, 9.7%, 22.7% and 23.1% through their provisional timing.
  Native single-state checks independently reproduce those later collisions.
- The existing 14 proposals finish in about 2.15 s; this is not a 120 s search
  timeout. Giving the identical finite proposal set more time does not add paths.
- 343 virtual feedback batches completed without freshness/fault errors; zero
  enable, disable, gripper or motion commands. The monitor closed cleanly.
- The first leg remains infeasible with this proposal family, so the full test
  cannot reach/check R1 → R2 and the final return. No fallback path was fabricated.

The next separate algorithm task would be to generate/refine curves with the
**continuous** joint bounds and route collisions accounted for, rather than
relaxing these limits or reducing the buffer. That is outside this map-only change.

## Verification

All verification used saved data/fake motors. **No physical camera or CAN access,
encoder query, motor command, torque change or arm movement.**

- Both native hardware-mode UIs passed saved-input one- and three-destination
  Calculate, preview play/pause, scrub, speed, fake-owner Start, normal independent
  final-checkpoint return, Hold, traveled-prefix Reverse + relax, selection
  invalidation/add/remove/exclusion, and Cancel/retry controls.
- Six fake-owner cycles completed (three per arm). All five action buttons stayed
  available. No final UI/render errors. Screenshots inspected. Both frozen camera
  scenes remained static; geometry was not repeatedly uploaded during playback.
- Dense independent native proofs at ≤2 ms plus joint-edge subdivision:

  | Saved-input test | Samples | Result |
  | --- | ---: | --- |
  | Right, one destination | 2,104 | all valid |
  | Right, three destinations + independent return | 9,192 | all valid |
  | Left, one destination | 1,421 | all valid |
  | Left, three destinations + independent return | 6,298 | all valid |

  These UI tests use the established experiment saved inputs, **not** the failed
  incident start/scene above. Left's two small additional endpoints are test-only
  files inside its run folder, not new user destinations.
- Planning workers retain the existing 3 GiB cap and serial execution. Observed
  UI calculation-group peaks were below 383 MB; UI test groups below 506 MB.
  The full incident test including native compilation peaked at ~1.35 GB. No OOM.
- Source/data hash audit guards all untouched generators, timing modules,
  controls, models, zones, recordings, destinations and original incident files.
  Historical audit tests explicitly allow only this resolution-wiring change;
  new tests preserve unchanged function bodies within edited modules as well.

**870 automated tests passed** (one existing SLSQP trial-bound warning). No
measured robot angles were clipped. Results and warmed native map timing are
recorded in `verification.json`, `warm_report.json` and `regression.log`. Physical tracking and
clearance remain untested; software collision results are not physical guarantees.

## Reproduce / evidence

Source ROS Jazzy and `install/setup.bash`; use the planner venv, one BLAS/OMP
thread and `PYTHONPATH="$PWD:$PYTHONPATH"`.

- `run_tests.sh`: regression suite, including `tests/test_depth_resolution.py`.
- `verify_incident.py NEW_FOLDER`: exact archived start/goal comparison.
- `verify_calculation.py NEW_FOLDER`: full saved-input Calculate reproduction;
  currently expected to reject all first-leg proposals as documented above.
  Run via `run_bounded.py` for the existing 3 GiB/no-CAN containment.
- `incident_case.json`: small original-return/native geometry regression fixture.
- `diagnose_remaining.py`, `remaining/report.json`: exact subsequent bounds and
  collision diagnostics; no production planner changes.
- `tests/hardware_workspace_ui_check.py`: both-arm native UI/fake-owner workflow.
- `benchmark_scene.py`, `warm_report.json`: warmed native load + two endpoint
  checks only; **not** a trajectory generation speed benchmark.

Original captures and large derived scenes/plans stay local and unchanged.
Only small fixtures, reports and reproducibility/audit sources are curated in Git.

Warm native scene load + two endpoint checks (three trials, median):
10 mm **0.772 s**, 5 mm **0.874 s**.
This excludes compilation and is not a complete planner-speed comparison.
