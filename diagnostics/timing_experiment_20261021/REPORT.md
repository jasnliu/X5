# Fixed-path timing comparison — 2026-10-21 UTC

## Scope and selected policy

Software only, arms off. Saved depth scene and saved starting poses; endpoint
poses remain right `recordings/record3.json` and left
`left_recordings/record1.json`. No recording interior was supplied to a planner.
The four benchmark inputs were frozen **before** integrating any retimer.

Winner: **Adaptive fixed-path retiming**, now automatic for all four existing
path choices in both `./plan_drums.sh experiment --right` and `--left`.
The path generators, candidate ranking, original curve coefficients, vertices,
robot models, collision margins and zone rules were not changed by this work.
Baseline hardware timing and recording playback are unchanged.

## Results

Seconds **outward only**, not computation time or whole-cycle time. Selection
uses the unweighted mean across all four cases. Every cell uses the same
0.6 rad/s, 1.2 rad/s² and 6 rad/s³ limits, rest endpoints and existing 1.001
duration reserve. The full cycle is `2 × outward + 2 seconds`.

| Timing method | Right Single arc | Right Clearance | Left Single arc | Left Clearance | Mean |
|---|---:|---:|---:|---:|---:|
| Original global scaling | 7.775 | 7.793 | 19.876 | 98.102 | 33.387 |
| Local quintic | 7.775 | 7.793 | 6.090 | 10.915 | 8.143 |
| Local jerk-limited S-curve | 7.775 | 7.793 | 5.174 | 9.051 | 7.448 |
| **Adaptive fixed-path** | **4.128** | **4.230** | **5.174** | **9.051** | **5.646** |

The winner reduces the mean by **83.09%**. On the right its reduction is about
46–47%; the much larger left improvement also removes unnecessary stops at
exactly collinear intermediate vertices. Those vertices are **still traversed**;
none are removed and no actual corner is rounded. The average is intentionally
not reweighted to reduce the influence of the originally slow left cases.

Local quintic and local S-curve only replace clocks on straight sections. For
the curved right paths they deliberately retain the old continuous law rather
than turn the curves into chords or introduce new stops. Hence the identical
right entries in their rows. The adaptive policy automatically uses an analytic
S-curve on the straight cases and time-density optimization on curved cases;
there is no manual per-case method selection.

## Tuning

- Original global scaling is already the smallest uniform multiplier for its
  fixed normalized curve and the agreed limits, apart from the shared reserve.
- Local quintic uses analytic polynomial derivative maxima and the shortest
  allowed duration in that rest-to-rest quintic family.
- S-curve uses the closed-form time-optimal scalar rest-to-rest solution for
  each straight block. All joints share its phase, preserving joint-space
  geometry. Triangular jerk, acceleration plateau and velocity cruise cases
  are handled, with true corners at rest.
- Both local policies were tested with/without passing exact-collinear
  intermediate points. The faster path-identical variant was retained.
- Adaptive curved timing tried 8, 12, 20, 32, 48, 64, 96, 128 and 192 cubic
  density controls, `ftol=1e-11`, up to 1000 SLSQP iterations and up to three
  between-grid constraint refinements per level. The actual solvers converged
  successfully in 3–28 iterations per attempt.

| Density controls | Right Single arc | Right Clearance |
|---:|---:|---:|
| 8 | 4.556624 | 4.624351 |
| 12 | 4.431778 | 4.416001 |
| 20 | 4.234589 | 4.354542 |
| 32 | 4.186628 | 4.297394 |
| 48 | 4.158014 | 4.263412 |
| 64 | 4.143715 | 4.244446 |
| 96 | 4.133315 | 4.237531 |
| 128 | 4.130629 | 4.232088 |
| 192 | 4.127995 | 4.229552 |

The last, substantially more expensive refinement gained only **0.0638%** and
**0.0599%**. This is a practical plateau, not a proof that no other nonlinear
retiming formulation could improve these curves. The final 192-control result
is retained rather than trading away that small improvement. All implementations
remain available in `drum_planner/timing/`; only the winner is wired into the UI.

## Verification completed

- **820 regression tests passed in 246.35 seconds**, including timing math,
  both arm modes, tool models, experiment behavior, baseline/hardware mocks,
  playback, rendering, calibration and artifact retention. See `regression.log`.
  Thirty-three of these tests directly cover the new timing implementation.
- All **16 benchmark cells** passed independent dense derivative checks and
  actual MoveIt/FCL joint-limit, zone and collision checks: **109,943** checked
  states. Derivative extrema are also checked between grid samples using
  polynomial stationary roots and both sides of knots.
- Curved paths keep the original polynomial coefficients bit-identical; the
  scalar clock is strictly increasing. Straight cases traverse every original
  segment and waypoint in order (geometry tolerance 1e-11 rad, collinear
  grouping tolerance 1e-13 rad). Start/end are at rest. No corner rounding.
- Fresh real CLI runs generated **all eight** integrated plans, four for each
  arm. The four benchmark cases selected the same original candidates and
  reproduced exactly the frozen source curve coefficients and vertices.
- Protected source hash audit passed for geometric planning/ranking, models,
  both goal recordings and the original URDF; see `integration_audit.json`.
- The contained benchmark worker peaked at **207.1 MiB**, below the 3 GiB cap;
  swap cap 128 MiB, AF_CAN excluded. See `benchmark_containment.json`.
- Both native Qt/VTK interfaces passed full-cycle animation for all four
  methods, play/pause, seek, speed, loop, layers/colors, camera-view navigation,
  save image, settings accept/cancel, real worker cancel/resume, budget/cache
  invalidation and reranking. The scene mesh remains resident across playback
  and method switches, with all 20,184 triangles retained. No renderer errors.
  Recapture success/failure recovery was tested with an explicit offline file
  fixture (the intentional injected failure in the logs is not a camera fault).
- Integrated planning/UI test workers peaked at **367.2 MiB**; no OOM or
  unexpected worker failures occurred. See `integration_resources.json`.
- All sixteen saved laws were reloaded from their curated files and revalidated
  against the curated, hash-identical models. Serialized outward/hold/reverse
  timestamps are strictly increasing. See `saved_laws_verification.json`.
- No motor commands, CAN queries or live camera acquisition were performed.

UI evidence is saved in `ui_right/` and `ui_left/`, including screenshots,
feature-level JSON reports and terminal logs.
Software checks do not establish physical motor tracking, torque/load limits or
real-world performance at the new durations.

## Retained evidence and reproduction

- `inputs.json`: hashes of four untouched input records, including original
  coefficients, path vertices, model metadata and backend requests.
- `inputs/models/`: byte-identical curated URDF/SRDF copies; benchmark resolves
  these rather than depending on disposable historical model directories.
- `report.json`: full precision table, proofs and references to all saved laws.
- `laws/<case>/<method>.json`: every tested final timing law, including complete
  tuning histories where applicable.
- `benchmark.log`, `benchmark_containment.json`, `integration_audit.json`.
- Original full working run: `planner_runs/timing_benchmark_20261021/` (local
  generated output, not all copied into Git).

Input right paths came from `planner_runs/20261021T033500Z-right-experiment-verified`;
left paths came from `planner_runs/20261021T041425Z-experiment-8c2f9b`. Only their
compact frozen inputs/results are curated to avoid another oversized push.
See `drum_planner/timing/README.md` for exact commands, mathematics and background
sources. No additional dependencies or GPU solver were needed.
