# Exact-output planner performance pass (2026-10-21)

Software only: saved scene/poses, no camera acquisition or CAN/motor commands.
Production path generation/ranking, motion limits, solver policy, refinement
levels and collision/zone validation were not relaxed or replaced.

## Implemented

* Bounded persistent artifact mirror for saved-input experiments, shared across
  launches. Exact request/model/code/environment keys, checksummed files, atomic
  publication, nonblocking interprocess locking and 512 MiB / 128-entry LRU cap.
  Run-local evidence remains intact. Missing/damaged/busy storage is a miss.
  Live Single arc acquisitions/readiness never use this experiment mirror.
* Delay analytic Jacobian construction until SLSQP requests it. Rejected
  line-search trial points no longer allocate/calculate unused dense Jacobians.
* Construct invariant spline derivatives once per extrema check. Use the same
  NumPy affine-Horner polynomial operations without repeated wrapper objects.
* Retain the old numerical implementation and its SHA-256 audit in `reference/`
  and `before_hashes.json`. Inputs include independent identical model copies.

## Quality and performance evidence

All four full-policy, 192-control comparisons have identical serialized
numerical output, including coefficients, extrema, optimizer iterations and
refinement decisions. Only elapsed-time measurements differ. Both versions
also passed the native dense collision/zone checker and derivative validation.

| Frozen case | Original (profiled) | Optimized (profiled) | Reduction |
| --- | ---: | ---: | ---: |
| Right clearance | 200.743 s | 181.838 s | 9.4% |
| Right single arc | 78.624 s | 63.059 s | 19.8% |
| Left single arc | 92.308 s | 77.635 s | 15.9% |
| Left clearance | 49.291 s | 35.814 s | 27.3% |

**Profiling adds overhead**, especially to Python loops; do not interpret those
times as normal GUI latency. A separate unprofiled left-clearance run took
23.645 s originally versus 18.077 s optimized (23.6% less). Two and four numeric
threads took 18.323 s and 18.123 s, with the same result: no meaningful advantage,
so the existing single-thread worker setting remains. These are individual
measurements, not a universal speedup guarantee.

The right experiment's first-session calculation stages totaled 264.023 s;
the identical inputs in a fresh second session totaled 0.360 s. Left stages
totaled 125.217 s versus 0.619 s. These exclude process startup, rendering and
human interaction. Shared pools/timing still operate within the cold session
(left clearance already shares sweep's selected curve).

Sixteen native Qt/VTK method playback cases passed (four methods, both arms,
two fresh launches). Tests exercised unselected startup, invalid/empty
destinations, rapid destination replacement, cancellation/retry, play/pause,
seek, speed, looping, exact return, camera-mesh controls and screenshots.
Cold/warm paths, timing laws, outward samples and collision proofs match
exactly. Maximum observed contained worker memory was 344.8 MiB; the existing
3 GiB / 128 MiB swap limits and serial queue remain unchanged.

Regression suite: **474 passed**, zero failures. The direct old/new rounded-curve
comparison emits one known SLSQP trial time-density bound warning; this is not
robot-joint clipping, and both versions return identical validated laws. The
production v2 wrapper already classifies this diagnostic. See `regression.log`.

`verification.json` is the machine-readable comparison. Raw profiles, laws,
native checker results, UI screenshots and containment records are under
`planner_runs/performance-*`; these bulk runtime files are not source artifacts.

## Reproduce

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv-planner-experiment/bin/python \
  diagnostics/planner_performance_20261021/compare.py --version current \
  --cases right_clearance,right_single_arc,left_single_arc,left_clearance \
  --output planner_runs/my-performance-check
```

Use `--version reference` for the retained engine and `--profile` for cProfile.
Run heavy checks serially within the existing resource limits. Ordinary UI
commands are unchanged; cross-session cache reuse is automatic after selecting
an identical destination. `X5_PLANNER_CACHE_DIR=off` disables just that reuse.
