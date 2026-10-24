# Timing and bounded left-arm corner smoothing

## Current speed-first experiment timing (2026-10-24 UTC)

The visible right/left experiment now uses `experiment/fast.py` and
`vulkan/fast.py`: 24-control GPU-native FP64 density optimization, no CPU
reference solve and no result-cache reuse. Continuous extrema/correction and
fresh dense scene validation remain mandatory. GPU-unavailable fallback is a
bounded vectorized CPU solve, prominently logged and labelled. Approximately
10% longer movement is an offline quality target, not a per-request guarantee.

The previous strict 1% adapter, all original CPU policies, hardware dispatch and
production Workspace curve generator remain unchanged. The new experiment uses
staged workspace proposals and avoids redundant ranking/world construction; it
does not claim bit-identical timing or global optimality. See
`DRUM_PLANNER_EXPERIMENTS.md` and `diagnostics/fast_gpu_20261024/REPORT.md`.
All following sections document retained historical policies unless noted.

## Historical strict-1% GPU optimizer (superseded experiment default)

The retained legacy experiment methods select `gpu_integration.py` for both arms.
`vulkan/` implements FP64 projected L-BFGS/augmented-Lagrangian timing with
on-device optimizer state, updates and parallel line search. This is a **new
optimizer**, not a claim of bit-identical SLSQP coefficients or a global optimum.
Path generation/ranking and the original left C3 curve choice are unchanged.
`--hardware` and `--hardware-sparse` keep their original CPU policies below.

Acceptance requires duration <= 1.01 times the exact-input fully refined CPU
reference, no added detected internal stops/deep slowdowns, independent continuous
extrema, exact curve identity, unchanged limits, and fresh collision/zone checks.
Failure of numerical/quality verification rejects the result. Only unavailable
GPU/build support triggers a prominently logged **CPU FALLBACK**; software Vulkan
is rejected. Analytic straight-block clocks are explicitly labelled analytic CPU.

The strict per-curve 1% guarantee requires a CPU reference qualification on a
cache miss. **Cold end-to-end calculation therefore includes the old CPU solve**,
not just the faster GPU stage. Logs/report metadata separate all phases and cache
hits. Exact CPU reference and GPU math caches are bounded; every scene is checked
again. GPU cache identity includes native/shader/Python implementation, device and
driver, exact curve, limits/policy and reference law. One GPU solve slot is shared
by the existing bounded timing children. No motor/camera authority is added.

See `DRUM_PLANNER_EXPERIMENTS.md` and
`diagnostics/gpu_timing_20261024/REPORT.txt` for setup, measured results and limits.
The historical exact-output comparisons below still apply to the retained CPU
implementations, not to the newly approved <=1% experiment timing policy.

## Exact-output checkpoint optimization (2026-10-22)

Hardware and experiment workflows now share these optimizations:

* `math_cache.py` stores only the exact timing calculation, independently of the
  camera scene. The key includes the complete original curve/clock/waypoints,
  settings, limits, code, Python/NumPy/SciPy and loaded numerical-library hashes.
  No approximate endpoint matching. A new scene still gets its own full dense
  collision/zone/derivative verification. Left C3 blend selection still checks
  geometry in that scene before reusing mathematics for the selected blend.
* Existing bounded atomic cache storage is shared (512 MiB / 128 entries).
  `X5_PLANNER_CACHE_DIR=off` disables cross-session reuse. Missing, corrupt or
  unavailable cache entries do not prevent normal calculation.
* Multiple checkpoints first generate/rank ALL paths serially, with unchanged
  budgets and candidates, then time independent rest-to-rest legs in two spawned
  processes. There are no concurrent geometric searches. Both children stay
  within the same job's 3 GiB memory / 128 MiB swap cap; cancellation kills the
  calculation group, not the separate motor owner. Single-leg behavior is unchanged.
* Reuse invariant path polynomials within each timing refinement. No changed
  summation order, sparse approximation, float32, fast-math, reduced refinement,
  solver tolerance or limits. Every density-dependent extremum is still checked.
* Within a checkpoint job, reuse validated scene metadata and identical generated
  model bytes. Native collision checks remain per-request. The left smoothing
  checker reuses its already-loaded world for final validation. Hardware final
  preparation evaluates its unchanged 2 ms timestamp grid in a vectorized batch.

Progress includes per-leg timing, math-cache hits and parallel-phase transitions.
There is no new UI selector or execution behavior. See
`diagnostics/planner_optimization_20261022/` for preserved pre-change sources,
full-policy exact-numerical comparisons and software-only performance evidence.
The older sections below describe the historical sequential implementation.

These are **device-free trajectory postprocessors**, not hardware controllers.
All five geometric path choices remain in both arm experiments; there is no
extra timing/smoothing selector. The baseline hardware workflow now shares
Workspace curve selection and the arm-aware timing policy through its own measured
acquisition and `baseline/curve.py` exact playback adapter. Existing recording
playback and `live` execution are unchanged. The pre-performance v1 implementation
is retained byte-for-byte in `diagnostics/planner_performance_20261021/reference/`.

## Retained CPU / standard hardware policy: right v1, left v2

`dispatch.py` routes right-arm plans directly to the **same v1 policy** retimer
(`integration.py`, `adaptive.py`, `core.py`, `selected.json`). Right geometry,
ranking and timing coefficients are preserved. Already continuous left paths
also retain v1. Old saved plans are not rewritten. The performance pass preserves
the algorithm and arithmetic while removing redundant derivative construction,
Polynomial wrappers and Jacobians for unused line-search trials. It does not
reduce the 192-control refinement, iteration cap, tolerances, or validation.
Saved-input experiments additionally reuse hash-verified completed results across
launches through a bounded disk cache. Live planning reuses only exact timing
mathematics across snapshots and validates the current scene independently.

New corner-stop left paths use **overlapping C3 corner blends + adaptive timing**
(`rounding.py`, `smooth_v2.py`, `selected_v2.json`), after the original candidate
selection. The old exact-polyline requirement forced a complete stop at each
non-collinear corner—even tiny bends. Local non-overlapping fillets removed
literal stops but still produced severe slowdowns between closely spaced bends.
The selected method blends adjacent direction changes together instead.

In normalized joint arc length, express the original polyline as a straight
term plus hinges `delta_velocity * max(s - corner, 0)`. Replace each hinge near
its corner by the integral of a quintic smoothstep. Compact blend intervals may
overlap: a tiny intermediate segment no longer forces a tiny rounding radius.
Outside those intervals the original straight sections remain. End caps change
progress only along the first/last straight segments, preserving exact endpoints
and rest. The result has continuous position, tangent, curvature and third
path derivative (C3). The v1 adaptive engine then times **that new checked curve**.
This is custom polynomial blending, not a claim to implement MoveIt TOTG.

### Controlled changes and validation

* Try halfwidths 0.06, 0.05, 0.04, 0.03, 0.02, 0.01, 0.005 of joint arc, shrinking
  if a wider blend fails. Only machine-precision collinear points are removed;
  real corners are not discarded by approximate simplification.
* Bound the complete new curve to a **1 degree L2 joint-space tube** around the
  original polyline, using sampled distance plus a continuous derivative/Lipschitz
  bound. This is not a one-degree workspace or stick-angle guarantee.
* Separately measure wrist and stick-tip deviations densely; reject above 10 mm.
  These Cartesian deviation measurements are sampled, not formal continuous bounds.
* Check continuous joint-position extrema and the actual new curve against the
  **same full robot, custom tools, stick, camera obstacles and existing zone**.
  Margins are unchanged. Recheck the final time-parameterized curve independently
  at 2 ms intervals plus phase events with the existing dense edge checker.
  These are dense numerical checks, not a continuous collision certificate.
* Retain 0.6 rad/s, 1.2 rad/s², 6 rad/s³ and the v1 duration reserve. Check derivative
  extrema and independently sampled derivatives; do not relax limits to remove stops.
* Compare scale-independent joint-speed metrics. A satisfactory blend has no
  internal stops (<5% cruise) or deep slowdowns (<25%), speed CV <=0.45 and
  tenth-percentile/cruise >=0.25. Cruise is the interior 90th percentile. New
  outward duration must be <=1.2 times v1. Endpoint ramps are excluded as documented
  in `motion_quality.py`. These are engineering screening metrics, not human ratings.
* Try at most two collision-valid timed curves, using 8/16/32/64/96 density
  controls. If none qualifies, **explicitly label and retain v1**; never hide the
  fallback or weaken obstacle checks. No additional UI locks are introduced.

All four default left choices pass. Single arc changes from 5.174 to 3.391 s
outward (4 to 0 internal stops); Clearance from 9.051 to 3.473 s (13 to 0).
`left_recordings/record1.json` is a **post-generation comparison only**, never a
source of route points, seeds, timing commands or a safety certificate. Endpoint
poses still come from the existing recording-final-pose interface.

The v2 cache is separate and hash-verified against original geometry, scene/pool,
code and policy. Plans retain `source_timing` and `original_path_waypoints`, and
store the rounded coefficients, timing law, bounds, attempted/rejected blends and
validation evidence. Preview evaluates the analytic law for seek/play/reverse;
the opposite arm stays fixed. The two-second hold and exact return are unchanged.
Speed controls are preview multipliers, not higher benchmark motion limits.

No new dependencies or GPU compute are required. Geometric searches remain serial. Multiple
checkpoint timings use two child workers within the same 3 GiB aggregate memory
cap, 128 MiB swap cap and CAN-denied planning job. Rendering geometry/detail is
unchanged. For experiment details, saved variants, metrics and validation see
`diagnostics/corner_smoothing_20261021/REPORT.md`. The reusable study is:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv-planner-experiment/bin/python \
  -m drum_planner.timing.rounding_study --output planner_runs/my_corner_study \
  --phase refined --cases left_single_arc,left_clearance --levels 8,16,32
```

Background: [SciPy piecewise polynomial representations](https://docs.scipy.org/doc/scipy/tutorial/interpolate/splines_and_polynomials.html)
and [BPoly endpoint derivative construction](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.BPoly.html).
MoveIt's [time parameterization documentation](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html)
also explains why path-changing smoothing needs collision rechecking; we perform
that rechecking rather than assuming that a small deviation is harmless.

## Version 1: exact-path timing experiment (preserved)

The sections below describe v1 and its original benchmark. V1 changes **when**
a selected path is traversed, never its geometry. Its original code snapshot,
unchanged policy and frozen benchmark remain available; left v2 is separate.

## Algorithms retained

All use 0.6 rad/s velocity, 1.2 rad/s² acceleration and 6 rad/s³ jerk per joint,
rest endpoints and the existing 1.001 duration reserve.

1. **Global scaling** (`core.global_law`): existing normalized quintic curve and
   one global duration multiplier. This is the historical timing reference.
2. **Local quintic** (`core.local_law(..., 'local_quintic')`): independently time
   straight rest-to-rest sections. The duration is the maximum of the exact
   velocity-, acceleration- and jerk-limited quintic durations. On an already
   continuous curved path it retains the original clock, rather than adding
   stops or approximating the geometry by straight chords.
3. **Local S-curve** (`core.local_law(..., 'scurve')`): analytic seven-phase
   jerk-limited rest-to-rest time optimum on each straight section, with
   triangular/acceleration-plateau/velocity-cruise cases. All joints use one
   scalar phase, preserving the straight line in joint space. The same curved
   fallback applies. No external Ruckig dependency is used.
4. **Adaptive fixed-path retiming** (`adaptive.optimize`, selected winner):
   the analytic S-curve branch for straight sections; for an original curved
   spline `q(s)`, optimize a positive cubic time density `h(s)=dt/ds`. Minimize
   `integral(h ds)` using SLSQP with analytic constraint Jacobians. The original
   spline coefficients remain bit-identical. This is one automatic policy,
   not a separate manual method choice for each arm.

The local methods test stopping at every vertex versus passing exactly collinear
intermediate vertices without stopping. **Every original waypoint is retained
and traversed in order**. Only machine-precision collinearity (1e-13 rad residual)
is accepted; no actual corner is rounded. Every non-collinear corner remains a
rest-to-rest boundary. On these frozen inputs the right paths are continuous
curves and the left paths are corner-stop polylines, explaining the branches.

For the curved branch, with primes denoting `s` derivatives:

```
velocity     = q' / h
acceleration = (q'' h - q' h') / h^3
jerk         = (q''' h^2 - 3 q'' h h' - q' h h'' + 3 q' (h')^2) / h^5
```

Positive B-spline coefficients enforce a monotone clock. Constraint samples are
refined at between-grid extrema. All polynomial intervals and both sides of
knots are then checked using stationary roots of these rational functions;
any remaining overage is removed by scaling the clock. An independent 2 ms
derivative check and existing MoveIt/FCL dense joint/zone/collision check follow.
No collision margin, zone definition or geometric optimization is changed.

## Tuning and result

The full benchmark used 8, 12, 20, 32, 48, 64, 96, 128 and 192 density controls,
`ftol=1e-11` and an iteration ceiling of 1000. Both curved cases converged at
every stage. The final refinement improved duration by only 0.064% and 0.060%.
The analytic local families have no remaining iterative tuning parameter after
selecting the faster exact-collinear policy. This establishes a practical
plateau for these tested families, **not a global-optimality proof for curves**.

Adaptive timing won the unweighted four-case average: **5.646 s outward**,
versus 33.387 s with the old clock. Full results, input hashes, all sixteen laws
and checks are retained in `diagnostics/timing_experiment_20261021/`.

Timing runs **after** the existing candidate ranking, inside the same contained
worker (3 GiB memory cap, 128 MiB swap, one worker at a time, CAN denied). A
separate hash-verified timing cache binds geometry, the scene/pool, code and
policy. No GPU computation or additional dependencies are required; rendering
continues to use the existing GPU-backed VTK scene without lowering detail.

## Reproduce without hardware

From the repository root, with the existing planner environment installed:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv-planner-experiment/bin/python \
  -m drum_planner.timing.benchmark --output planner_runs/my_timing_benchmark \
  --levels 8,12,20,32,48,64,96,128,192 --ftol 1e-11 --maxiter 1000
```

This developer command runs serially and denies CAN in Python. The production
experiment launcher additionally places workers in the OS memory/address-family
sandbox. `--math-only` omits FCL and is not equivalent to the full validation.
The frozen benchmark uses curated, hash-identical URDF/SRDF copies so it does not
depend on disposable old session models. It does not open the camera.

Background: [Ruckig's original paper](https://arxiv.org/abs/2105.04830) describes
jerk-limited time-optimal state transitions, and its
[official tutorial](https://docs.ruckig.com/tutorial.html) distinguishes phase
synchronization (straight-line motion) from ordinary time synchronization.
Our straight branch is the simpler scalar symmetric rest-to-rest special case;
the curved branch is custom fixed-path time-density optimization, not Ruckig.
