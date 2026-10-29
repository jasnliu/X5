# Optional sparse-timing hardware fork

## Shared right-J2 position correction (2026-10-28)

This fork retains its existing path/timing algorithm, drive settings and
velocity/acceleration/jerk policy. Like other new selected-right planner models,
its generated URDF now imposes J2 >= 0° in encoder/display coordinates, after
calibration mapping. Other bounds and left behavior remain unchanged. Old
right-arm preparations require recalculation before a new Start; existing
checked prefix recovery remains readable. See
[the operating-limit study](diagnostics/right_j2_operating_limit_20261028/REPORT.md).

**2026-10-26 clarification:** normal `--hardware` now uses the newer experiment-equivalent
path/recovery and configurable timing pipeline. This opt-in `--hardware-sparse`
fork still uses its historical path generator and fixed original limits; it does
not acquire DT1 or expose the new profile controls. Comparisons below are dated
historical results, not a claim of current parity with normal hardware.

```bash
./setup_planner_sparse.sh               # once, installs pinned optional dependency
./plan_drums.sh --hardware-sparse --right
./plan_drums.sh --hardware-sparse --left
```

The original commands remain unchanged:

```bash
./plan_drums.sh --hardware --right
./plan_drums.sh --hardware --left
```

The sparse fork reuses the same three windows, Workspace curve generator,
destination/checkpoint selection, Calculate/Start buttons, preview speed,
Hold, traveled-prefix Reverse + relax and independently planned normal return.
Opening either program does not calculate a trajectory or enable/move an arm.
The sparse control-window title identifies **SPARSE IPOPT FORK**. Do not run two
hardware workspaces against the same physical buses concurrently.

## What changes

Only the fixed-path timing solver changes: sparse IPOPT through CasADi replaces
dense SLSQP. Path generation and ranking are the same. The positive cubic
`dt/ds` parameterization, all nine refinement levels through 192 controls,
three between-grid refinement passes, limits and reserve are retained. Signed
per-joint inequalities replace the equivalent maximum-absolute-joint constraint,
allowing smooth sparse derivative matrices. IPOPT uses exact symbolic derivatives
and its sparse MUMPS linear solver, not GPU acceleration or generated C code.

Every selected curve still receives exact continuous extrema checks and the
existing dense collision/zone check. The workspace path coefficients are not
refitted. The resulting speed profile is **not promised bit-identical** to SLSQP:
this is a separate solver experiment, not a replacement of the approved version.
No global timing optimum is claimed. Small numerical differences are expected.

The sparse fork currently performs a fresh timing solve on every Calculate;
there is no timing-cache lookup or hidden dense-solver fallback. Geometric
run-local caching remains the same as the original. If no sparse iterate beats
the existing feasible global clock, that incumbent is retained and recorded in
the law's `fallback_used` field. Errors leave the ordinary retry/recovery controls
available. Workspace curve paths are continuous; unsupported corner-stop paths
are explicitly rejected rather than silently refitted.

The optional dependency is pinned to `casadi==3.7.2` and installed without
upgrading NumPy/SciPy or other original planner dependencies. It is loaded in
`.venv-planner-experiment` by a disposable timing child, never the feedback thread
or motor controller. Existing two-worker timing concurrency and the shared 3 GiB
calculation-group limit are unchanged. Cancel includes the sparse children.

## Offline use and progress

```bash
./plan_drums.sh --hardware-sparse --right --simulate PATH_TO_RIGHT_PLAN
./plan_drums.sh --hardware-sparse --left --simulate PATH_TO_LEFT_PLAN
```

Calculate in this offline mode creates a new sparse plan from simulated joints,
selected destinations and the saved scene; Start operates fake motors only.
The terminal reports each route, `SPARSE IPOPT` refinement level, nonzero Jacobian
count, computation time and final verification. The original hardware/experiment
commands do not select the sparse solver unless explicitly requested.

Implementation: `timing/sparse.py`, `timing/sparse_integration.py`, with opt-in
plumbing through the existing hardware UI and worker processes. Solver reference:
[CasADi nonlinear programming documentation](https://web.casadi.org/docs/#nonlinear-programming).
