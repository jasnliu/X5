# Sparse IPOPT hardware fork — 2026-10-23 UTC

## Delivered

```bash
./plan_drums.sh --hardware-sparse --right
./plan_drums.sh --hardware-sparse --left
```

Original `--hardware`, `singlearc`, `baseline` and experiment modes retain their
existing timing. The fork reuses the existing hardware UI and controller rather
than creating a second, divergent motor-control implementation. Only seven shared
files gained explicit opt-in routing. The original timing implementations and
policies, Workspace curve generator, native collision code, renderer and actuator
code are byte-identical to the pre-task snapshot.

New code: `drum_planner/timing/sparse.py` and `sparse_integration.py`. Optional
`casadi==3.7.2` was installed into the planner venv with `--no-deps`; no existing
packages were upgraded. `setup_planner_sparse.sh` reproduces the installation.
See [SPARSE_HARDWARE.md](../../SPARSE_HARDWARE.md) for operation.

## Algorithm boundary

- Same frozen Workspace curve, candidate selection and destination/checkpoint
  behavior; no recording interior samples become planning inputs.
- Same positive cubic time-density parameterization, nine control resolutions
  through 192, up to three between-grid refinement passes, limits and reserve.
- Sparse IPOPT/MUMPS replaces dense SLSQP. Signed inequalities on each joint are
  equivalent to the maximum-absolute-joint constraint. Sparse symbolic derivatives
  avoid introducing its nonsmooth maximum operator into the solver.
- Independent exact velocity/acceleration/jerk extrema and dense collision/zone
  validation remain. Speed profiles are not guaranteed bit-identical to SLSQP.
- The sparse fork currently does no timing-cache lookup. It does not silently
  fall back to dense SLSQP. Its original feasible global-clock incumbent remains
  available if no sparse candidate improves it; that outcome is explicitly marked.
- No compiled-check rewrite, extra parallel workers, analytical robot-FK rewrite
  or GPU method was added. Existing two-worker timing and shared 3 GiB cap remain.

## Cold mathematical verification

Five frozen real selected curves were recomputed at all policy levels, with no
timing cache. All sparse solves converged; no global-clock fallback was selected.
Each received a fresh native world check.

| Curve | Sparse calculation | Original movement duration | Sparse movement duration |
| --- | ---: | ---: | ---: |
| Right checkpoint 1 | 36.70 s | 4.210607849 s | 4.210607862 s |
| Right checkpoint 2 | 82.63 s | 1.937870211 s | 1.937870197 s |
| Right checkpoint 3 | 72.81 s | 2.393038925 s | 2.393038637 s |
| Right return | 83.83 s | 3.867521401 s | 3.867521159 s |
| Left Workspace curve | 57.66 s | 2.839629375 s | 2.839629273 s |

These are measured cold solve times, not total Calculate times or a controlled
speedup comparison: other software tests ran concurrently. Sparse optimization
has setup overhead and is not promised faster on every curve. The original mode
is retained for direct comparison rather than silently replacing it.

Path polynomial coefficients were **bit-identical**; maximum movement-duration
difference was **0.288 microseconds**. The speed-variation metrics remained very
close and no internal stops or deep slowdowns were added. Exact continuous limits,
rest endpoints and native collision/zone checks passed. This is measured evidence
on these inputs, not proof of global optimality or all future cases.

## Application verification

- **886 existing regression tests passed**, with the known SciPy internal trial
  bounds warning. No physical arm positions were clipped.
- **16 sparse/preservation tests passed**, including signed-constraint equivalence,
  Jacobian sparsity, exact geometry, defaults, CLI routing and worker propagation.
- Final combined sparse/repository/preservation audit: **26 passed**.
- Actual three-window UI tested for **both arms**, using saved scenes and fake
  motors. One and three destinations were calculated with the sparse backend.
  Preview play/pause, scrub, speed, normal playback/holds/independent return,
  Emergency Hold, traveled-prefix Reverse + relax, destination edits and Cancel
  all passed. Six simulated owner cycles completed; no UI/rendering errors.
- **Ten actual UI-generated legs** matched the preserved default UI paths exactly;
  durations matched within 10 microseconds and no additional stop/slowdown appeared.
  This includes the separately generated final-checkpoint-to-rest returns.
- Actual `plan_drums.sh` startup tested in sparse-right, sparse-left and original
  hardware-right **offline simulation** modes. No automatic calculation/Start;
  all controls remained available and the timed preview closed cleanly.
- An actual sparse worker was cancelled during optimization, reaped, and did not
  publish a ready law. Peak calculation-group memory observed in the UI tests was
  **740,229,120 bytes (about 706 MiB)** within the unchanged aggregate 3 GiB cap.

The first regression run found one historical assertion that demanded the entire
acquisition function source remain text-identical. It now strips only the new
default-inactive sparse parameter/forwarding line and still compares the entire
remaining function exactly. The full suite was rerun successfully; no behavior
test was disabled to hide a failure.

**All work was software-only: no physical camera acquisition, CAN access, motor
commands, enabling, holding, releasing or physical movement.** Native UI playback
used simulated motors, not physical execution. The original fault/recovery code
was not modified.

## Evidence

- `verification.json`, `parity.json`, `cli.json`: aggregate and comparison results.
- `evidence/`: compact benchmark/UI/cancellation reports and inspected screenshots.
- `before_hashes.json`, `before/`: original source boundary.
- `regression_final.log`, `final_sparse_tests.log`: completed test logs.
- `benchmark_sparse.py`, `verify_parity.py`, `check_cancel.py`: reproducible local
  study tools; the former two reference retained local run inputs.
- Full disposable results remain local under `planner_runs/sparse-20261023-*`.
  They are not added wholesale to Git.

Regression commands:

```bash
./diagnostics/planner_optimization_20261022/run_tests.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH="$PWD:${PYTHONPATH:-}" \
  .venv-planner-experiment/bin/python -m pytest -q tests/test_sparse_hardware.py
```
