# Exact-output planner optimization — 2026-10-22

Implemented in the shared hardware/experiment planning pipeline, for both arms.
Commands, UI, methods, route selection, collision/zone policy, camera resolution,
motion limits, checkpoint holds and return behavior are unchanged.

## Changes

1. **Cache exact timing mathematics independently of the camera scene.** The key
   includes the complete curve and original clock, solver policy, implementation,
   limits and numerical environment. Reusing a timing law does **not** reuse a
   live-scene collision proof: the requested scene is still checked. Corrupt,
   disabled or unavailable storage falls back to calculation. The existing bounded
   cache remains 512 MiB / 128 entries. A short bounded lock wait prevents two
   timing workers from unnecessarily recomputing an existing entry.
2. **Run up to two checkpoint timing jobs concurrently.** All routes, including
   the independent final-checkpoint-to-rest return, are selected serially first.
   This avoids concurrent timing reducing a time-budgeted route search. Fresh
   interpreter children remain in the existing aggregate 3 GiB calculation group;
   cancellation includes them. No GPU dependency or additional motor owner.
3. **Reuse invariant work.** Reuse model/scene setup within a checkpoint sequence
   and exact path derivative polynomials within a timing solve. The left smoothing
   pipeline reuses its loaded world but repeats the final validation. No solver
   refinement levels, candidates, tolerances or samples were removed.
4. **Batch playback preparation.** Evaluate the same 2 ms timeline in arrays rather
   than one Python call per timestamp. Positions are bit-identical, including
   multi-checkpoint missions and their return legs.

## Measured performance

Four actual right-arm Workspace curves: three checkpoints and the independent
return from the saved successful `20261022T215453Z-hardware-right-8993f5` run.
The old implementation was preserved and recomputed at full policy settings.

| Timing workload | Measured wall/solve time |
| --- | ---: |
| Old implementation, sum of four serial solves | 1,278.09 s |
| New implementation, cold two-worker batch | 878.27 s |
| New implementation, cached batch **including fresh scene validation** | 3.32 s |

The cold batch used **31.3% less time** in this measurement. These are **not total
Calculate times**: acquisition and geometric route search are additional. A
user-owned camera/audio program imposed heavy, varying background CPU load during
the comparison. It was left alone. These results demonstrate a measured benefit,
not a controlled universal speedup guarantee. Cache benefit requires an exactly
identical curve/clock/policy; a new camera scene can produce a different curve and
therefore require a new solve. No approximate matching is used.

Warm batch evaluation examples improved from 114–245 ms per leg to 0.62–1.20 ms
without changing positions. This is a smaller preparation-stage saving, not the
main cold-solve improvement. Full left smoothing checks also preserved the laws;
warm timing/checking took 2.81 s and 3.41 s for the two selected left benchmarks.

Observed peak memory was about 475 MiB for the cold timing batch, about 489 MiB
for the largest native-UI calculation, and about 539 MiB for full mission parity.
The 3 GiB limit is for the calculation group, not the entire desktop application.

## Verification

- **883-test regression suite passed**, with one existing SciPy warning about an
  internal trial iterate being bounded (not clipping measured arm positions).
- After the final cache-lock improvement, **48 cache/checkpoint tests passed**.
  Two additional scalar-versus-batched mission checks also passed.
- Final repository, cache, checkpoint and hardware-workspace audit: **84 passed**.
- Full-policy comparison: all four right curves and both left rounded benchmark
  curves retained identical numerical laws, durations, geometry and samples.
  Runtime counters are intentionally excluded from numeric equality comparisons.
- Fresh native collision/zone validation passed on all benchmark curves. Left
  smoothing proof using the reused world equaled a separate independent check.
- **Both native hardware UIs tested with simulated motors and saved inputs**:
  one and three destinations; Calculate; preview play/pause/scrub/speed; simulated
  Start; checkpoint holds and final return; Hold; Reverse + relax; Cancel/retry;
  destination add/remove/exclusion; renderer error inspection. No reported UI or
  rendering errors. Both three-window layouts were visually inspected.
- Ten leg comparisons and two complete three-checkpoint missions matched between
  the experiment and hardware pipelines: identical candidates, timing and cycle.
- Real timing-child cancellation reaped all disposable processes without resource
  warnings or publishing a partially ready mission.
- Hash audit confirms the 87 other captured source/input files were unchanged;
  the ten intended source changes are listed in `preservation.json`. Physical
  control logic and selected timing policies were unchanged.

All testing in this task was **software-only**: no real camera acquisition, CAN
access, motor commands, enable/disable or physical motion. This is not a claim of
physical execution validation or that all future inputs are bug-free.

### Development-test notes

The initial cold benchmark completed all timing work but its reporting harness
incorrectly expected a `valid` field instead of the existing `samples_checked`
proof schema. The harness was corrected and every completed result was checked
again; the reported cold wall time conservatively uses the original job elapsed
time. A first GUI launch lacked the repository PYTHONPATH; it was rerun correctly.
Early cancellation semaphore warnings were eliminated by replacing the temporary
multiprocessing-pool implementation with bounded subprocess/file IPC. The final
tests above apply to the corrected implementation.

## Evidence and reproduction

- `verification.json`: aggregate result and measured times.
- `evidence/`: compact copies of benchmark, UI, parity and cancellation results.
- `reference/` and `before/`: preserved pre-change implementation.
- `before_hashes.json`, `preservation.json`: change boundary audit.
- `regression_final.log`, `final_cache_regression.log`, `final_audit.log`: actual
  test output.
- `run_tests.sh`: software regression entrypoint.
- `verify.py`, `batch.py`, `check_smoothing.py`, `mission_parity.py`: full-policy
  study tools. The full-study tools reference this checkout's saved run inputs;
  bulk runtime inputs, candidate pools and temporary IPC remain local under
  `planner_runs/optimization-20261022-*`, not added wholesale to Git.

Run the regular regression suite with:

```bash
./diagnostics/planner_optimization_20261022/run_tests.sh
```

Normal application commands remain unchanged:

```bash
./plan_drums.sh --hardware --right
./plan_drums.sh --hardware --left
./plan_drums.sh experiment --right
./plan_drums.sh experiment --left
```
