"""Render the measured results, not estimates, into the retained report."""
import json,statistics
from pathlib import Path
b=Path(__file__).resolve().parent;s=json.loads((b/'summary.json').read_text())
g=s['cold_job_wall_s'];cpu=s['old_cpu']['elapsed_s'];launch=s['full_launch'];motion=s['right_legs']
old=sum(r['old_movement_s'] for r in motion);new=sum(r['new_movement_s'] for r in motion)
read=lambda name:json.loads((b/(name+'-job')/'job.json').read_text())['elapsed_s']
pytest=(b/'pytest-final.log').read_text().strip().splitlines()[-1]
text=f'''# Speed-first GPU experiment overhaul — 2026-10-24 UTC

## Result and scope

**Target met on this machine and benchmark.** Both `plan_drums.sh experiment`
arms now expose one fresh GPU workspace pipeline. No hardware planner, original
CPU solver, original Workspace curve generator, collision policy, motion limit,
model or captured destination was changed. No physical/CAN/camera tests occurred.

The fair comparison below is **one Workspace curve mission**, saved rest → R1
→ R4 → independently planned rest, not five old methods versus one new method.

| Calculation | Measured wall time |
|---|---:|
| Retained pre-GPU CPU Workspace curve, freshly rerun | {cpu:.2f} s |
| New Vulkan pipeline, three fresh process/jobs | {', '.join(f'{x:.2f}' for x in g)} s |
| New Vulkan median | {statistics.median(g):.2f} s |
| New Vulkan including fresh launcher/input initialization | {launch['total_s']:.2f} s |
| New bounded CPU fallback, fresh job | {s['cpu_fallback_job_wall_s']:.2f} s |
| Left saved rest → L1, fresh Vulkan job | {read('left2'):.2f} s |

The fair CPU-to-GPU-pipeline reduction is **{cpu/statistics.median(g):.1f}×**.
This is **not a claim that GPU hardware is {cpu/statistics.median(g):.1f}× faster**.
The bounded CPU fallback is slightly faster than Vulkan on this small workload.
Most of the gain is removing the full CPU reference/refinement and redundant
path/world/scoring work. Vulkan remains the requested default; no GPU result is
secretly a CPU-reference result. Fallback is visible in logs and preview labels.

Full initialization is reported separately ({launch['prepare_s']:.2f} s preparation,
{launch['calculation_s']:.2f} s contained calculation). It includes saved-input
setup, not opening/rendering the GUI or capturing a new camera scene.
The contained-job measurements include process/service startup and completion;
internal worker timers in `cold.json` are slightly shorter and are not the
primary table. These are measurements, not a universal latency guarantee.

## Cold protocol

- Fresh Python process, fresh output directory, fresh fitted paths, new native
  collision world, new GPU device/optimizer state. No route/timing/CPU-reference
  cache reads in the new pipeline.
- `MESA_SHADER_CACHE_DISABLE=true` for all final GPU benchmarks. No cached
  driver shader results are required. Precompiled executable/SPIR-V program
  code is retained, as with an installed program; compilation/dependency
  installation and OS file/page caches are not included or globally cleared.
- Same frozen state, endpoint bytes, scene, model/calibration, passive arm,
  zone3, tool geometry, margins and limits. The scene SHA is
  `235bd66e4aa9fe9f19d12879be820a78e811a7f3dd336ee1843b3cab5b78759b`.
- GPU: AMD Radeon 780M Graphics, RADV PHOENIX, FP64. Per-law metadata records
  device/driver identity, implementation hash and actual backend.
- One contained calculation group at a time, 3 GiB aggregate cap, no CAN address
  family. No concurrent benchmarks. New mission legs run serially; one world
  lasts only for that request. A process lock serializes GPU timing across jobs.
- Legacy comparison uses byte-identical pre-GPU worker SHA
  `dba10181a488b7d5f8758a2335d9b72a6a197358a1bb5813a44d0fe44daeead6`,
  a fresh pool, `X5_PLANNER_CACHE_DIR=off`, and its original two timing children.
  This is the retained CPU implementation, including its existing CPU efficiency
  improvements—not a deliberately slowed substitute.

## Movement quality

All three right-arm path coefficient/waypoint identities and their preliminary
source clocks are identical to the freshly rerun old Workspace curve result.
Only the final clocks differ. The left nominal path also matches its retained
CPU benchmark coefficient identity.

| Leg | Old movement | New movement | Increase |
|---|---:|---:|---:|
'''
for r in motion:text+=f"| {r['leg']} | {r['old_movement_s']:.4f} s | {r['new_movement_s']:.4f} s | {100*(r['ratio']-1):.2f}% |\n"
l=s['left'];text+=f"| Left rest → L1 | {l['old_movement_s']:.4f} s | {l['new_movement_s']:.4f} s | {100*(l['ratio']-1):.2f}% |\n"
text+=f'''
Right mission movement totals **{old:.4f} → {new:.4f} s (+{100*(new/old-1):.2f}%)**.
Its two two-second checkpoint holds are unchanged: total preview mission
{old+4:.4f} → {new+4:.4f} s. No added detected internal stops or deep slowdowns.
The roughly 10% target is an offline benchmark criterion, **not a runtime
reference gate** and not a global-optimality proof.

Verification includes continuous rational velocity/acceleration/jerk extrema,
independent dense derivative checks, rest endpoints, original polynomial
identity, continuous joint bounds and final dense FCL/zone checking at 2 ms with
0.003 rad L1 edge subdivision. Numerical density correction keeps the existing
reserve. Whole-path/safety checks were not replaced with the cheap proposal
screen. Limits remain 0.6 rad/s, 1.2 rad/s² and 6 rad/s³.

## Bounded study and discarded approaches

- Initial full-proposal/individual-clearance arrangement: 32.69 s for the right
  mission. Profiling identified repeated soft-clearance mesh rebuilding and
  timing obviously blocked proposals.
- Temporary coarse whole-arm padding preference: 17.26 s; discarded, and the
  original soft-clearance metric restored. No final margin changes.
- Batched original clearance scoring plus coarse rejection: 13.38 s.
- Staged proposals, 24 timing controls: 9.61 s internal worker time. Omitting
  redundant *soft* ranking for a sole candidate reduced this to about 7 s.
- Eight frozen curves: 32 controls took 0.72–1.11 s each, movements +0.85–3.85%.
  A 20-control/short-iteration version reached +13.66% on one case and was
  rejected. Selected 24-control version took 0.55–0.87 s in that initial study,
  with +1.66–4.95% movement. Final full-job timings include all other work.
- Always stopping after moderate-prior proposals was rejected: left movement
  became 3.67 s versus 2.84 s. Current task-independent staged policy expands
  the stronger-prior family when several shapes compete, restoring the old
  left route and its roughly +5% clock. It also explores that family when a
  sole first-stage route fails final validation. No destination-name special cases.

## Generalization, fallback and UI

Fresh ±0.01 rad first-joint endpoint perturbations, not saved destination edits:
'''
for row in s['variations']:
 text+=f"- {row['name']}: {read(row['name']):.2f} s cold job, {row['movement_s']:.4f} s movement; final checks passed, no detected stops/deep slowdowns.\n"
text+='''
Real loader test forced the llvmpipe ICD. It was rejected as a GPU and printed
`GPU UNAVAILABLE — FAST CPU TIMING FALLBACK`; the resulting valid plan/preview
metadata reports CPU, not Vulkan. Numerical execution errors do not silently
fall back. The checked uniform clock guards against a bounded optimizer making
its simple feasible starting clock worse.

Native Qt/VTK checks passed for both arms: startup unselected, actual destination
selection, invalid file recovery, rapid selection cancellation/reaping, fresh
recalculation, no-route fixture recovery, play/pause/scrub/speed/loop, full cycles,
optional one/two/three checkpoints and independently planned return. Right UI
also exercised R2/R3; left checkpoint variations are private diagnostic fixtures.
Frozen scene actors were retained, and no camera or motor access occurred.
Screenshots were visually inspected. These are software checks, not physical
motion validation.

Final software regression run:

```
'''+pytest+'''
```

Original hardware, sparse hardware, CPU timing, workspace geometry and
checkpoint invariants are included in those regressions. `preservation.json`
checks initial source/input hashes and identifies every in-scope existing-file
change. No Git reset, staging, commit, package upgrade or captured-data rewrite.

## Reproduce and retained evidence

Run from `/home/jason/Proyectos3/X5`, source ROS/install, and set
`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1`.
Use new output directories; benchmark scripts refuse to overwrite finished runs.

```bash
OUT=diagnostics/fast_gpu_20261024/replay-$(date +%s)
.venv-planner-experiment/bin/python diagnostics/fast_gpu_20261024/run_job.py \
  "$OUT-job" diagnostics/fast_gpu_20261024/cold_case.py \
  --context planner_runs/20261024T002242Z-experiment-c440af/context.json \
  --output "$OUT"
# Full fresh setup, or the historical CPU comparator (run serially):
.venv-planner-experiment/bin/python diagnostics/fast_gpu_20261024/cold_launch.py "$OUT-launch"
.venv-planner-experiment/bin/python diagnostics/fast_gpu_20261024/old_cpu_once.py "$OUT-old-cpu"
```

- `cold_case.py`, `run_job.py`: fresh actual experiment worker with shader cache off.
- `cold_launch.py`: actual experiment prepare/start_plan workflow, full cold setup.
- `old_cpu_once.py`: original CPU mission with caches off (new directory required).
- `timing_study.py`: bounded control-count study on eight retained curves.
- `make_cases.py`: private ±0.01 rad endpoint fixtures, no destination-folder writes.
- `summary.json`: exact quality/geometry/continuous-limit comparison and timings.
- `evidence/`: compact exact laws, source curves, cloned model bytes and checks.
- `tests/test_fast_gpu.py` and the native UI scripts cover current behavior;
  prior implementations/tests remain available for historical comparisons.

Bulk candidate pools, screenshots from intermediate trials, duplicated plans,
and compiled outputs remain local and ignored. The curated artifact manifest
retains the small reproducible evidence set. The previous experiment source/UI
and documentation are copied under `before/` for comparison/rollback.
'''
(b/'REPORT.md').write_text(text)
