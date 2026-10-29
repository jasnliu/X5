import json,statistics
from pathlib import Path
b=Path(__file__).resolve().parent
load=lambda p:json.loads(p.read_text())
legs=['checkpoint_1','checkpoint_2','return_to_start','left'];names=['Saved rest → R1','R1 → R4','R4 → saved rest','Left nominal destination']
rows=[]
for leg,name in zip(legs,names):
 p=b/('left-baseline/plan.json' if leg=='left' else 'baseline1/'+leg+'/plan.json');q=b/('left-final2/plan.json' if leg=='left' else 'final6/'+leg+'/plan.json')
 old=load(p)['duration_outward_s'];new=load(q)['duration_outward_s'];rows.append((old,new))
lines=['| Movement (holds excluded) | Previous GPU | New CPU | Reduction |','|---|---:|---:|---:|']
for name,(old,new) in zip(names,rows):lines.append(f'| {name} | {old:.6f} s | {new:.6f} s | {100*(1-new/old):.2f}% |')
old=sum(p[0] for p in rows[:3]);new=sum(p[1] for p in rows[:3]);lines.append(f'| **Right three-leg total** | **{old:.6f} s** | **{new:.6f} s** | **{100*(1-new/old):.2f}%** |')
before=[load(b/f'baseline{i}/cold.json')['elapsed_s'] for i in [1,2,3]];after=[load(b/f'final{i}/cold.json')['elapsed_s'] for i in [4,5,6]]
trade=load(b/'final-tradeoff/report.json');tr=['| Controls | Right movement | Right timing compute | Left movement | Left timing compute |','|---|---:|---:|---:|---:|']
for n in [96,192,384]:
 a=[r for r in trade if r['n']==n];right=[r for r in a if r['case']!='left'];left=next(r for r in a if r['case']=='left')
 tr.append(f"| {n} | {sum(r['duration_s'] for r in right):.6f} s | {sum(r['compute_s'] for r in right):.3f} s | {left['duration_s']:.6f} s | {left['compute_s']:.3f} s |")
report='''# Fixed-path movement-speed experiment — 2026-10-24

## Decision and scope

Use **96-control sparse CPU timing with batched continuous polynomial-root checks**
for the right and left `plan_drums.sh experiment` programs. CPU use is intentional,
not a GPU failure. This gives shorter motion at essentially unchanged cold
calculation time. Keep the exact current path generator, selected path, limits,
reserve, collision/zone policy, endpoint rest conditions and two-second holds.
No hardware program, shared timing policy, launcher, destination, calibration,
model, package, notification setting, Git history or artifact policy was changed.
No camera/CAN/motor access or physical motion validation occurred.

## Measured results

'''+ '\n'.join(lines)+f'''

For the right mission, the two unchanged holds add 4 seconds to either movement
total. The final return is independently planned, not retraced.

Fresh-process **full calculation**, including imports, request/model setup,
all three newly generated paths, timing and final checks:

- Previous GPU runs: {', '.join(f'{x:.3f}' for x in before)} seconds;
  median **{statistics.median(before):.3f} s**.
- New CPU runs: {', '.join(f'{x:.3f}' for x in after)} seconds;
  median **{statistics.median(after):.3f} s**.
- Contained job wall times (including the small launcher overhead): previous
  7.094–7.216 s; new 7.101–7.211 s.
- Current full saved-input launch **including scene/session preparation**:
  **8.239 s** (1.104 s preparation + 7.135 s calculation); the preceding full
  launch measured 8.160 s.
- Left nominal fresh full calculation: **6.371 s before / 6.824 s after**
  (the preceding new-optimizer run measured 6.718 s).
  Left path generation alone was 4.075 / 4.293 s in those runs; new timing itself
  was 0.955 s. This single left comparison includes normal path-fit runtime noise.

These are fresh processes and fresh output directories, not cached plan loads.
Inputs are the same saved scene/poses/destinations. Every path is recalculated.
No CPU-reference timing law, prior optimizer solution, learned template or result
cache is read by the new optimizer. Shader disk caching was disabled in GPU
comparisons. Existing installed code/compiled binaries are not rebuilt or the OS
rebooted for each trial; “cold” here means fresh calculation, not cold disk pages.

The source path coefficients, waypoints, source-clock data and selected candidate
are exactly equal before/after for all four nominal cases. Endpoint, selected
arm, scene-mask pose and model bytes are compared separately. See `summary.json`
and `preservation.json`.

## What changed mathematically

The geometric joint curve q(s) stays identical. A positive cubic B-spline
h(s)=dt/ds changes only how quickly the arm traverses it. A fresh bounded
24-control vectorized CPU solve provides the initialization and a checked backup.
A sparse IPOPT solve then minimizes the integral of a 96-control density.

Velocity constraints reduce exactly to h >= max_j |q'_j| / 0.6. Signed acceleration
and jerk constraints include **all seven joints** at every collocation point.
The bounds remain 0.6 rad/s, 1.2 rad/s² and 6 rad/s³. The final checker finds
stationary roots of the same rational derivative expressions as the unchanged
`timing/core.py`, on every original path/density interval and both knot sides.
Positivity and density/first-derivative continuity are checked as well. A final
homogeneous correction plus the original 0.1% reserve enforces continuous limits.
Playback uses the exact original path composed with this monotone clock.

The important calculation optimization is batching affine polynomial transforms,
convolutions and small companion eigenproblems instead of repeatedly constructing
Python Polynomial objects. For tested 96-control laws, continuous checks fell
from about **0.52 s to 0.018 s**; the largest absolute peak discrepancy over the
28 retained comparison laws was below 7e-15. Independent original-checker tests
also cover random positive densities, zero polynomials, discontinuities,
interior nonpositivity and eigensolver failure. No sampled-only certification or
relaxed margin was substituted.

The normal sparse solve is bounded to 90 iterations / one second of solver wall
time, with at most one extra refinement when continuous correction is meaningfully
larger than the reserve. Setup/seed/verification are additional time. Invalid,
nonconverged or slower attempts never replace a better continuously checked
incumbent. Missing/broken sparse dependencies or no improvement yield a clearly
logged checked seed/uniform fallback. Nonconvergence is explicitly logged even
when a useful checked partial result wins. A batched eigenproblem failure falls
back to the unchanged root checker with an explicit log.

## Alternatives and stopping point

The diagnostic study compared higher-control CPU and Vulkan augmented-Lagrangian
solves, fresh CPU initialization followed by GPU refinement (hybrid), sparse
IPOPT, C1/C2 quadratic/cubic/quartic/quintic densities, constraint screening,
polynomialized inequalities, nonuniform knots and one/two/three refinement rounds.
All were on the same four fixed curves. Several exotic parameterizations were
slower or numerically unreliable; they were discarded, not installed. Screening
out initially non-dominant joints was especially unreliable and is **not used**.
A feasible scaled research result is not automatically a useful movement result.

The final resolution tradeoff was repeated with the **same fast root checker**
for every contender, not by unfairly leaving slow checking on the reference:

'''+ '\n'.join(tr)+'''

Doubling 96 to 192 controls saved only **26 ms across the entire right mission**
while adding about **1.65 s** of timing computation. Going to 384 saved only
another 6 ms while adding several more seconds. Left saved 18 ms for about
0.44 s extra computation at 192 controls; 384 saved only another 3 ms.
I stopped at 96 controls: the extra movement gain is too small for this task's
cold-start balance. This is a measured practical stopping point, **not proof of a
global time optimum or of reaching physical actuator capability**. The best
high-resolution reference is not a certified lower bound. Necessary scalar/velocity
lower bounds are retained in `bounds_initial.json` but are too loose to certify
optimality under coupled fixed-path constraints.

## Verification

- **672 software regression tests passed**, one warning from an unchanged legacy
  SLSQP rounded-left test; see `pytest-final2.log`. Includes current GPU legacy,
  hardware routing, original/sparse timing, checkpoint, planner and scope tests.
- Eight fixed curves (three right mission legs, nominal left and ±0.01 rad endpoint
  variations on both arms) improve over the preceding GPU timing. Original root
  checker independently verifies every final law; exact geometry, endpoint rest,
  dense derivatives, no internal stops/deep slowdowns are checked.
- Full fresh nominal plans retain dense 2 ms collision/zone and 0.003 rad L1 edge
  checks. Each arm's saved scene and original tool/zone policies are retained.
- Native UI verification results are recorded beside `ui-right`, `ui-left`,
  `checkpoints-right` and `checkpoints-left`: both arms passed startup-unselected,
  actual selection, invalid/no-route recovery, cancel/retry, play/pause/scrub/speed,
  full cycles and optional one/two/three checkpoints with new returns. No VTK/Qt
  errors. READY screenshots use the existing framebuffer-aware capture routine;
  backend text explicitly says intentional CPU. These are software tests only.
- Hash audit compares 267 pre-task files. Only the experiment integration/menu/UI
  and experiment documentation are edited; new optimizer/checker and tests are
  separate files. Shared core, original GPU, both hardware modes, path generator,
  inputs, model and limits are unchanged.

## Reproduce and evidence

From `/home/jason/Proyectos3/X5`, source ROS/install and bound math threads:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
OUT=diagnostics/timing_speed_20261024/replay-$(date +%s)
.venv-planner-experiment/bin/python diagnostics/fast_gpu_20261024/run_job.py \\
  "$OUT-job" diagnostics/timing_speed_20261024/cold_case.py \\
  --context planner_runs/20261024T002242Z-experiment-c440af/context.json --output "$OUT"
# Add --baseline to load the exact saved pre-task fast experiment module.
# For left, use planner_runs/gpu-20261024-ui-left/context.json.
```

- `before/`, `before_hashes.json`: exact pre-task experiment/timing source and hashes.
- `baseline1..3`, `final4..6`, `left-baseline`, `left-final2`, `full-launch-final`: cold logs,
  containment reports, actual plans, validation and timing provenance.
- `study2`, `warm`, `quartic`, `screen`, `single`, `polynomial`, `warped`, `seed`:
  bounded diagnostic candidate studies; `*_configs.json` saves settings.
- `final-tradeoff`: final-checker resolution comparison; `verify2` has the eight
  final laws and independent proofs. `batch_report.json` cross-checks math/runtime.
- `summary.json`, `preservation.json`: final numerical/input/source audits.
- `study1` was discarded after a diagnostic grid bug (nextafter(0,-inf)); the
  corrected implementation filters samples to [0,1] and has a regression test.
  `new1..3` precede batched checking; `new3` and the seed trial briefly overlapped,
  so neither is used for performance conclusions. `final1..3` precede only a
  minor logging correction (do not announce a third solve after the second);
  `final4..6` and the final regression run use the delivered source bytes. Reported final cold runs are
  serialized with no other heavy calculation running.
- Diagnostics stay local under the existing generated-artifact policy; this task
  does not change the Git allowlist, stage/commit data or delete earlier studies.

Primary background consulted: [CasADi documentation](https://web.casadi.org/docs/),
[TOPP3 third-order fixed-path timing](https://arxiv.org/abs/1609.05307), and
[jerk-constrained manipulator timing study](https://arxiv.org/abs/2404.07889).
The implementation uses the project's existing exact density equations; no claim
that a second-order speed planner alone satisfies the existing jerk constraint.
'''
(b/'REPORT.md').write_text(report)
