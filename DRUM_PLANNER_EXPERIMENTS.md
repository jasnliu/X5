# Software-only gesture experiments

```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh experiment
```

The robot can remain **off and disconnected**. Startup reuses the **saved camera
scene and saved resting pose used in the record3 comparison**. It does not open
CAN or the camera. Both defaults and their hashes are in
`drum_planner/experiment/default_inputs.json`. The main goal is still **only the
final joint pose of `recordings/record3.json`**. Recording interiors are never
planner seeds, waypoints, training data, or objective inputs.

Standalone `./plan_drums.sh baseline` and its hardware behavior are unchanged.

## Four retained candidates — not a declared winner

The previous baseline/reversals/coordination/spatial/combined experiment menu has
been replaced. The initial research screened eight approaches, then tested five
compact-route variants; compact corridor generation is now a shared backbone,
not a redundant fifth menu entry.

| Mode | What it does | Trade-off to review |
|---|---|---|
| `progress` — Joint progress | Fresh endpoint-corridor RRT routes, then collision-checked partial-joint shortcuts. Ranks backtracking, workspace detours and overshoot. | Often less motor backtracking; the geometric winner can need a slow corner-stop timing fallback. |
| `sweep` — Broad sweep | Endpoint-preserving elastic deformation of fresh compact routes; emphasizes broad wrist/stick curves. | Can improve the overall sweep while allowing more joint travel. |
| `single_arc` — Single arc | Searches two low-frequency sine components per joint, initialized from compact routes generated for this task, not recordings. | Smooth broad shape, but may use more motor excursion; falls back to a valid compact proposal when necessary. |
| `clearance` — Clearance blend | Ranks the shared generated/refined routes by broad shape plus a soft obstacle-proximity penalty. | Often roomier/simpler wrist routes, but not necessarily closest to the recording or lowest joint travel. |

All modes can honestly select the same route on a particular task. A preferred
refinement is not guaranteed to win: the UI reports the actual source, and the
JSON retains source/fallback provenance. Compact proposals remain available to
all modes. No deliberately bad path is introduced just to make previews differ.

## Same UI workflow

1. The saved scene loads once. **All four methods start calculating/cache
   automatically**; one contained worker at a time. Terminal logs show searches,
   rejected proposals, accepted candidates, scores, selection and cache reuse.
2. Choose a ready method. Switching cached modes does not regenerate or upload
   the depth mesh. An unfinished selection is prioritized next.
3. Play/pause the animation, seek on the timeline, go to start/goal, change
   animation speed, loop, rotate/zoom, switch mesh/points/colors, fit the view or
   save a screenshot. Playback is outward → 2-second hold → exact reverse.
4. **Cancel** stops the worker and remaining queue but keeps finished results.
   **Cache all** resumes missing methods. **Generate** reranks the completed
   pool; change the seed to request different proposals.
5. **Gesture settings…** and the pool budget invalidate/rebuild the relevant
   caches. Failed jobs can be retried without restarting the whole UI.
6. **Recapture scene** is an explicit optional camera operation, never automatic.
   A failed recapture retains the previous scene. Successful recapture replaces
   it only after acquisition/initialization completes, then caches the methods
   again. The arm pose remains saved, not live.

Default shared-pool budget: **120 seconds**, editable 5–600 seconds. Setup and
finishing verification of an already-started candidate are separate. Default
six compact searches; each uses at most four seconds. If no compact proposal is
valid, up to two unrestricted searches are an explicitly labelled fallback.
A no-feasible-route result is a normal result, not permission to weaken checks.

## Objectives and editable settings

Scoring is geometric, using outward path progress rather than elapsed time.
The candidate is timed and checked first. Timing does not improve its score;
hold/return are excluded. Scores are rounded to eight decimal places before
a deterministic candidate-index tie-break, so numerical copies cannot win on
floating-point noise. Duration is not a tie-break. The 120-second absolute timing cap remains; there is
no 2× fastest-duration cutoff. Some low-backtracking routes therefore remain
slow. Their actual duration is displayed rather than hidden.

Let:

- `b` = total extra joint travel divided by net absolute joint change (floor 0.5 rad).
- `d` = mean wrist/stick-tip excess path-length ratio over endpoint displacement.
- `o` = mean normalized excursion outside each endpoint bounding box.
- `T` = 0.65 × broad wrist turning + 0.35 × broad tip turning, in radians at 5 cm scale.
- `S` = RMS disagreement of endpoint-normalized joint progress (0.15 rad denominator floor).
- `P` = mean squared shortfall from 10 cm estimated distal clearance, using the
  existing FCL 5 mm padding bands. This is a soft estimate, **not exact distance
  or a physical safety certificate**.

Default economy objective: `4b + 2d + 2o + 0.10T + 0.5S`.
Default broad-arc objective: `1.5b + 2d + 2o + 0.35T + 0.25S`.
Clearance blend: broad-arc objective `+ 3P`.

`progress` uses economy; `sweep` and `single_arc` use broad-arc; `clearance` uses
clearance blend. There is **no recording-similarity term** in any of these.

| Setting | Default | Range |
|---|---:|---:|
| Extra joint-travel weight | 4 | 0–12 |
| Endpoint-overshoot weight | 2 | 0–8 |
| Clearance-blend proximity weight | 3 | 0–10 |
| Base endpoint corridor expansion | 7° | 3–25° |
| Compact searches | 6 | 2–24 |
| Seed | 19 | 1–2147483647 |

Balanced / Fewer detours / More clearance presets are provided. These change
preferences or the search domain, **not collision padding or zone membership**.
The corridor searches expand the per-joint endpoint ranges by approximately
0.65, 1, 1.65, 1, 2.05, 1.65 times the base expansion, within original joint limits.

## Unchanged checks, model and rendering

- Original full-arm, custom gripper and stick collision geometry and baseline margins.
- Original right `zone3.json`: original gripper TCP point and legacy 5 mm
  membership tolerance; neither the left arm nor the stick defines zone membership.
- Exact start/goal; continuous polynomial joint extrema; 0.6 rad/s, 1.2 rad/s²,
  6 rad/s³ limits; final curve checking at ≤2 ms and ≤0.003 rad joint-L1 edges.
- Frozen original acquisition self-filter for study pose perturbations: changing
  a synthetic scenario does not change which scene points were masked.
- Entire frozen mesh and regular robot visual geometry retained. No rendering
  quality reduction, mesh decimation, or background camera refresh was added.
- One 3 GiB / 128 MiB swap / 96-task contained planning worker, bounded arrays,
  disk-backed candidates, hash-bound cache and whole-process-tree cancellation.
  CAN is blocked by the worker's OS address-family policy and a Python guard.

## Study and evidence

See `diagnostics/gesture_study_20261020/REPORT.md` for the measured comparison,
eliminations, retained trade-offs, held-out tests and UI/production-check evidence.
The protocol, frozen sources, case manifest, candidates, scores and logs are kept
in that directory. No recorded data was overwritten.

Final verification: 518 regression tests passed; all four modes completed native
GUI playback/control tests. The unchanged production checker independently
validated 82 selected curves at 1,077,765 dense states. Final study planning jobs
peaked at 373.7 MiB (3 GiB hard cap); no OOM occurred. These are software tests,
not physical-arm or live-camera validation.

Main benchmark: the exact saved resting pose → record3 final pose. Nearby tests
use ≤1.5° starting-joint changes or ≤3° goal-joint changes. Broader generalization
to a new kit, a very different start/goal or another robot is **not established**.

Recording similarity is evaluated separately AFTER generation. Since record3
starts from a different pose, reports include raw differences and endpoint-chord
residual shape differences. The residual statistic is not physical retargeting,
proof of equal tasks, or a demonstration supplied to the planner. There is no
single combined evaluation score or claim of a universally best humanlike path.

## Commands

```bash
./plan_drums.sh experiment --list-methods
./plan_drums.sh experiment --method single_arc
./plan_drums.sh experiment --headless --method all --budget 120
./plan_drums.sh experiment --state /absolute/saved_state.json --scene /absolute/camera/scene.json
./plan_drums.sh experiment --settings gesture_settings.json
```

Settings JSON can override a subset of the fields above. Old fluidity and
uncertainty fields are rejected rather than silently misinterpreted. Outputs go
to a new `planner_runs/<timestamp>-experiment-<id>` directory (or `--output`).
Saved plans also work with `./plan_drums.sh planned-view PATH/plan.json`.

No additional dependencies were needed. The existing ROS/OMPL/FCL, NumPy, SciPy,
Qt and VTK environment is used. Search/collision calculations remain on CPU;
the existing OpenGL renderer continues to display the full mesh.
