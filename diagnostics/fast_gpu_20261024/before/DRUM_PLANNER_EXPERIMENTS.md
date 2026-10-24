# Software-only gesture experiments

```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh experiment --right
./plan_drums.sh experiment --left
```

The robot can remain **off and disconnected**. Omitting the side retains the
previous right-arm behavior. `--left` and `--right` are mutually exclusive.

Right startup reuses the **saved camera
scene and saved resting pose used in the record3 comparison**. It does not open
CAN or the camera. Both defaults and their hashes are in
`drum_planner/experiment/default_inputs.json`. The GUI starts with **no destination
selected**. Choose a JSON file from `destinations/right` in the **Checkpoint 1**
dropdown to start calculating. `destinationR1.json` is the saved record3 endpoint.
Only the selected file's final joint pose is used. Recording interiors are never
planner seeds, waypoints, training data, or objective inputs.

**Left mode uses the same five algorithms, camera scene, UI, ranking, margins
and worker memory limits**, but changes only left joints 1–7. Its dropdown shows
`destinations/left`; `destinationL1.json` is the saved left record1 endpoint.
Its default start was captured
from the powered-but-disabled left arm on 2026-10-21 UTC using 160 state queries
on `can1`; no motor enable, motion, gripper or relax commands were sent. The right
arm is held at the original saved pose in the simulation and was not queried.
`drum_planner/experiment/default_left_inputs.json` records these input hashes.
Both versions run entirely offline after loading their saved inputs.

### GPU-native timing (both arms)

Experiments now request **Vulkan FP64 timing** on this computer's AMD Radeon
780M. Only the fixed-path timing optimizer is replaced: projected L-BFGS with
an augmented Lagrangian, parallel line search, gradients and optimizer updates
on the GPU. The five path generators/ranking, joint limits, motion limits,
collision/zone checks and approved left-arm C3 geometry selection are unchanged.
Hardware modes, including the separate sparse fork, retain their CPU solvers.

Every accepted GPU law must have a movement duration **no more than 1% longer
than the original fully refined CPU law for that exact curve**. This is a hard
acceptance gate, not a prediction of optimality. Detected internal stops/deep
slowdowns must not increase. Independent continuous velocity/acceleration/jerk
extrema and a fresh scene-bound collision/zone check still run on the CPU.

**Important first-run cost:** enforcing that exact comparison requires a CPU
reference. An unseen curve therefore still runs the old CPU timing calculation
once before its GPU calculation. Exact-input reference caches avoid repeating
this qualification. Logs separate CPU qualification, GPU timing and final checks;
GPU-only benchmarks are not cold end-to-end Calculate times. Left smoothing may
also need its original CPU reference selection. This version prioritizes the
strict 1% guarantee; it does not promise a few-second first calculation for every
new destination. Repeated calculations can reuse the checked GPU mathematics too,
but always check the current scene again.

`GPU AVAILABLE`, `CPU REFERENCE QUALIFICATION`, `GPU OPTIMIZATION START` and
`GPU TIMING READY` identify the phases. If no supported GPU/FP64 driver or build
toolchain is available, the log prominently prints **GPU UNAVAILABLE — CPU TIMING
FALLBACK**, and the result/preview label says **CPU FALLBACK**. llvmpipe/software
Vulkan is never counted as a GPU. Numerical failure or failure of the 1% gate
rejects the GPU result instead of silently falling back. Exact analytic
straight-block timing is explicitly labelled CPU analytic (no nonlinear solve).

The backend builds automatically with the existing `g++`, `glslc`, Vulkan headers
and loader; it installs/upgrades nothing. Compiled binaries/shaders are
content-addressed under `drum_planner/build/vulkan_timing/`. Solver results include
device/driver and implementation identities. One GPU solve at a time is allowed,
including checkpoint jobs; cancellation and the existing aggregate memory cap
remain. New laws/reports are saved as `gpu_timing_law.json` and
`gpu_timing_report.json` beside each plan. No camera or motor access is added.

### Optional checkpoints (experiment modes only)

- The first destination is checkpoint 1. **Add checkpoint** adds another dropdown,
  up to **three destinations total**. **Remove last checkpoint** returns to fewer
  destinations; a single destination retains the original behavior.
- Each dropdown excludes the immediately preceding destination. A → B → A is
  allowed. Changing an earlier selection clears an adjacent duplicate. If there
  are no other saved destinations (currently the left folder has only one), the
  extra dropdown says so; remove it or capture another destination and Refresh.
- Calculation begins when every displayed dropdown has a selection. Changing,
  adding or removing a checkpoint cancels/reaps the old worker and invalidates
  the old full-route preview. All five methods remain available and calculate
  serially; the budget shown is **per leg**, not divided between checkpoints.
  Workers additionally allow up to ten minutes of timing/reference/validation
  overhead per leg, including the independently planned return. This is a wall-time ceiling, not a changed solver setting;
  memory limits and cancellation remain unchanged. Cache reuse avoids repeating
  completed leg searches and timing refinements.
- Every leg is independently generated from the preceding exact endpoint, with
  the same selected method, timing policy, frozen scene, opposite-arm pose, zones,
  models and collision margins. No recorded interior motion is used. The camera
  self-mask stays at the original snapshot pose, never at simulated checkpoints.
- Playback: saved rest → checkpoint 1 → checkpoint 2 → checkpoint 3, with a
  **two-second hold at each selected checkpoint**, then a newly planned path from
  the final checkpoint to the original saved rest. The return does not retrace
  earlier checkpoints or add holds there. It uses the same selected method, full
  collision/zone validation and timing—not an unchecked straight-line shortcut.
  A single-destination preview keeps its original reverse behavior.
  The speed selector scales preview time as before. Scrubbing evaluates each
  checked analytic timing law rather than fitting a new curve across checkpoints.
- A failed leg labels the full route unavailable; it never skips the checkpoint
  or silently plays only a successful prefix. A blocked return also makes the
  full route unavailable; there is no automatic reverse-through-checkpoints fallback. Method selection, changing targets,
  removing checkpoints, Cancel and retry remain available. Completed individual
  leg pools/timing remain reusable by the existing content-checked caches.
- Hardware mode now has separate Calculate-driven destination/checkpoint controls
  and uses Workspace curve. Experiment selection still autoqueues all methods;
  hardware selection never calculates until Calculate is clicked. See `BASELINE_HARDWARE.md`.

### Reusing calculations without reducing quality

Completed saved-input candidate pools, CPU reference and GPU timing results are also mirrored
in a persistent cache, so closing/reopening the program does not require solving
the same destination again. It is shared by the two experiment launchers, but
keys distinguish arms, exact requests/scene/start/goal, geometry, policies, source
code and numerical environment. A changed input is a miss, not an approximate
match. Existing run-local artifacts and provenance remain available.

Storage defaults to `$XDG_CACHE_HOME/openarmx/drum-planner/<project-id>` (normally
`~/.cache/openarmx/drum-planner/<project-id>`), capped at 512 MiB / 128 entries.
Files are hash-checked, publication is atomic, and busy/corrupt/unavailable cache
storage falls back to normal calculation. `X5_PLANNER_CACHE_DIR=off` disables
cross-session reuse; setting it to a directory isolates a benchmark cache.
Terminal logs explicitly report persistent hits. Live Single arc acquisition,
readiness and motor state are **not** restored from this software cache.

The retained CPU reference/fallback calculations avoid constructing Jacobians for line-search trials
that do not request them, hoist invariant polynomial derivatives, and avoid
temporary polynomial wrapper objects. Mathematical operations, solver settings,
refinement levels, grids, collision/zone checks and motion limits are unchanged.
Geometric searches remain serial; CPU reference workers are single-threaded and
memory-bounded. No GPU is required to use the explicit CPU fallback.

Left currently uses **`left_zones/zone1.json`**, the existing left movement
envelope (there is no left zone3). Membership is the same original-URDF gripper
TCP point check with 5 mm tolerance, not a stick-point test. The left tool now
uses **`gripper2left.stl` + `gripper2right.stl`**, not the right-arm claws. This
traditional-grip draft assembles their 14.8 mm bore perpendicular to the wrist
extension. The sleeve is 126.5 mm long; the 14.8 mm-diameter stick projects
**10.25 inches (260.35 mm) beyond the forward face**, with no rear protrusion.
The modeled stick length, including the part inside the sleeve, is 386.85 mm.
Both halves and the stick participate in collision checking and rendering.

Draft mounting: the halves use opposite CAD yaw (0 / 180 degrees), translations
of +/-27.15 mm in hand X, and the nominal hand Z=15 mm mounting plane. Their
bore axes coincide at hand X=0, Z=27.4 mm and point along hand +Y. This clocking
points forward at the custom center and places the tip near the snare at the
saved goal. It is **not a measured registration of the mounting screw holes**;
the user will refine orientation/offset from the preview. CAD base features
are retained intact. The closed assembly represents the existing +14.16 degree
raw motor target (internal coordinate negative); it does not infer slider
travel from that motor angle or change any motor commands. Exact physical
mount alignment remains unverified. The right tool is unchanged, including
its 11-inch exposed stick. New runs regenerate the model; completed candidate
caches are reusable only when the geometry and other inputs match exactly.
Archived plans retain their historical geometry.

The camera-to-base calibration is
shared; the right-arm fitted joint offsets are **not copied onto left joints**.
No calibration or hardware behavior is changed.

To take a new start snapshot for one session (optional, arm must be powered,
stationary and disabled), run:

```bash
./plan_drums.sh experiment --left --capture-start
```

This queries only the selected arm, closes CAN, then runs the ordinary software
experiment. It does not track the arm continuously or overwrite the frozen
default. `--state PATH` remains available for fully offline custom start states.
The saved camera scene is separate, not synchronized to this new pose; explicit
**Recapture scene** remains available when the kit has changed. `--zone PATH`
selects another envelope with matching arm/schema; `--left --zone none` explicitly
omits the point envelope but retains all collision checks.

The standalone `./plan_drums.sh baseline` shares **Workspace curve selection and
the original CPU arm-aware timing policy**, using its own current acquisition and existing
three-window hardware workflow. These experiment modes and their saved artifacts
remain software-only; they do not authorize hardware motion.

## Retained CPU reference/fallback and left corner smoothing

All four original path choices remain, plus the independent Workspace curve fork described below. There is
no timing-method selector. The CPU reference/fallback uses **v1 adaptive fixed-path
retiming** on the right and **v2 bounded C3 corner blending followed
by adaptive timing** on corner-stop left paths; continuous left paths retain v1.
The GPU optimizer then times the exact selected curve as described above.

This addresses the repeated left-arm pauses: preserving a sharp polyline exactly
required stopping at every corner. V2 makes small, local geometric changes and
can overlap blends across short intermediate segments, avoiding artificial
stop/go motion. It does not generate routes from recording interiors.

The original v1 code/policy is preserved. V2 checks a continuous 1-degree L2
joint-space deviation bound, sampled wrist/tip deviations <=10 mm, continuous
joint bounds and the actual new curve's full robot/tool/stick collisions and
zone membership. It then independently rechecks the timed curve. Existing
collision margins and motion limits (0.6 rad/s, 1.2 rad/s², 6 rad/s³) are unchanged.
If a new scene/path cannot be rounded successfully within those bounds, the plan
is explicitly labeled as a v1 fallback, rather than bypassing checks.

For the saved left tasks, Single arc improves from **5.174 to 3.391 s** outward,
and Clearance from **9.051 to 3.473 s**, with **zero detected internal stops**.
The complete four-choice results and record1 comparison are in
`diagnostics/corner_smoothing_20261021/REPORT.md`. These are software one-way
motion durations, not planning compute times or measured physical performance.

The two-second hold and exact return remain. Preview evaluates the analytic new
curve/clock for play, pause, seek, reverse and display speed changes. The other
arm stays static. Plans retain original geometry/timing alongside the new law
and validation. New experiments use the GPU policy above; archived plans keep their
old paths/timing. Candidate-ranking logs deliberately retain the original
reference durations, followed by the new smoothing/timing progress.

Caches are separate from the raw shared candidate pool and hash-verified. Workers
keep the existing memory limits and bounded checkpoint concurrency. All compared methods
remain in `drum_planner/timing/`; see its `README.md`. The original timing study
is in `diagnostics/timing_experiment_20261021/REPORT.md`. The baseline hardware
backend retains the CPU policy; experiment modes gain no motion permissions.

## Four retained candidates plus a Cartesian-first experimental fork

The previous baseline/reversals/coordination/spatial/combined experiment menu has
been replaced. The initial research screened eight approaches, then tested five
compact-route variants; compact corridor generation is now a shared backbone,
not a redundant fifth menu entry.

| Mode | What it does | Trade-off to review |
|---|---|---|
| `progress` — Joint progress | Fresh endpoint-corridor RRT routes, then collision-checked partial-joint shortcuts. Ranks backtracking, workspace detours and overshoot. | Often less motor backtracking; the geometric winner can need a slow corner-stop timing fallback. |
| `sweep` — Broad sweep | Endpoint-preserving elastic deformation of fresh compact routes; emphasizes broad wrist/stick curves. | Can improve the overall sweep while allowing more joint travel. |
| `single_arc` — Single arc | Searches two low-frequency sine components per joint, initialized from compact routes generated for this task, not recordings. Valid continuous arcs have priority over compact proposals. | Smooth broad shape, but may use more motor excursion; continuous compact fallback only when no continuous arc is valid, segmented fallback last. |
| `clearance` — Clearance blend | Ranks the shared generated/refined routes by broad shape plus a soft obstacle-proximity penalty. | Often roomier/simpler wrist routes, but not necessarily closest to the recording or lowest joint travel. |
| `workspace_curve` — Workspace curve | Jointly fits a low-complexity Cartesian cubic Bezier curve and continuous joint posture; secondary stick-tip and joint regularization. Independently generates and checks moderate/broad route alternatives. | A prototype, not a replacement for Single arc. Can produce a much cleaner gripper route, but wrist simplicity alone need not improve tip motion or route length. No legacy fallback hidden under this name. |

The original four modes can honestly select the same route on a particular task. The UI reports
the actual source, and the JSON retains source/fallback provenance. Compact
proposals remain available to those four modes, but Single arc uses them only as a
fallback when no validated continuous arc exists. No deliberately bad path is
introduced just to make previews differ.

### Workspace curve mini-experiment

Select **Workspace curve — Cartesian-first broad sweep** in either arm's method
dropdown. It has a separate proposal pool/budget, so it does not change the
original four generators, rankings, or the hardware `singlearc` mode.

The curve anchor is the calibrated selected `openarmx_<side>_hand` origin at the
gripper base (the existing wrist metric), **not the stick tip**. Zone membership
still uses the original-URDF gripper TCP, exactly as before. A cubic Bezier target
and nine joint knots are fitted together using batched model FK. Seven straight/
broad-bend proposals are explored at two shared route-prior strengths (2 and 5),
the same for every destination and both arms. Joint endpoints are exact. The tip's
second spatial differences and joint smoothness discourage unnecessary tool/arm
motion. A 25 mm fit tolerance is a geometric prototype filter, not a relaxation
of collision margins. The *played quintic curve* is screened at 801 geometry samples against the Bezier
shape, separately from the usual continuous bounds and dense collision/zone checks.

Ranking considers wrist/tip total turning, curvature variation, detour length,
joint backtracking/coordination and the existing soft clearance estimate. Neither
movement duration nor recordings' interior routes enters the ranking. The usual
arm-aware curve preparation and exact analytic playback remain; GPU timing is
experiment-only. `clearance_weight` and
`backtrack_weight` affect this fork's ranking; corridor/seed/overshoot settings
belong to the original shared pool and do not alter its deterministic proposals.
No successful curve means a visible **no feasible route** result, not a crash or
a silently substituted Single arc path. Retry and the other methods stay usable.

The bounded tuning programs, discarded settings, results and comparison are in
`diagnostics/workspace_curve_20261021/`. This is a four-destination viability
study (R1/R2/R3/L1), not proof of generalization to unseen targets. The method
remains available even where the comparison is unfavorable.

## Same UI workflow

1. The saved scene loads once, with no trajectory calculation and the destination
   unselected. **Choose a destination to calculate/cache all five methods**;
   one contained worker at a time. Terminal logs show searches,
   rejected proposals, accepted candidates, scores, selection and cache reuse.
   Changing destination stops playback, clears the old results, cancels/reaps
   the old worker, and restarts for the latest selection. Rapid selection changes
   never run multiple planners or install a result from the previous destination.
   The initial saved start, scene, zone, margins, algorithms, and timing policy
   stay unchanged; checkpoint legs start at the preceding endpoint.
   **Refresh destinations** picks up newly captured files without restarting.
   Empty folders or invalid files show a recoverable message; no default goal is
   silently selected. Each selection freezes a copy of its JSON for reproducibility.
2. Choose a ready method. Switching cached modes does not regenerate or upload
   the depth mesh. An unfinished selection is prioritized next.
3. Play/pause the animation, seek on the timeline, go to start/goal, change
   animation speed, loop, rotate/zoom, switch mesh/points/colors, fit the view or
   save a screenshot. Playback visits all selected checkpoints with a two-second
   hold at each, then follows the independently planned return to the original
   starting pose (single-destination previews retain their original reverse).
4. **Cancel** stops the worker and remaining queue but keeps finished results.
   **Cache all** resumes missing methods. **Generate** reranks the completed
   pool; change the seed to request different proposals.
5. **Gesture settings…** and the pool budget invalidate/rebuild the relevant
   caches. Failed jobs can be retried without restarting the whole UI.
6. **Recapture scene** is an explicit optional camera operation, never automatic.
   A failed recapture retains the previous scene. Successful recapture replaces
   it only after acquisition/initialization completes, then caches the methods
   again if all checkpoints are selected. The selected sequence is retained; the
   arm pose remains saved, not live.

This dropdown changes **only the experiment GUI**. Single arc/hardware modes
retain their existing goal selection. The explicit `--headless` batch workflow
retains its recording-endpoint defaults for existing study scripts.

Default shared-pool budget: **120 seconds**, editable 5–600 seconds. Setup and
finishing verification of an already-started candidate are separate. Default
six compact searches; each uses at most four seconds. If no compact proposal is
valid, up to two unrestricted searches are an explicitly labelled fallback.
A no-feasible-route result is a normal result, not permission to weaken checks.

## Objectives and editable settings

**Single arc continuity priority (shared with `baseline`, for either arm):**

1. Collision/zone-checked `single_arc` proposals with `stop_at_corners=false`.
2. If none exist, checked continuous compact/unrestricted fallback proposals.
3. Only if neither exists, checked segmented proposals that may require stops.

The original broad-arc score ranks **within** these groups, never across them.
This fixes a live-start case where a segmented route won by only 0.81% and forced
four stops despite a valid continuous arc being available. It is a selection
change, not a new speed optimizer or permission to cut corners near obstacles.
The other three methods retain their original ranking. Source-hashed disk caches
are invalidated automatically; existing archived plan files are not modified.
Logs include the group counts and explicitly label any fallback. Saved reports
record `continuous_arc_first_v1`, each candidate's tier and the selected class.

Scoring within each group is geometric, using outward path progress rather than elapsed time.
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
clearance blend. Single arc applies its continuity priority before this score.
There is **no recording-similarity term** in any of these.

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
- Right `zone3.json` / left `zone1.json`: the selected arm's original gripper TCP
  point and legacy 5 mm membership tolerance; neither the opposite arm nor the
  stick defines zone membership.
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
./plan_drums.sh experiment --right --method single_arc
./plan_drums.sh experiment --left --method single_arc
./plan_drums.sh experiment --left --headless --method all --budget 120
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

New experiment requests share Baseline's versioned `occupied_volume_v1` clearance
accounting: full occupied voxel boxes + requested buffer + 5 mm edge allowance,
without adding the voxel radius to the robot a second time. Both arm modes use
the same policy; candidate caches include its identifier. Old saved requests
retain their legacy accounting. Algorithms, path preferences and timing policy
are unchanged. See `diagnostics/start_collision_20261021/REPORT.md`.

## 5 mm camera occupancy (2026-10-22)

New calculations use 5 mm cells instead of 10 mm, without changing Workspace
curve, timing, zones, robot/tool geometry or the 20 mm + 5 mm clearance contract.
Both arms use the same map resolution. New hardware captures already contain
5 mm cells; new saved-input experiments/simulated recalculations reconstruct them
from the original full-resolution depth image in a separate `scene_5mm` folder.
Archived scenes/plans retain their recorded resolution and are never rewritten.
The frozen depth preview mesh and motor controls are unchanged. No camera,
encoder or physical motion was used for this change's verification. See
`diagnostics/voxel_5mm_20261022/REPORT.md` for results and limitations.
## Checkpoint calculation optimization (2026-10-22)

The five path choices and their quality settings remain unchanged. Candidate
searches stay serial; after all checkpoint routes are locked, at most two timing
workers process the independent legs inside one shared 3 GiB calculation job.
Console output identifies each leg and exact timing-cache hits. New scenes still
receive their own collision/zone checks. Timing-only cache entries can cross
sessions/scenes because they contain no collision proof or hardware authority.
The existing cache byte/entry limits and `X5_PLANNER_CACHE_DIR=off` apply.
