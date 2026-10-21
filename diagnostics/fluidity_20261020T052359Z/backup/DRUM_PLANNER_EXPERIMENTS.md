# Software-only trajectory experiments

```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh experiment
```

The robot can be **powered off / disconnected**. The Gemini camera is required for
an initial scene unless `--scene` is supplied. No CAN interface is opened, not
even for encoder queries. There is no Execute button or motor-control path.
Existing `camera`, `live`, `preview`, `plan`, hardware controls and recordings are
not replaced by this mode.

## Workflow

1. The newest successful saved **disabled-arm measurement** supplies both arm
   poses. Its full path is printed. This is **saved, not current/live** feedback.
   `--state FILE` overrides it; a missing state never falls back to querying CAN.
2. Capture one full-FOV native 1280×800 scene using the established five-frame
   temporal confirmation. Stop/release the camera, then build the same display
   mesh as the existing planner. It never refreshes in the background.
3. **All nine methods start calculating automatically** as soon as the initial
   scene is ready. They run sequentially in the background, starting with the
   initially displayed method, and cache results without needing any selection.
   The selector shows queued/calculating/ready/no-feasible-route status, and a
   counter shows progress. Select a ready method to load it immediately; selecting
   an unfinished method moves it next in the queue without aborting the current
   calculation. Cached playback remains available while other methods calculate.
   **Generate** reruns only the selected method once the worker is idle.
4. **Play animation** runs outward → two-second hold → exact reverse to the saved
   start. Pause, seek, goal/start, speed, loop, rotate/zoom, layers, color, fit and
   Save view are available. Only the model moves.
5. **Cancel** stops the current planning process tree and the remaining automatic
   queue, keeping completed cached results. **Cache all** resumes missing methods.
   No partial path becomes playable. Changing uncertainty settings or the budget
   rebuilds the automatic queue. These controls and **Recapture scene** are enabled
   when idle (use Cancel first if necessary). Recapture explicitly opens/releases
   the camera; a successfully installed new scene invalidates old trajectories
   and automatically queues all methods against the new scene.

Progress goes to the launching terminal and each run's `worker.log`. Default
search/optimization budget is **60 s**, adjustable from 5–600 s. Model loading,
first-time compilation, completed-operation final checks and file saving are
separate; a hard worker deadline of budget + 60 s prevents a stuck solver from
running forever. A solver can return **no feasible route**, especially with large
uncertainty assumptions or a tight budget. That is a result, not permission to
weaken collisions. Failed/timed-out jobs cannot be played.

**No recorded motion is used as a seed.** Only the final seven joint angles of
`recordings/record3.json` supply the goal. All intermediate routes are generated
from the model, frozen scene and endpoints. No demonstration guidance, preference
learning or manually placed scene obstacles are used.

## Methods and honest implementation boundaries

### Right-arm zone3 constraint

Every newly generated plan (including all nine methods) must keep the **single
original `openarmx_right_hand_tcp` gripper point** inside `right_zones/zone3.json`,
using exactly the existing recorder/playback membership calculation:

* Rebuild the convex hull from the recorded points with `Zone.load`.
* Use the unchanged `model/openarmx.urdf` and saved right-joint coordinates for
  this point, **not** the camera-corrected collision-model kinematics.
* Accept when every signed plane distance is at most the existing **5 mm outward
  tolerance plus 1e-9 m numerical tolerance**. No new expansion or zone edits.
* The left arm, custom gripper surface and drumstick do not need to fit inside
  the zone. Full-arm/stick obstacle and self-collision checks still apply separately.

OMPL proposals and edges, IRIS collision witnesses, scenario checks and the final
densely sampled timed curve use the same predicate. Native CHOMP/STOMP proposals
are also rejected if their final curve leaves it. The goal hold and exact reverse
use the same constrained path. An outside start/goal produces no feasible route;
there is no unchecked entry movement or clipping to the zone.

The zone/reference-model hashes are stored in requests, results and the session
cache signature. Restart the experiment after editing zone3; a running session
refuses mismatched inputs. Historical saved previews are not retroactively
replanned. Zone membership remains a TCP workspace condition, not whole-arm
containment or a substitute for collision checking.

| Selector | Implementation |
|---|---|
| Baseline | Existing `planning.best_plan`: independent OMPL seeds, 40/30/20/10 route score, fastest-valid normalization and 2× duration cap. |
| Clearance first | Successive extra-clearance constrained searches with FCL, followed by lexicographic bottleneck/low-decile clearance, exposure, length and duration ranking. This is a finite clearance-ladder search, not a claim of globally maximal clearance. |
| Robust | Bounded world displacement envelope, moving-joint bound checks and self-clearance envelope; new routes generated against those constraints. No physical MPC controller. |
| Chance constraint | New RRT candidates plus a native-CHOMP broad-route proposal, correlated whole-path uncertainty scenarios, nominal FCL validation, then an empirical chance filter using a one-sided Wilson bound with a 95% Bonferroni candidate adjustment. This is a scenario-based adaptation, not a literal reproduction of the full MCMP paper. |
| CVaR | Same RRT/CHOMP from-scratch candidate mechanism, but ranks the empirical upper tail of scenario losses instead of mean proximity or speed. Does not impose the chance mode's probability threshold. |
| Diverse route families | Independent and increased-clearance searches plus random feasible configuration-space waypoint searches; clusters joint-arc routes by normalized RMS distance and ranks clearance first. It does **not** prove different mathematical homotopy classes. |
| IRIS + GCS | Genuine Drake IRIS operating on iteratively sampled **full-robot FCL C-space collision point witnesses**, with cells shrunk 3% toward their free seeds, then `GcsTrajectoryOptimization` with Clarabel convex relaxation and rounding. Rejected GCS curves feed new witnesses into affected regions. Regions are seeded by fresh geometry-planned paths, including a native CHOMP proposal, never a recording. Regions remain sampled approximations, not certified continuous free sets. Not a tip-only workspace corridor. |
| CHOMP | Installed MoveIt CHOMP and its native distance-field/covariant optimizer, initialized from a newly planned path. Final validation is the unchanged full-body FCL checker, not CHOMP's approximate distance field alone. |
| STOMP | Installed MoveIt STOMP plugin and actual stochastic trajectory optimizer, initialized from a newly planned path. Its output also undergoes independent dense FCL validation. |

Transit comfort excludes the small configuration-space neighborhoods immediately
around the given endpoints. Additional search comfort tapers there because the
goal intentionally approaches the ride cymbal. **Original hard collision padding,
full stick, self-collision checks and joint limits never taper or disappear.**
Soft clearance is a distal FCL padding-band estimate (5 mm bands, capped at 160 mm),
not an exact Euclidean distance certificate. A short path cannot compensate for a
worse minimum-clearance tier in clearance-first ranking.

All selectable successful results are retimed with the established quintic
interpolation and analytic velocity/acceleration/jerk extrema. The final curve is
checked at ≤2 ms plus ≤0.003 rad joint-L1 edge subdivisions. All original moving
collision bodies and the complete custom stick remain in those checks. No camera
or robot geometry is simplified to make the algorithms run faster.

CHOMP checks stationary left-joint bounds with zero measurement tolerance. Its
isolated optimizer model (also used by STOMP) applies only the original checker's
already accepted three-encoder-count static-left allowance, retaining the exact
measured angles/FK. It does not clip measurements or change any moving right-arm
limit. Independent final validation always uses the original model/checker.

## Editable experimental uncertainty assumptions

**Uncertainty assumptions…** offers Small, Moderate and Large presets and editable
fields. They are **bounds**, not measured standard deviations or calibrated failure
probabilities. Defaults are moderate: depth 3 mm; camera translation 2 mm; camera
rotation 0.05°; per-joint deviation 0.05°; tool size/mount 2 mm and 0.25°; scene
motion 2 mm. These allowances are **additional to** the existing nominal world
buffer. Zero is supported for explicit nominal-only comparisons, not presented as
proof that physical errors are zero.

* Robust combines isotropic displacement bounds, using a conservative 1.5 m
  lever per joint and 5 m camera range. It can reject the resting endpoint before
  route search; widening the route cannot fix an endpoint outside the envelope.
* Scenarios apply bounded camera SE(3) shifts, surface/scene inflation, stick
  inflation, constant joint bias plus smooth bounded tracking deviations.
  Each scenario is correlated across the entire path, not independent jitter
  at every time step. These are **hypothetical computations**, never commands.
* Default: 64 scenarios, chance limit 0.15, CVaR tail level 0.90. A scenario fails
  if any of its 101 arc-sampled poses violates full-body FCL/bounds. Loss is
  `10 × any failure + fraction of poses inside an extra 20 mm comfort band`.
  CVaR uses exact fractional empirical tail mass, not just the worst sample.
* Statistical bounds are approximate Wilson intervals with a Bonferroni candidate-count adjustment and concern this assumed sampled model only. They are not
  continuous-path or real hardware safety probabilities. Missing surfaces,
  humans entering later, or an incorrectly saved pose are not fixed by margins.
* The camera calibration is reused, not rerun. Presets describe possible residual
  error; they do not assert that the encoders/calibration are defective.

## Reproducible/offline runs

```bash
./plan_drums.sh experiment --list-methods
./plan_drums.sh experiment --scene /absolute/path/camera/scene.json --state /absolute/path/measured_start.json
./plan_drums.sh experiment --headless --method all --budget 60
./plan_drums.sh experiment --headless --method cvar --settings assumptions.json --scene /absolute/path/camera/scene.json
```

`--method` accepts `baseline`, `clearance`, `robust`, `chance`, `cvar`, `families`,
`iris_gcs`, `chomp`, `stomp`. `all` is headless-only. Settings JSON may override
individual fields (see `drum_planner/experiment/config.py`). `--output` must name
a new folder; the default is `planner_runs/<timestamp>-experiment-<id>`.

Outputs: `context.json` (hash-bound scene/state), initial capture, generated model,
per-method settings, raw generated candidates, reports, `plan.json`, logs and
`containment.json`. The GUI caches only the latest result per method; older runs
remain on disk for comparison. A saved successful result can also be inspected
with `./plan_drums.sh planned-view PATH/plan.json --autoplay`.

## Dependencies and resource containment

```bash
./setup_planner_experiment.sh
```

Python dependencies live in `.venv-planner-experiment`, inheriting system ROS,
NumPy, SciPy, Qt and VTK. Drake is pinned in `experiment/requirements.txt`; GCS uses
**Clarabel**, not a paid solver. Native MoveIt CHOMP/STOMP are already installed.
The existing system Python/default launcher are not replaced.

One experiment job runs at a time. Build, capture and planner workers run in
independent systemd user **services**, not the VS Code application cgroup:
3 GiB RAM max, 128 MiB swap max, whole-process-tree cancellation, bounded thread
count and hard wall timeout. Kernel address-family restriction excludes CAN;
Python also installs a CAN-denying audit hook. Scenario batches, logs/history and
candidate counts are bounded; unused worker worlds die when the method finishes.
The native GPU scene and robot meshes remain resident and unchanged while
rotating/playing; a method switch updates actor transforms, not triangles.

## Algorithm references

* [MoveIt CHOMP](https://moveit.picknik.ai/main/doc/how_to_guides/chomp_planner/chomp_planner_tutorial.html)
* [MoveIt STOMP](https://moveit.picknik.ai/main/doc/how_to_guides/stomp_planner/stomp_planner.html)
* [Drake IRIS](https://drake.mit.edu/pydrake/pydrake.geometry.optimization.html)
* [Drake GCS trajectory optimization](https://drake.mit.edu/pydrake/pydrake.planning.html)
* [GCS motion planning paper](https://arxiv.org/abs/2205.04422)
* [Monte Carlo motion planning](https://arxiv.org/abs/1504.08053)
* [CVaR risk-averse planning](https://arxiv.org/abs/2011.01578)

These experiments compare planning behavior; none are physical collision
certifications or authorization to energize the robot.

### GCS memory and graph detail

The IRIS adapter uses point collision witnesses, with each computed cell pulled
3% inward toward its free seed. It retains seed-chain connections and three
nearest-cell neighbor proposals, then adds only intersecting-region edges. This
bounded sparse graph avoids an all-pairs conic program; optimality is only relative
to the resulting finite graph/rounding, not the entire robot configuration space.
It does not reduce robot, camera, collision or display geometry fidelity.

Scenario comparisons share scene/padding work across candidates while preserving
every scenario and collision query. Their perturbation field is indexed by
outward path progress; the reverse uses that same static field in reverse. This
is not a claim about independently changing return-tracking errors or a moving
scene.

## Verification on this computer (2026-10-19)

Evidence is in `diagnostics/planner_experiment_20261019T073849Z/`:

* `final_all_results.json`: all nine modes ran against the actual Gemini snapshot.
  Eight default modes produced validated trajectories. Moderate robust bounds
  rejected the resting endpoint normally; an explicit zero-error robust control
  also generated and played a valid route. Defaults were not weakened to pass.
* `independent_verification.json`: successful curves rechecked with the original
  file-based MoveIt/FCL backend, continuous joint/derivative bounds and exact
  start/goal/hold/reverse semantics.
* `ui_full.json`, `ui_resilience.json`: each successful method played a complete
  cycle; tested selection/cache, pause/resume, seek, speed, loop, goal/start,
  render controls, settings, recapture, cancellation and worker-failure handling.
* `real_cli_complete/experiment_view_report.json`: normal CLI launched with a
  fresh camera capture, stayed open for 120 seconds, completed playback, rendered
  without errors and uploaded the frozen mesh only once.
* `planner_regression_final.log`: 328 tests passed;
  `calibration_regression.log`: 110 tests passed.
* `preservation_audit.json`: existing recordings, calibration, original model,
  hardware paths and launchers retained. Only the existing CLI and CMake entry
  points changed among the 43 initially hashed files.

This verifies software behavior on the tested scenes, not future scene
feasibility or physical robot execution. No motor commands or CAN queries were
used during these experiments.

### Automatic-cache follow-up (2026-10-19)

`diagnostics/experiment_autocache_20261019T170321Z/` records a fresh-camera UI test:
all nine methods calculated/cached without a single method selection, playback
completed while other methods were running, and cached selection caused no new
generation. Cancel/resume, settings/budget invalidation and explicit camera
recapture/requeue also passed. The frozen mesh was uploaded once during the batch;
all workers retained the 3 GiB cap. `regression.log` records 337 passing planner
tests, including nine added queue/controller tests. A cached no-feasible-route
result remains distinct from a playable trajectory.
