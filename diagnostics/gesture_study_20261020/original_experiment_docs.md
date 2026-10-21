# Software-only fluidity experiments

```bash
cd /home/jason/Proyectos3/X5
./plan_drums.sh experiment
```

The robot may be **powered off / disconnected**. This mode never opens CAN, even
for encoder queries. It uses the latest successful saved relaxed-arm measurement
(the full path is printed), and the final joint pose of `recordings/record3.json`.
**No interior recorded poses are used.** Hardware baseline playback is unchanged.

## Same UI and workflow

1. Capture the initial full-FOV native Gemini 1280×800 scene using the established
   five-frame confirmation, then stop/release the camera. No background refresh.
   `--scene FILE` reuses a saved scene; `--state FILE` overrides the saved pose.
2. Automatically generate a **shared candidate pool**, then calculate/cache all
   five rankings. No selection is needed to start. The selector shows progress;
   full search, rejection, score and winner logs go to the terminal/worker log.
3. Select a ready method to display it instantly. An unfinished selection moves
   that ranking next in the queue without aborting the running job. Cached
   playback stays available while other rankings finish.
4. **Play animation** runs outward → two-second goal hold → exact reverse to the
   saved start. Pause, timeline seek, goal/start, speed, loop, rotate/zoom, layers,
   color, fit and Save view remain available. Only the model moves.
5. **Cancel** stops the current worker and remaining queue, retaining completed
   results. **Cache all** resumes missing results. A partial pool is never reused.
   **Generate** reranks the selected method using the same completed pool. To
   compare new random candidates, change the seed in **Fluidity settings…**.
6. **Fluidity settings…**, pool budget and **Recapture scene** are available when
   idle (Cancel first if needed). Changes invalidate affected UI caches and queue
   all five methods again. Recapture replaces the frozen scene only after complete
   successful acquisition, initialization and mesh loading. Camera remains off
   otherwise. Failed jobs can be retried without restarting the program.

The previous risk/uncertainty/GCS/CHOMP/STOMP experiment modes have been removed.
The old uncertainty dialog is replaced with relevant fluidity controls in the
same place. Historical output folders are not deleted or rewritten.

## Five methods

All are **ranking objectives**, not five independent planners. They see the exact
same fully checked, timed trajectories. Therefore selecting a different method
cannot silently cause a different random search or different collision margins.

| ID | Preference |
|---|---|
| `baseline` | Existing 40% workspace economy, 30% broad bending, 20% obstacle proximity, 10% joint economy. Retains the existing fastest-valid reference normalization and duration tie-break, but **not** the 2× duration eligibility cutoff. |
| `reversals` | Fewer meaningful direction reversals across the seven motors. Hysteresis ignores excursions smaller than the editable threshold. Score = reversal count + excess joint travel / 30 degrees. The excess travel is measured from hysteresis-filtered extrema, not raw sample noise. |
| `coordination` | Gradually varying direction of seven-joint progress. Score = total joint-space turning (radians) + square root of dimensionless joint-space bending energy. A straight joint-space path scores zero even if its motors move different amounts or signs; shoulder-then-elbow paths bend and score worse. |
| `spatial` | Simple wrist and stick-tip curves. For each: dimensionless bending energy + 0.25 × dimensionless curvature-vector variation + excess length ratio. Scores are summed for wrist and tip. Broad curves are preferred to concentrated/repeated turns and detours. |
| `combined` | Equal-weight reversal, coordination and spatial scores, normalized by each metric’s shared-pool median (floored at 1). This normalization and tie-break are independent of duration. See the per-run report for the exact normalization constants. |

The three new metrics use uniform **outward joint-arc progress**, never elapsed
motion time. Goal hold and reverse are excluded. Joint curves are then evaluated
at a fixed angular scale, wrist/stick curves at a fixed physical spatial scale.
Adding duplicate time samples or slowing execution does not remove geometric
turns. These are engineering hypotheses about the user's preference, **not** a
claim that human movement follows a universal score. No demonstrations or
preference learning are involved. These metrics do not replace collision checks.

`Fluidity settings…` offers Fine detail, Balanced and Broad gestures presets:

| Setting | Balanced default | Range |
|---|---:|---:|
| Meaningful reversal excursion | 2 degrees | 0.1–10 degrees |
| Joint-curve sampling scale | 0.05 rad | 0.01–0.2 rad |
| Wrist/stick sampling scale | 0.05 m | 0.01–0.1 m |
| Independent geometric searches | 6 | 2–24 |
| Random seed | 19 | 1–2147483647 |

These scales affect **soft scoring only**. They do not thin the collision model,
change zone3, reduce the rendered mesh, or change verification resolution.

## Shared pool and motion checks

The pool uses independent from-scratch RRTConnect proposals through the existing
MoveIt/FCL model. Each proposal tries the existing smooth quintic and corner-stop
quintic timing variants, with the original path as a fallback if compression fails.
Only independently checked variants are admitted. Every mode uses the same pool
key and candidate hashes, reported in its `experiment_report.json`.

There is **no 2× fastest-path duration filter in this experiment**, for any method.
The established absolute 120-second outward limit, 0.6 rad/s velocity, 1.2 rad/s²
acceleration and 6 rad/s³ jerk limits remain. Continuous polynomial extrema check
joint/derivative bounds; the final curve gets full-body/stick collision and zone
checks at ≤2 ms plus ≤0.003 rad joint-L1 edge subdivisions. No bad candidate can
be accepted because its fluidity score is good. A rejected proposal is normal
search progress, not an application fault.

Right zone3 uses **only the original `openarmx_right_hand_tcp` point**, original
`model/openarmx.urdf` FK, and the existing `Zone.contains` convention: signed hull
plane distance ≤5 mm + 1e-9 m. Neither left arm nor stick must fit inside the zone.
Full arm, custom right gripper and stick collision checks remain separate.

Search budget is **60 seconds per shared pool** by default, adjustable 5–600.
An already-started candidate finishes verification even at the search deadline.
Setup/finalization are separate, with an outer budget+60-second worker deadline.
The remaining four rankings reuse the pool instead of repeating the searches.
A no-feasible-route result is cached normally; it does not authorize relaxed
checks. Separate modes may choose the same candidate; this is an honest result,
not a reason to fabricate different trajectories.

## Commands and outputs

```bash
./plan_drums.sh experiment --list-methods
./plan_drums.sh experiment --headless --method all --budget 60
./plan_drums.sh experiment --method coordination
./plan_drums.sh experiment --scene /absolute/path/camera/scene.json --state /absolute/path/measured_start.json
./plan_drums.sh experiment --settings fluidity_settings.json
```

`--method` accepts `baseline`, `reversals`, `coordination`, `spatial`, `combined`.
`all` is headless-only. Settings JSON accepts partial overrides of the five fields
in `config.py`; old uncertainty fields are rejected with an explanation. `--output`
must name a new directory. Default output: `planner_runs/<timestamp>-experiment-<id>`.

Outputs include `context.json`, camera snapshot, generated model, per-method
settings/plan/report, `shared_pools/<hash>/pool.json`, raw and validated candidates,
logs and resource-containment reports. Pool identities include state, scene,
calibration, model, zone, endpoint, settings, budget and relevant source hashes.
Cancelled/incomplete, modified or incompatible pools are not silently accepted.
All methods report scores, candidate IDs, metrics and normalization constants.
Saved successful plans also work with `./plan_drums.sh planned-view PATH/plan.json`.

## Memory, rendering and dependencies

`./setup_planner_experiment.sh` creates the project-local environment, inheriting
system ROS, NumPy, SciPy, Qt and VTK. No Drake or other extra pip package is now
required; previously installed packages are not uninstalled from this computer.

One worker runs at a time in a separate systemd user service: 3 GiB RAM maximum,
128 MiB swap maximum, bounded thread count, hard timeout and whole-process-tree
cancellation. CAN is blocked by OS address-family restriction and a Python audit
hook. Fine validation arrays are temporary; candidate payloads live on disk and
only the selected trajectory is loaded by a ranking worker. Completed pools are
shared, including their expensive collision/FK results. Logs/history are bounded.
The complete frozen GPU mesh stays resident; switching methods updates robot
transforms without rebuilding scene geometry.

Software validation is not physical playback verification. These experiment
outputs cannot energize the robot and do not certify obstacles outside camera view.

## Verification of this replacement (2026-10-19 local time)

Evidence: `diagnostics/fluidity_20261020T052359Z/verification.json`.

* 578 planner regression tests passed, including the five-method metrics, shared
  cache integrity, queue/cancel/retry behavior and unchanged hardware-mode mocks.
* A real Gemini capture produced eight checked candidates; all five rankings
  used identical pool/candidate hashes. Winners were independently rechecked
  through the original file-based MoveIt/FCL backend, including joint polynomial
  extrema, motion limits, zone3, exact endpoints and hold/reverse semantics.
* Every method played a complete outward/hold/return animation in the visible
  native UI. Pause/resume, seek, goal/start, all speeds, loop, cached selection,
  rendering layers/colors/navigation, Save view, settings/presets, cancel/resume,
  budget invalidation, Generate and actual camera recapture/requeue passed.
* The normal CLI with a fresh camera capture stayed open for 90 seconds, cached
  all five modes, completed an autoplay cycle and reported zero render errors.
  Its full frozen scene was uploaded once; there were no background camera updates.
* Workers retained the 3 GiB memory cap. Largest sampled planner-service memory
  in this run was about 354 MiB (this is sampled cgroup accounting, not a universal
  peak guarantee). Shared pools avoid repeating searches/FCL loading for rankings.
* Protected recordings, original model, zone3, hardware baseline sources,
  `planning.py`, `style.py`, the production C++ checker and launchers were unchanged.

All tests were software-only. No encoder queries or physical movement occurred.
