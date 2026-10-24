# Working on X5

This file gives a new agent the project's working conventions without requiring
the previous chat. Follow the current user's scope and authorization; historical
permission to move hardware does **not** carry into another task. Sections 1–5
describe durable preferences. Section 6 is an implementation snapshot verified
on **2026-10-22**: check source before treating those details as current.


## 2026-10-24 experiment-only update (supersedes older experiment snapshot below)

The user approved a speed-first overhaul of `plan_drums.sh experiment` for both
arms. Its visible menu now has one GPU fast workspace pipeline. See
`DRUM_PLANNER_EXPERIMENTS.md`, `experiment/{fast,fast_path,fast_config}.py` and
`timing/vulkan/fast.py`. No CPU reference or planning-result cache is used by
this default. Approximately 10% extra movement duration is an offline target,
not a strict runtime comparison gate. Full limits and final scene checks remain.
The original five-method implementations, CPU policies, hardware generator and
hardware dispatch are retained unchanged; older statements about five visible
experiment choices, two timing children or strict 1% qualification are historical.
Do not accidentally route hardware through the new experiment-only pipeline.

## 1. How to work with this user

- **Respect discussion versus implementation.** “Don't change code,” “brainstorm,”
  and “diagnose first” mean inspect/explain only. “Don't begin anything yet” means
  wait. When asked for a plan/questions, resolve consequential uncertainties
  together; after approval, implement without repeatedly reopening settled choices.
- **Prefer the smallest proven adaptation.** Reuse working playback, joint/sign
  conversions, camera acquisition, and planners. A request to play a calculated
  recording is not an invitation to build another control framework. Expand the
  architecture only for a demonstrated need.
- Preserve unrelated behavior, source, recordings, calibration captures, and
  accepted models. A left-arm adaptation must not silently change the right arm;
  a timing experiment must not silently change path generation.
- Diagnose before blaming hardware, encoders, calibration data, or the user.
  Reproduce the specific failure and distinguish evidence from hypotheses. A
  plausible simulation is not physical validation; a failed solver does not
  automatically mean the collected data must be discarded.
- Explain plainly, with exact commands/paths and useful numbers. Stream progress
  for long calculations. Avoid repetitive safety lectures and speculative fixes.
- Complete the authorized scope and requested verification. Do not claim success
  after merely writing code. Report an actual blocker promptly rather than
  fabricating a pass, silently waiting indefinitely, or claiming all bugs impossible.
- Overnight permission means persistent, purposeful investigation—not exhaustive
  brute force or endless work on clearly unpromising approaches.

## 2. Experiments are the preferred way to resolve algorithm choices

1. **Define the objective and comparison first.** When several materially different
   approaches are plausible, test a reasonable bounded set. For a small tuning
   question, use a smaller study rather than rebuilding the experiment application.
2. **Control inputs.** Hold scene, start, endpoint, robot/tool geometry, zone,
   collision margins, motion limits, and seed policy consistent where applicable.
   Save settings, provenance, results, runtime, and enough information to reproduce
   the comparison. Do not compare different starts and call it an algorithm effect.
3. **Separate path generation/ranking, timing, perception, and execution.** Change
   only the requested dimension. If corner smoothing is authorized to alter a
   path, bound the alteration and recheck the actual resulting curve, not just its
   original waypoints. Keep the other arm's approved behavior unchanged.
4. **Measure the requested quality, not an easy proxy.** “Humanlike” or “fluid” here
   means a coherent broad movement: fewer joint reversals, less backtracking and
   unnecessary bending, coordinated joints, no arbitrary internal stops, and room
   around obstacles rather than threading narrowly between drums. Fastest or
   shortest alone does not establish this. Use relevant geometric, clearance,
   speed-profile, and duration measurements; explain their limitations.
5. **Recordings are benchmarks, not hidden route templates.** The final pose of
   `recordings/record3.json` or `left_recordings/record1.json` may supply the goal.
   Interior recorded motion may be used for post-generation comparison, but must
   not seed, constrain, copy, or teach the generated route unless explicitly
   authorized. Similarity to one recording is not proof of generalization or safety.
6. Rule out clearly worse approaches using both objective features and benchmark
   comparisons. Retain meaningful finalists for visual judgment when requested;
   do not force a single winner from subjective near-ties. Conversely, when the
   user explicitly requests one objective winner, use that criterion and report
   per-case results (for example, timing across two paths and both arms).
7. Check modest variations of start/goal and both arms when studying generalization.
   Single arc and clearance bend/blend were visually preferred; that is useful
   evidence, not a universal guarantee. Do not add preference learning unasked.
8. Preserve requested historical implementations and policies for comparison and
   rollback. Replacing visible experiment choices does not require deleting raw
   data or every old implementation. Avoid accumulating obsolete visible modes.
9. Keep experiments software-only by default. Use saved scenes/poses, serialized geometric
   searches, bounded timing workers, cached results, and terminal progress. Justify GPU work by measurement,
   not assumption. Stop tuning when further gains are not meaningful.

## 3. Safety: keep essential behavior, stop inventing workflow barriers

Previous versions repeatedly failed because speculative restrictions were layered
on top of the actual task. **Do not repeat these mistakes:**

| Previous mistake | Required approach |
| --- | --- |
| Rejecting motion because the shoulder or resting arm is outside camera view / minimum depth | Preserve the approved observed-obstacle policy. Do not require every robot part to remain visible or silently treat all unknown space as occupied. Unseen space is not certified clear, either. |
| Pausing right-arm motion because the relaxed left arm swayed, or vice versa | Passive-arm sway is not a stillness veto. Retain its measured geometry and genuine motor-fault handling; do not command it. |
| Adding stricter hardware margins than the approved planner | Keep the agreed margins unless a change is requested or justified with concrete evidence. Do not double-count occupied voxel size as additional clearance. |
| Adding setup reviews, typed phrases, duplicate readiness steps, arbitrary capture expiry, or mandatory soak tests to every run | Keep actions direct. Do the requested validation, not a growing checklist. A user clicking Start authorizes that run; do not add a second ceremony or queue surprise motion. |
| Graying out Hold/Reverse after a fault, leaving power-off/restart as the only escape | Keep recovery controls reachable. Explain unmet prerequisites and allow retry/reconnect in the same workspace. Clickable does not mean blindly moving with unusable feedback. |
| Returning through a historical custom center regardless of the current task | Emergency recovery reverses the traveled prefix to the measured rest; normal multi-checkpoint return is independently planned from the final checkpoint. `start_beat.sh` centering is a separate workflow, not a universal recovery rule. |
| Treating optimizer-induced feedback starvation as a new mechanical hazard | Fix resource/process isolation and the actual feedback failure; do not add gates or hide it by weakening freshness checks. |

For a proposed new blocking check, identify the concrete failure it prevents,
evidence for the threshold, and the recovery route. Prefer fixing the root cause
over expanding the state machine. Do not suppress real faults, mask obstacles,
clip encoder poses, or weaken limits simply to make a test pass.

### Non-negotiable hardware distinctions

- An arm being powered on is **not** permission for the agent to move it. Obtain
  current explicit permission for physical tests. “No movement” also excludes
  unsolicited enable/disable, centering, gripper setup, and torque-hold commands.
  Read-only encoder/camera tests must be identified separately from motion.
- Software-only tests use fake transport/saved inputs. Access actual cameras or
  encoder queries only when part of the agreed test; never launch a hardware
  execution path as an ordinary software smoke test.
- Control only the selected arm. The opposite stays relaxed/query-only: no enable,
  hold, open, or disable commands. Enforce this at the transport boundary, not
  merely in displayed joint targets. Preserve established bus/sign conventions.
- **Never automatically relax an arm suspended in midair after an ordinary fault,
  cancellation, or window closure.** Preserve motor ownership/HOLD when possible;
  give the operator Reverse to rest + relax. Return the traveled prefix, settle at
  the supported original rest, then relax. Do not substitute an unplanned shortcut.
- A stall, thermal fault, lost power, or unusable feedback can make holding or
  reversing impossible or harmful. Report that clearly and request necessary
  physical intervention; do not promise recovery software cannot deliver or
  blindly increase drive. Do not turn every ordinary error into torque-off.
- Custom grippers have fixed sticks/rubber-band constraints. Use the existing
  customized closed targets; never add automatic opening sweeps or generic homing.
  Include the custom gripper and drumstick in collision geometry.

## 4. UI and performance expectations

- Keep preview, measured hardware state, and commands distinct. The current design
  is **three windows**: disconnected trajectory preview (play/pause, scrub, speed),
  measured live-arm view, and control panel. Both views share the frozen camera scene.
- Moving a real arm updates the live view, not the disconnected animation. Movement
  of the selected arm after Calculate invalidates that plan with a useful error;
  Calculate can retry without restarting. Do not apply this rule to passive sway.
- Buttons should do what they say. Avoid needless modals, typed confirmations,
  overlapping widgets, duplicate render windows, and disabled-control dead ends.
  Busy/rejected actions must not create another motor owner or pending surprise Start.
- Preview speed changes during a physical run must not silently change that run.
  Show errors and recovery options while preserving the existing controller's state.
- Production hardware Workspace curve calculates on **Calculate**, not startup. Experiment GUI
  starts with no destination selected and no trajectory calculation. Selecting a
  destination queues all methods using bounded serial work; changing destination
  cancels the old work and restarts for the new pose. Do not conflate these workflows.
  Experiments optionally accept up to three ordered checkpoints (first destination
  included), with two-second holds. After the final checkpoint, independently plan
  a return to the original saved start, without retracing earlier checkpoints.
  Single-destination preview retains its original reverse behavior.
  Later legs start at the preceding goal; their perception self-mask stays at the
  original saved snapshot pose. Hardware mode also accepts these checkpoints, but selection only invalidates the
  old plan; it never calculates until Calculate is clicked. Normal multi-checkpoint
  playback uses the independently checked return. Emergency recovery reverses the
  traveled prefix. Holds remain two real seconds at any speed.
- Capture a scene once per acquisition and keep it static for that plan/preview;
  this is not live obstacle avoidance. Show robot URDF/custom-tool visuals normally,
  separately from the camera mesh. Do not mesh the robot into disconnected triangles.
- Optimize retained geometry, actor transforms, redraws, and data copies before
  reducing visible quality. Avoid rebuilding static meshes on every UI frame.
- Earlier work exhausted resources and disrupted the editor. Keep one bounded heavy
  calculation group, bounded caches/logs, cancellation and process cleanup. Preserve
  existing worker memory limits. Planning must not starve encoder feedback: native
  optimizers can hold Python's GIL, so computation belongs in a separate process.
  Never kill a holding motor owner while cleaning up disposable planning workers.
- Saved-input experiment pools/timing can persist across launches via
  `experiment/cache.py`; keep exact request/code/model identities, checksums and
  bounded storage. Never reuse experiment acquisition/readiness as live evidence.
  Performance changes must retain timing policy and numerical-output regression
  comparisons against `diagnostics/planner_performance_20261021/reference/` and
  `diagnostics/planner_optimization_20261022/reference/`. Checkpoint routes are
  now all selected serially before at most two timing children start, under the
  same aggregate 3 GiB job cap. Never parallelize time-budgeted searches casually.
  `timing/math_cache.py` may reuse exact timing mathematics across live scenes;
  this is NOT permission to reuse a scene collision proof or hardware readiness.

## 5. Verification and repository discipline

- Confirm `/home/jason/Proyectos3/X5` is the intended checkout; older copies exist.
  Read `git status` first. Preserve user changes and untracked models/data. Do not
  reset, stage everything, push, rewrite history, or clean artifacts without scope.
- Reproduce the reported failure through the actual requested mode. Camera preview
  alone does not test calibration capture plus CAN; mocked playback does not prove
  physical motion. A device name is not proof of usable frames or the selected FOV.
- Test the relevant regressions, normal workflow, and failure/retry/recovery paths.
  For UI work, exercise actual windows and controls, not only import tests. For
  arm-specific changes, test both side routing and preservation of the other side.
- If asked for N seconds of actual-camera operation, run that mode for N seconds
  and inspect errors. Do not substitute fake inputs or old logs. Conversely, do not
  impose a minute-long hardware test the user explicitly skipped.
- State what passed: syntax/unit tests, simulated integration, native UI, read-only
  hardware, or authorized motion. Never present historical test counts as a fresh
  run. Tests should reproduce bugs, not merely change assertions to hide them.
- Keep code, UI text, documentation, cache versions, and tests consistent. Cache
  identity must include relevant arm, inputs, model/calibration, and algorithm/policy.
  Investigate different inputs/selection/timing before “fixing” experiment/production
  parity with a separate implementation.
- Preserve raw recordings and calibration captures. Derived corrections belong in
  traceable outputs; recollect only when evidence shows the inputs are unusable.
- Keep Git artifacts curated: consult `config/git_retained_artifacts.json` and
  `scripts/git_artifacts.py`. Do not commit bulk caches, runtime IPC credentials,
  or huge generated runs. Respect existing notification setup; alerts remain generic,
  without prompts, transcripts, or secrets. Do not modify unrelated user tooling.

## 6. Current project map — verify against source when changing behavior

```bash
./plan_drums.sh --hardware --right   # Hardware-capable UI; no automatic motion
./plan_drums.sh --hardware --left
./plan_drums.sh --hardware-sparse --right # Optional sparse IPOPT timing fork
./plan_drums.sh --hardware-sparse --left
./plan_drums.sh experiment --right # Saved-input software experiment
./plan_drums.sh experiment --left
./plan_drums.sh --hardware --right --simulate PATH_TO_RIGHT_PLAN
./plan_drums.sh --hardware --left --simulate PATH_TO_LEFT_PLAN
```

`--hardware` defaults to right; `singlearc` and `baseline` remain compatibility aliases.
All three now use Workspace curve, not Single arc. Internal
`drum_planner/baseline/` names and recovery protocols intentionally remain. Do not
rename them casually. `experiment.sh` is a separate older program.

`--hardware-sparse` is a separate opt-in hardware fork (see `SPARSE_HARDWARE.md`).
It changes only the fixed-path timing solver to sparse IPOPT/CasADi, not the path
generator, UI, limits, collision checks or motor control. Original `--hardware`
and experiments retain their prior timing. Do not silently promote this fork to
the default, increase its worker count, shorten refinement, or add the other
previously discussed optimization methods. Its dependency is installed with
`setup_planner_sparse.sh` into the existing planner venv without package upgrades.

Hardware and experiment GUIs select endpoints from `destinations/right` or
`destinations/left`. Hardware initially selects the first saved destination;
experiments start unselected. Original default endpoints are retained as destinationR1/L1.

| Selected arm | Goal (final pose only) | Zone | Actuator bus |
| --- | --- | --- | --- |
| Right | `destinations/right/*.json` | `right_zones/zone3.json` | `can0` |
| Left | `destinations/left/*.json` | `left_zones/zone1.json` (same as left beat workflow) | `can1` |

- Zone membership uses the original selected gripper reference point, **not the
  stick tip**, with the existing membership rule. Collision geometry still includes
  the tools/sticks. STL units are millimetres. Right grip is inline; the accepted
  left model is perpendicular/traditional grip. Consult model code rather than
  guessing mounting offsets or changing an already approved model.
- Camera: rigidly mounted ORBBEC Gemini 2, calibrated into robot coordinates.
  Preserve the accepted native depth FOV and calibration transform conventions.
  Hardware mode acquires the current pose/scene; experiment defaults use saved inputs.
- Production hardware uses the identical `experiment/workspace_curve.py` generator,
  settings and timing dispatcher, with freshly acquired inputs. The five experiment
  choices remain unchanged. Never substitute an experiment saved start/plan for
  hardware acquisition. Shared checkpoint composition does not provide motor authority.
- Timing dispatch preserves right-arm adaptive fixed-path timing (v1). Left cornered
  paths use bounded C3 smoothing plus adaptive timing (v2); continuous paths can
  retain v1. Old timing implementations remain available. Do not round after checking
  collision and then play an unchecked curve, or approximate a different playback law.
- Experiment-only update (2026-10-24 UTC): both sides now explicitly request the
  GPU-native Vulkan FP64 optimizer in `timing/vulkan/`, via `gpu_integration.py`.
  Accepted durations must be <=1.01 times the original CPU reference on the exact
  same curve; cold reference qualification still costs a CPU solve. Log that time
  separately, never call it GPU acceleration. GPU unavailable means clearly logged
  CPU fallback; numerical/quality failure means rejection, not hidden fallback.
  Path generators, approved left C3 geometry and hardware CPU policies are unchanged.
- Collision policy is `observed_obstacles`; padding policy is `occupied_volume_v1`.
  Keep approved baseline margins and avoid counting voxel volume twice. Measured
  robot-depth self-masking is a separate perception operation, not blanket deletion
  of contacts with the stick.
- Control panel: Calculate, Start, Emergency Hold, Reverse to rest + relax, Cancel
  calculation/pending start. Normal single cycle: measured rest → goal → settle/two-second
  hold → reverse → settled original rest → relax selected arm and gripper. Multiple
  checkpoints each settle/hold two seconds, then use a newly planned final→start leg.
  Midair faults keep HOLD; Reverse uses the already-traveled mission prefix.

### Read these before editing the corresponding subsystem

- `BASELINE_HARDWARE.md`: current Workspace curve hardware UI/ownership/recovery contract despite
  the historical filename; `DRUM_PLANNER.md` and `DRUM_PLANNER_EXPERIMENTS.md`.
- `drum_planner/common.py`, `planning.py`, `model.py`, `right_zone.py`, `cpp/`:
  side/goal inputs, collision policies, custom tools, zones, native checking.
- `drum_planner/experiment/{config,algorithms,search,app,runtime}.py`: candidate
  methods, selection, experiment UI, bounded workers.
- `drum_planner/timing/README.md` and `dispatch.py`: current timing and retained studies.
- `drum_planner/baseline/{arms,planner,workspace,windows,work,computation,curve,motion,transport,service}.py`:
  arm routing, shared planner, three-window UI, isolated computation, exact playback,
  motor ownership and recovery.
- `camera_playback/`, `centering/motors.py`, `safe_zone/encoder.py`: established
  hardware conventions and customized gripper targets; preserve their sign semantics.
- `diagnostics/singlearc_arms_20261021/REPORT.md` and `run_tests.sh`: dated integration
  evidence and a software test entrypoint, not proof of present hardware performance.

For Python tests, source the ROS/workspace environment to avoid false `rclpy`
collection failures. Select relevant tests; a documentation-only edit does not
require starting hardware or rerunning a lengthy robotics experiment.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  .venv-planner-experiment/bin/python -m pytest tests/test_singlearc_arms.py
```
