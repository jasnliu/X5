# Normal experiment: joint-limit profiles and simulated starts

Completed 2026-10-25 UTC. Scope: `plan_drums.sh experiment --right/--left` only. The pre-existing `--fast` mode and all hardware modes are unchanged. No physical motion, motor/gripper commands, fresh encoder queries or fresh camera captures were performed during this task.

## Delivered

- **Simulated start**: Default remains the acquired pose. Selecting a destination immediately places only the selected simulated arm there; it inserts no approach movement. The return ends at that selected start. Start/target files are privately snapshotted. Invalid position bounds are rejected, not clipped.
- **Joint limits / profiles…**: seven independent velocity, acceleration and jerk values, in rad/s, rad/s² and rad/s³; explicit right/left JSON save/load; positive finite input validation and wrong-side rejection. The original defaults remain **0.6 / 1.2 / 6 on every joint**. No unmeasured higher limits are installed as defaults.
- **Fast / Balanced / Thorough timing effort**, default Balanced. Bounded sparse CPU timing accepts only its fastest independently checked clock. The original 0.1% duration reserve, rest endpoints, native full-arm/stick collision/zone validation and exact playback law remain.
- Display of calculated joint peaks and constraint utilization, including every checkpoint and return leg. The profile hash/start identity prevents stale results from being installed after a change.
- Start/limit/effort changes invalidate obsolete work, cancel/reap it and recompute the selected targets. No target selected still means no trajectory calculation. Start equal to target gives a collision-checked **zero-movement** result and the usual two-second hold.
- The original acquired arm pose remains the perception self-mask even after a simulated teleport, on later checkpoint legs and on camera-only refresh. The passive arm stays captured and unchanged. No calibration or camera geometry was changed.

Normal usage (these commands keep the established fresh camera/pose acquisition at startup):

```bash
./plan_drums.sh experiment --right
./plan_drums.sh experiment --left
# Optional preselection:
./plan_drums.sh experiment --right --start R1 --destinations R2
./plan_drums.sh experiment --right --start R1 --destinations R2 \
  --motion-profile /path/right_limits.json --timing-quality thorough
```

Use the GUI **Joint limits / profiles…** button to create profiles. The default save location is `config/experiment_motion_profiles/right` or `left`. New launches do not silently restore a previous custom limit set. The full schema and control behavior are in `DRUM_PLANNER_EXPERIMENTS.md`. These profiles change simulation timing only, not motor protection or hardware settings.

## Current R1 → R2: calculation/speed balance

All rows use the same current R1, R2, frozen scene, acquired passive-arm pose, calibrated geometry and original limits. Each row is a new complete CLI process, freshly generating and timing the path. No trajectory/timing/reference cache is used. This includes saved-input initialization, path generation, timing and validation; **it excludes live camera/encoder acquisition**. These are single process-cold samples, not distributions or guaranteed latencies; OS filesystem/library caches were not flushed.

| Effort | One-way movement, no holds/return | Timing calculation | Complete cold calculation |
|---|---:|---:|---:|
| Fast | 1.764933 s | 1.930 s | 8.134 s |
| Balanced | 1.656984 s | 3.459 s | 9.635 s |
| Thorough | 1.646733 s | 4.804 s | 11.090 s |

Balanced saves about **108 ms** of movement versus Fast for about **1.50 s** more cold computation in these runs. Thorough saves another **10.25 ms (0.62%)** for **1.45 s** more computation. Balanced is therefore the default; Thorough is available when that small improvement matters. The nested work retains its fastest valid incumbent, though separate wall-time-limited runs can vary with machine load.

This is **not a new route-speed breakthrough at unchanged limits**: approximately 1.657 s matches the earlier well-refined result on this same Workspace curve. The useful changes are explicit per-joint constraints, reliable convergence with unequal limits, selectable extra effort and an isolated R1-to-R2 starting position. The approximately 1.647 s result is not proof of the global fastest feasible time.

### Fixed-path timing comparisons

`timing_final.json` compares the retained old bounded solver with the new efforts on four identical polynomial sources. These are same-process math timings, not the full cold numbers above. All 16 laws passed the independent original continuous-root checker and exact geometry equality.

| Frozen curve | Retained old | Fast | Balanced | Thorough |
|---|---:|---:|---:|---:|
| current_R1_R2 | 1.895711 s | 1.764933 s | 1.656984 s | 1.646733 s |
| historical_checkpoint_1 | 4.212610 s | 4.212610 s | 4.212610 s | 4.205700 s |
| historical_return_to_start | 2.712258 s | 2.712258 s | 2.712258 s | 2.702366 s |
| historical_left | 2.857103 s | 2.857103 s | 2.857103 s | 2.838998 s |

The retained solver has a one-second sparse budget; its stop iterate varies. Its 1.896 s current-R1/R2 sample is not a stable historical speed baseline. This study does not claim every new run is that percentage faster than the best old result. The old implementation is unchanged and remains available for comparisons.

### Unequal-limit and left-arm tests

- A **hypothetical, simulation-only** distal-joint profile gave about **1.355 s per R1↔R2 transfer**, on the exact same polynomial path. It retained J1–J4 defaults but used J5 V/A/J=1.2/4.8/48 and J6–J7=2.4/19.2/384. These are diagnostic inputs, **not recommended or measured hardware capabilities**. They demonstrate that changing the supplied limits actually changes timing; they were not installed as defaults.
- This unequal case exposed poor nonlinear initialization when starting from a globally stretched seed. Keeping fresh raw seed coefficients for initialization, while separately retaining the certified feasible incumbent, fixed the convergence problem. `conditioning_results.json` records the final repeated cases; the seed is never accepted unchecked.
- Left tests use L1 and a synthetic endpoint only **1° away in J6**, not a claimed full drum transfer: 0.459 s at original limits/Balanced, 0.304 s with the unequal diagnostic profile/Thorough. Because both limits and effort differ, that comparison is not attributed solely to limits.
- Both sides passed multi-checkpoint tests with unequal profiles, zero first/return legs, two-second holds, unchanged passive geometry and the correct acquired self-mask. The left L1→near→L1 pair takes 0.612 s movement with the diagnostic profile/Balanced.
- The saved captured-rest→L1 case remains **no_feasible_route** in this frozen scene. It was retained as an expected rejection, not bypassed or counted as a successful route.

## Verification and preservation

- **258 regression tests passed**, five historical preservation-hash checks deselected after recording their failures. They refer to destination/calibration states predating this task: four expect an old R1 hash and one an old `common.py` calibration-default hash. Both files exactly match the before-task snapshot. See `legacy_check_notes.json`, `regression_tests.log` and `regression_final.log`; the historical expectations were not rewritten to manufacture a green suite.
- **12/12 actual headless CLI cases had their expected outcome**: 11 accepted plans plus the blocked left default case. This includes both arms, original/custom limits, all effort levels, reversed endpoints, stationary endpoints and checkpoint returns.
- **Both native Qt/VTK windows tested** with real controls and real contained planning: ordinary Default-start target selection without first teleporting, immediate custom spawn, invalid values, wrong-side profile, exact JSON save/load, apply, cancel/retry, obsolete-result rejection, stationary target, Play/Pause, timeline, speed and close-while-running cleanup. Screenshots are retained. Camera-only Recapture was tested with **mocked capture completion and real initialization/planning**, not a fresh live camera test. Default-start mask initialization and retention of external CLI targets across recapture were included in the final regressions.
- A separate real CLI GUI launch with explicit external start/target JSON paths automatically calculated the correct result once and built the static scene mesh once.
- Final independent audit rechecked **11 nonstationary laws** using the original continuous polynomial-root checker, checked all checkpoint masks/limits/passive poses/returns, and confirmed the four forward R1→R2 source paths (three efforts plus custom profile) are **bit-identical** to the prior Workspace source, including the original source timing coefficients.
- Every audited planning job retained the **3 GiB cap**, serial execution and **AF_CAN exclusion**. No result/reference cache reuse. Custom profile identities, captured and simulated states, accepted extrema and actual backend are recorded per plan.
- `before_hashes.json` and `audit.json` cover the current-task preservation boundary, including the user’s pre-existing uncommitted `--fast` work. Hardware, shared timing, path generation, model/tools, raw recordings, destinations and calibration were preserved. No commits, staging, cleanup of user data or package changes.

## Evidence and reproduction

- Summary: `summary.json`, `results.csv`, `headless_final_results.json`, `timing_final.json`, `supplemental_results.json`, `ui_results.json`.
- Identity/scope: `provenance.json`, `before_hashes.json`, `audit.py`, `audit.json`, `legacy_check_notes.json`.
- Frozen inputs: `inputs/` plus the already retained capture at `diagnostics/r1_r2_wrist_speed_20261024/session/acquisition/`. The saved capture uses `calibration_sessions/20261024T211629Z-1227b7b1-reprocessed/calibration.json`; provenance hashes identify all bytes.
- Scripts: `headless_study.py` (11 serial fresh CLI processes), `timing_study.py` (16 math cases), `conditioning_check.py`, `ui_check.py` (native windows, no devices). `supplemental_check.py` runs the additional left checkpoint and external-path CLI GUI checks.
- Full raw per-plan outputs/logs remain locally under `runs_final/`, `left_sequence_final/`, `cli_ui_final/`, `ui_verified_right/`, `ui_verified_left/` and `math_laws/`. Only concise evidence, scripts, fixtures and screenshots are allowlisted; bulk runtime sessions are not added to Git.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
X5_STUDY_RUN=replay .venv-planner-experiment/bin/python diagnostics/experiment_controls_20261025/headless_study.py
# Native Qt/VTK, requires the existing desktop DISPLAY; capture is mocked:
X5_UI_RUN=replay_ .venv-planner-experiment/bin/python diagnostics/experiment_controls_20261025/ui_check.py
bash diagnostics/experiment_controls_20261025/run_regressions.sh
.venv-planner-experiment/bin/python diagnostics/experiment_controls_20261025/supplemental_check.py replay
```

`timing_study.py replay` can be run in the existing bounded `experiment.runtime.Job`; the optional positional tag works even though that job intentionally filters unrelated environment variables. `audit.py` audits the retained final local raw runs. Reproduction regenerates new trajectories rather than loading final plans. Runtime varies with system load. No software timing test establishes mechanical acceleration, torque, tracking, thermal or striking performance.
