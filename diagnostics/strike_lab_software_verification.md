# Strike lab software verification — 2026-10-02

Historical software-phase record. The claims and source comparison below apply
to that phase, before hardware authorization. Later hardware tuning, code changes,
and physical evidence are recorded in [EXPERIMENT_RESULTS.md](../EXPERIMENT_RESULTS.md).

**Software only. No physical CAN connection, motor enable, centering, hold, strike, or live motion notification was performed.**

## Implemented

- Independent `experiment.sh`, Tk panel, RViz model, spawned controller, and synthetic dynamics.
- All eight planned method families, exposed as nine preserved modes.
- Fixed 10-degree target, initial 9.8–10.2-degree depth acceptance, full >9-degree dwell integration.
- Shared smoothness/readiness/return/feedback checks, bounded parameter search, validation, mixed-interval endurance.
- Source snapshots, parameters, full-rate traces, failures, incomplete attempts, reports, and non-destructive rescoring.
- Explicit future hardware adapter with notification-before-setup, gripper-first custom right center, J7-only strikes, and right-only relaxation. This path was tested with fakes, not a robot.

## Checks

- Full repository regression: **337 tests passed** (including 49 strike-lab tests). See `strike_lab_software_tests.txt` and `strike_lab_software_unit_tests.txt`.
- Real Tk + ROS + spawned synthetic worker: **nine modes passed**, actual panel screenshot reviewed; physical CAN socket creation forbidden by audit hook. See `strike_lab_software_ui.txt` and `strike_lab_software_ui.png`.
- Complete launcher: robot_state_publisher + RViz + Tk + worker, nine synthetic modes passed, automatic closure clean. See `strike_lab_software_launch.txt`.
- RViz robot mesh actually rendered; separate visual capture and terminal-style shutdown check in `strike_lab_software_visual_launch.txt` and `strike_lab_software_rviz.png`.
- Shell syntax, Python compilation, and preserved-source checks passed. Existing beat launcher/controller files match their before-task hashes (`strike_lab_software_protected_sources.txt`).
- Process-group Ctrl-C now produces a clean worker stop, not a spurious fault; covered by a spawned-process regression test and the complete live UI/ROS simulation check. The pre-fix log is retained as `strike_lab_software_ctrl_c_before_fix.txt`.
- Current campaign source snapshot matches the implemented executable source byte-for-byte.
- CLI rescoring reproduced every original pass/fail verdict in a new directory; originals retained.

## Short synthetic campaign

Configuration: `config/experiment_software_smoke.json`. This reduced budget exercises the software workflow, not physical calibration or statistical validation.

Session: `experiment_results/software_verification/20261002T072425-378cb86e`
- 207 attempts: **203 accepted, four depth-invalid candidates retained**.
- 81 screening + 81 refinement + 27 validation + 18 short-endurance attempts.
- All nine modes completed every planned software-check stage.
- No physical winner is declared. The four failed torque-led neighboring candidates demonstrate rejection rather than hidden successful-subset selection.

### Synthetic validation data (three repetitions per mode; not robot measurements)

| Mode | Passes | Median peak ° | Median lower-zone ms | P95 lower-zone ms | Median cycle ms |
|---|---:|---:|---:|---:|---:|
| baseline | 3/3 | 10.012 | 76.101 | 76.101 | 498.000 |
| cosine | 3/3 | 9.990 | 54.220 | 54.220 | 416.000 |
| gravity | 3/3 | 10.012 | 71.238 | 71.238 | 472.000 |
| hybrid | 3/3 | 9.990 | 73.638 | 73.638 | 460.000 |
| impedance | 3/3 | 9.990 | 77.968 | 77.968 | 412.000 |
| optimized | 3/3 | 9.990 | 81.235 | 81.235 | 418.000 |
| powered | 3/3 | 9.990 | 77.238 | 77.238 | 412.000 |
| torque | 3/3 | 10.012 | 78.438 | 78.438 | 412.000 |
| variable_damping | 3/3 | 10.012 | 74.035 | 74.035 | 482.000 |

These data use an assumed inertial/gravity/delay plant. They establish that the code and evaluation pipeline run, not which mechanism will sound best or work best on hardware.

## Preserved evidence

- Final and earlier verification campaigns: `experiment_results/software_verification/`.
- Earlier development smoke runs and source versions: `experiment_results/software_development/`.
- Complete UI/ROS synthetic runs: `experiment_results/software_ui_stack/` and `experiment_results/software_visual_check/`.
- An initial visual-test harness signalled only the ROS launcher PID, which caused ROS to escalate child shutdown after five seconds. That harness log is retained as `strike_lab_software_parent_only_signal.txt`; the normal terminal-style check signals the whole launched process group instead.

## Deferred until authorized hardware testing

Actual motor inertia/gravity, communication jitter, feasible torque/velocity/gains, thermal behavior, candidate tuning, 100-trial validation, endurance, and the real-arm winning mechanism are unverified. No simulated winner should be installed as the playing controller.
No stick or cymbal is involved in this proxy experiment, so even subsequent unloaded physical results will not prove tone or actual cymbal contact duration.
