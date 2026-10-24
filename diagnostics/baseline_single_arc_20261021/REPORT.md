# Baseline command: Single arc + current timing integration

Date: 2026-10-21. Software-only work; the powered physical arm was not moved,
enabled, relaxed, or queried. The camera was not accessed. Saved inputs and
independent simulated motor owners were used instead.

## Changes

- Fixed the pre-calculation `DisplayModel` interface: it now declares its right
  arm, joint list and `right_drumstick` link. The renderer and measured-pose
  updates previously expected metadata only present in loaded trajectory models.
- `./plan_drums.sh baseline` now generates and ranks **Single arc** candidates
  through the same implementation/settings as `experiment --right`, then invokes
  the current arm-aware timing dispatcher. For right-arm paths the dispatcher
  deliberately retains the adaptive fixed-path law. The newer bounded C3 corner
  fix is left-only, as established by the previous timing experiment.
- Added an exact timing-law adapter to the existing baseline executor. It
  supports both the new analytic laws and historical baseline plans without
  refitting sampled points. It recomputes derivative extrema rather than trusting
  cached limit metadata. The existing controller/state machine is unchanged.
- Preview speed changes now scale the analytic clock, not just sample timestamps.
  Preview, execution and traveled-prefix return use the same curve. The goal hold
  remains two seconds at every speed.
- No widgets, controls, layout, button availability, or ownership protocol were
  changed. Existing hold/fault/reverse-and-relax behavior is retained.

The current Calculate acquisition still supplies the start pose and frozen camera
scene. Only the final pose of `recordings/record3.json` supplies the goal, never
its recorded path. The original zone3 gripper-point rule, full robot/custom-stick
collision checking and baseline margins remain in force. An experiment cache or
saved default pose is not promoted into eligibility for physical playback.

## Offline backend and execution checks

`verify_backend.py` ran the production backend under the existing resource-limited
worker, with CAN excluded from permitted socket families. On the retained right
start/scene it generated **bit-identical Single arc path and timing coefficients**
to the independently retained `experiment --right` result.

| Check | Result |
| --- | --- |
| Outward duration at 1x | 4.1279948583 s |
| Nominal outward + goal hold + reverse | 10.2559897166 s |
| Dense exact-curve samples | 2,065, plus connecting edge checks |
| Native collision/zone validation | All valid |
| Baseline obstacle padding | 0.020 m, unchanged |
| Prefix recovery checks | 20%, 60%, 100% of outward trajectory |
| Maximum joint velocity | 0.5988740155 rad/s, limit 0.6 |
| Maximum joint acceleration | 1.1955217393 rad/s², limit 1.2 |
| Maximum joint jerk | 5.9820359401 rad/s³, limit 6 |
| Exact-curve evaluator median / p99 | 0.1103 / 0.1330 ms |
| Contained worker peak memory | 386,232,320 bytes (368.34 MiB) |
| Complete backend verification wall time | 99.625 s |

The cycle duration above excludes real hardware settling; it is not a physical
performance measurement. The offline output is deliberately rejected by the real
hardware loader because no current acquisition has been performed.

Evidence: `backend_verification.json`, `backend_resources.json`, `backend.log`,
and the non-executable `offline_plan/`. The retained request points to previously
curated hash-identical robot models instead of disposable build output.

## Native UI and simulated controllers

`verify_ui.py` exercised the actual native Qt/VTK windows, first through the
previously failing no-plan constructor (without polling devices), then using the
new generated plan and independent **simulated** owners. All checks passed:

- Exactly three windows; unchanged controls, no overlaps, no automatic planning.
- Play/pause/scrub and all speed settings; frozen scene uploaded only once per view.
- Live pose updates isolated from disconnected preview; invalidation/recalculation
  and prerequisite errors remain recoverable without restarting.
- Full outward → two-second goal hold → reverse → settle → relax cycle.
- Emergency hold → checked prefix reverse → original start → relax.
- Injected simulated CAN fault: stale display, all buttons available, no midair
  relaxation, no queued surprise recovery; explicit retry after feedback recovery
  successfully reverses and relaxes at the start.
- Busy actions do not create another motor owner; changing preview speed never
  changes an active owner's requested speed.

Evidence: `ui_report.json` (IPC token omitted), `ui.log`, three screenshots and
`startup_controls.png`, plus `simulated_cycles/` event/final-status logs.
The intentional fault message in the UI report is a tested recovery scenario,
not an unexpected failure. `final_status.json` is the authoritative final owner
state; the periodically written `status.json` can precede terminal IPC completion.

## Regression suite

**864 tests passed in 285.82 seconds**, including 20 new Single arc integration
cases. This covers the existing baseline hardware-policy/UI/workspace tests,
timing and corner-smoothing tests, both-arm experiment and tool-model tests,
collision/zone/trajectory preview tests, calibration, recordings and device-free
read-only acquisition tests. See `run_tests.sh` and `regression.log`.

The complete regression process peaked at 1,761,576 KiB RSS (~1.68 GiB) and
reported zero swaps. Native simulated owners all ended in `RELAXED` with
`safe_to_close=true`; the test-created owner services exited normally.
The actual `./plan_drums.sh baseline --simulate ... --seconds 3` launcher also
exited successfully with no startup/render errors, no automatic plan, no motor
owner and all five buttons enabled (`cli_simulation_report.json`). Its system
Python resolves the same NumPy/SciPy versions as the contained backend test.

## Reproduction and boundaries

```bash
bash diagnostics/baseline_single_arc_20261021/run_tests.sh
./plan_drums.sh baseline --simulate \
  diagnostics/baseline_single_arc_20261021/offline_plan/plan.json
```

The simulation command uses fake motors and does not open CAN or the camera.
Normal operation remains `./plan_drums.sh baseline`, followed by Calculate and
an explicit Start click. No real Start click was performed during this work.

The preservation audit in `preserved_sources.json` covers the existing geometric
planners, arm-aware timing implementations/configuration, model generator,
recordings, vendor model, launcher, service and feedback code. All 20 hashes
still match their pre-change bytes. Existing left-arm experiment behavior and
right-arm path/timing generation were not altered; only the baseline integration
uses them now.

Software checks establish consistency of the planned/previewed/executed commands,
not physical tracking, holding torque, camera accuracy or actual collision
clearance. Physical verification requires the user's separate movement permission.
