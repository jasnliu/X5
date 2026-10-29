# Generated beat playback: shared controller and startup handshake

Date: 2026-10-28. Checkout: `/home/jason/Proyectos3/X5`.

## Scope and result

Normal `start_beat.sh --hardware` generated-path execution now delegates each
outward/prepared-return segment to the existing
`drum_planner.baseline.motion.Controller`, not a second trajectory-following
implementation. `camera_playback/generated_controller.py` adapts only the
already-powered dual-arm policy and sequence handoff. The common controller's
`step`, `send`, elapsed-time evaluation, tracking/discontinuity/scheduler checks,
and measured-pose fault Hold are reused. The original controller source is
byte-for-byte unchanged.

The existing parent remains the sole motor owner. It retains arming at acquired
rest, left-then-right startup, endpoint settling, percussion handoff, prepared
right-then-left return, Hold, unconditional explicit Relax/Stop, and final
rest release. The existing selected-position-only delegate and J7/J6 MIT target
handoff remain. There are no competing hardware controller instances.

**No physical devices were opened or commanded in this work.** Tests used saved
camera/encoder inputs, fake motors, and in-memory/mock transports. UI logs that
say “Disable sent” refer to fake motors. No arm, gripper, or ESP32/hi-hat motion
was performed. These results do not certify real servo tracking or vibration.

## Root cause and correction

The old worker's first feedback-startup check compared the GUI heartbeat against
0.5 seconds, although that timestamp was written *before* multiprocessing spawn,
imports/unpickling, and the parent's read-only feedback refresh. The two reported
runs failed before their first path command. The parent remained alive; this was
not evidence of an unreachable path or a failing motor.

`reproduce_before.py` loads the preserved original worker and reproduces the
identical error with a simulated 0.55-second startup delay and fake motors:
`before_reproduction.json`. Path progress remains zero and no trace is started.
This deterministic test isolates the bad first-check condition; it does not
pretend to identify every millisecond spent by the original hardware launch.

The corrected lifecycle is automatic:

1. Initialize child, validate the already-generated curve, acquire fresh feedback
   and confirm existing motor modes; do not send moving targets.
2. Signal ready; wait for acknowledgement from a live GUI callback.
3. Only then start the shared controller's path clock and **unchanged 0.5-second
   active GUI watchdog**.

Initialization/acknowledgement has a separate 10-second deadline, covering the
existing bounded 3-second feedback and 0.5-second mode startup plus cold-process
work. Cancellation and parent-process loss are checked throughout. A ready
worker never starts merely because a timestamp remains present. No operator
confirmation/button was added. A genuine GUI stall after acknowledgement still
faults to a measured powered Hold; there is no automatic return or relaxation.

`result.json` now records executor, initialization/acknowledgement delays,
controller phase, peak tracking error, and maximum running heartbeat age.
The prepared geometry, timing laws/durations, joint limits, DT1 policy, drive
settings, and 200 Hz target schedule were not changed.

## Fresh verification

### Unit/integration regressions

- `final_controller_tests.log`: **111 passed**. Includes both-arm cold-unpickling
  over 0.5 seconds, no motion before acknowledgement, the actual hardware
  `start_stream` method with device I/O substituted (including its feedback drain),
  startup cancellation/Relax, bounded unacknowledged startup, unchanged active
  watchdog, 80 ms missed-tick protection, exact outward/prefix/legacy-connector
  laws, common timing/side/discontinuity guards, packet allowlists, MIT target
  signs/ownership, and existing recording/strike regressions.
- `regression_tests.log`: **416 passed, 1 legacy checksum failure**. This broader
  run includes generated sequence/controls, worker Hold, prepare-only isolation,
  baseline hardware/controller, current hardware adapter, right J2/rest recovery,
  both-arm routing, and camera-playback tests.
- The one failure is
  `tests/test_singlearc_arms.py::test_algorithm_timing_models_recordings_and_shared_motors_preserved`:
  its October 21 historical manifest expects an older `drum_planner/model.py`
  hash. That file was already modified before this task (mtime October 27,
  21:23:45 local). It was not edited here, nor was the test weakened to make it
  pass. The current behavior-specific J2/rest tests pass.

Thus **527 tests passed across the final two suites**, with that one unrelated
historical-manifest failure explicitly retained.

### Actual GUI, saved failing mission, fake motors

`tests/generated_controller_ui_check.py` uses this exact reported failed mission:

`planner_runs/generated-beat-20261028T114902-81cf47ce/plan-b5cbbf61/mission.json`

Ten workflow assertions passed in each of two runs:

- planner venv: `ui_report.json`;
- hardware launcher's `/usr/bin/python3`, ROS domain 92, **with both native 3-D
  windows and the Tk control panel**: `system_ui/ui_report.json`.

Each run covers uninterrupted left→right goal arrival; full cached right→left
return; no end replanning; a 180 ms GUI pause without stretching the path;
Emergency Hold with fixed powered poses; partial cached return; intentional
post-start heartbeat loss followed by operator recovery; Emergency Relax during
slow startup before any path begins; Relax with stale parent feedback; and
Stop/Close during movement. Cancellation is joined before target handoff.
Each report contains 11 worker outcomes: six completed segments, four deliberate
operator cancellations, and one deliberately injected active-heartbeat fault.
There were no unexpected playback faults.

The first system-runtime run completed its controls but hit a test-harness report
folder creation error. The harness now creates that folder and the complete
system-runtime run was repeated successfully; the original log is retained as
`system_ui_first_reporting_failure.log`.

### Exact-path and preservation audit

`verify_outputs.py` / `preservation_and_trace_audit.json` verify:

- **16,961 actual fake-writer trace samples** across the two final GUI runs;
- maximum difference from the original curve at its recorded path time:
  **2.220446049250313e-16 rad** (return-connector floating-point roundoff);
- completed outbound endpoints/durations remain exact;
- per-segment median send intervals approximately **4.985–5.022 ms** (200 Hz);
- twelve protected source/input files, including common controller, curve,
  recovery law, generated planning/sequencing, DT1 policy and the actual saved
  mission/plans, retain their before-work hashes;
- parent `generated_transport.py` executable AST is unchanged (comment only).

The exact-source snapshots for changed pre-existing files are under `before/`.
No source staging/reset, camera recapture, motor-limit change, model retraining,
or physical test was performed. Test windows/workers were closed afterward.

## Reproduction commands (device-free)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
.venv-planner-experiment/bin/python -m pytest -q \
  tests/test_generated_stream.py tests/test_generated_controller_startup.py \
  tests/test_recording_spawn_feedback.py tests/test_hybrid_strike.py \
  tests/test_snare_ownership.py tests/test_snare_beat.py tests/test_snare_worker_io.py
ROS_DOMAIN_ID=92 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 \
  -m tests.generated_controller_ui_check --with-preview \
  --report-dir diagnostics/generated_controller_reuse_20261028/system_ui
.venv-planner-experiment/bin/python \
  diagnostics/generated_controller_reuse_20261028/verify_outputs.py
```
