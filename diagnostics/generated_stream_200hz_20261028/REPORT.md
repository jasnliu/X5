# Generated beat: isolated exact-path 200 Hz execution

## Scope

User authorized only the first proposed improvement: move generated playback
off the GUI clock, without reshaping the trajectory, retiming it, or changing
motor settings. Physical arm tests were explicitly prohibited.

Changed normal generated `start_beat.sh --hardware` outward left/right playback
and its already-prepared right/left return. `plan_drums.sh`, its planners/timing,
the old recording worker, strike/calibration algorithms and hardware settings
were not changed. No CAN, ESP32, microphone or camera was opened for these tests.

## Implementation

- `camera_playback/generated_stream.py`: spawned selected-arm position writer.
  Reuses established recording-worker CAN sockets, locking/sign conventions,
  bounded send retries and feedback decoding with a *narrower* command allowlist.
  Only position writes and mode queries are admitted. Gain, speed, current,
  enable/disable, gripper, mode writes and opposite-arm targets are prohibited.
- Evaluates the existing `Curve.at()` or `recovery_at()` law on absolute 5 ms
  deadlines. Missed slots are skipped, never replayed in a burst. The existing
  80 ms actual-command guard remains. GUI heartbeat loss stops progress and
  attempts a fresh measured Hold. No automatic return or relaxation.
- Full feedback/tracking and the original joint/envelope/gripper checks remain.
  Parent GUI retains enabling, settling, motor ownership and explicit controls.
  Parent target writes are excluded until the writer has exited. Hold preserves
  the latest commanded prefix and worker-measured stop pose, not old GUI progress.
- Existing MIT J7/J6 owners remain unchanged during return; checked references
  are passed to their existing shared goals, never CSP writes to those joints.
- Emergency Relax and Stop/Close stop the new delegate before disable attempts.
  A delegate/handoff error cannot veto the existing unconditional disable action.
- `generated_simulation.py` runs the same spawned writer on explicit fake motors;
  deterministic fake-clock unit tests retain their existing device-free path.
- Double-buffered progress has a monotonic generation counter and no mutex that
  a forcibly stopped delegate could strand in the GUI.

Nominal movement durations, polynomial coefficients, endpoints, V/A/jerk limits,
drive caps and gains are unchanged. Worker startup/endpoint settling are separate
from movement duration; this is not a promise of zero scheduling latency.

## Software evidence

- **145 tests passed** in the selected generated/recording/transport/hold/strike
  regressions (`tests_final.log`). The **49 directly relevant tests were rerun
  after the final progress-buffer adjustment**, all passing (`focused_final.log`).
- Actual spawned-process tests cover both sides, exact reference equality,
  skipped deadlines/no catch-up burst, 80 ms failure, GUI heartbeat loss,
  invalid feedback, cancellation, exclusive target ownership, prohibited writes,
  existing MIT reference routing and unconditional emergency release.
- Saved current curves and prepared returns were compared at 251 times per side
  per direction, with array-exact reference equality and no plan mutation.
- **7 native Tk integration checks passed** (`ui_report.json`, `ui_final.log`):
  a deliberate 180 ms GUI pause did not stall the separate writer; Hold retained
  power and the correct prefix; partial and full prepared returns succeeded;
  full left/right endpoint playback succeeded; Emergency Relax worked with stale
  feedback; Stop/Close exited during an active writer. No end-of-run replanning.
- Final native UI run: `planner_runs/generated-beat-20261027T233747-0e0af593/`.
  Its fake-motor traces had per-stream median intervals **4.983–4.996 ms**,
  99th percentiles **6.788–7.417 ms**, and a worst interval **10.492 ms**.
  These are software scheduling observations, **not CAN timing measurements or
  a hard-real-time guarantee**. Three cancelled traces are intentional Hold,
  Emergency Relax and Stop tests, not failed normal runs (`cadence.json`).
- **210 protected source/data files stayed byte-identical**, including planner,
  timing, recording smoothing, strike code, saved destinations/recordings and
  calibration JSON/source inputs (`preservation.json`, `protected_before.json`).
- Seven changed/new Python files passed syntax parsing. Original changed-source
  snapshots for this task are in `before/`; prior user modifications were retained.

## Reproduce without devices

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
.venv-planner-experiment/bin/python -m pytest \
  tests/test_generated_stream.py tests/test_generated_beat.py \
  tests/test_generated_stop_return.py tests/test_generated_worker_hold.py -q
.venv-planner-experiment/bin/python -m tests.generated_stream_ui_check
```

The native UI test requires the desktop display and the existing saved software
mission under `diagnostics/generated_beat_controls_20261028/verified/`.
It explicitly forbids physical CAN and replaces motor/ESP32 access.

## Remaining physical question

No real arm was moved, enabled, relaxed or queried. Vibration reduction, physical
tracking and real-bus cadence remain unmeasured and require later user-authorized
hardware testing. No gain/filter/retiming change was added to compensate for
an untested physical effect.
