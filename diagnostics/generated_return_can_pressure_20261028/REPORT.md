# Generated Hold/Return CAN queue-pressure correction — 2026-10-28

## Scope

Only normal generated `start_beat.sh --hardware` feedback scheduling and generated
Hold/Return transmission handling were changed. **All verification was offline.**
No physical CAN socket, camera, microphone, ESP32, motor enable/hold/disable, or
physical movement was used. Fake-channel tests and saved-scene GUI simulations
are not physical CAN timing or robot recovery certification.

## Evidence and reproduction

- The saved real session `planner_runs/generated-beat-20261028T204722-1edea731/`
  reports `right worker: [Errno 105] No buffer space available` in
  `last_fault.json`. All 16 stored states there are running, with 3.207 ms
  host receive ages. This identifies a transmit failure, not proof that the
  physical encoders stopped. Those ages describe that later snapshot only.
- Both outbound stream results succeeded; subsequent right-return writers were
  cancelled within roughly 59–116 ms. This supports investigating return/owner
  handoff rather than changing path geometry or goals.
- Before correction, injected brief ENOBUFS/EAGAIN in the **actual right and
  left generated Hold branches and both replacement recovery workers** caused
  worker exit despite fresh feedback and a healthy parent heartbeat.
- Separately, a query-send error in `GeneratedMotors.poll` incorrectly latched
  `feedback_failed_at`, making `fresh()` false despite recent received states.
- `before_test.txt` records the pre-fix regression run: 16 failures / 6 passes.
  This includes expected failures for the not-yet-implemented helper API and
  clearer diagnostics; it does not represent 16 independent hardware faults.

Average traffic alone is not the problem: eight-query bursts competed with CSP
and MIT writes. No physical bus trace was taken, so actual queue occupancy and
electrical health are not claimed as measured facts.

## Correction

1. **Generated Hold/Return only:** `PoweredPathHold.send` tolerates brief
   ENOBUFS/EAGAIN. On refusal it restores the last accepted controller state,
   receives new feedback on the next worker iteration, and recomputes for the
   current checked target. It never sleeps then resends an obsolete command.
   A continuously blocked send fails at the existing 20 ms playback/MIT budget;
   non-transient send errors fail immediately. Controller gains, bounds, signs,
   velocity/acceleration behavior and freshness requirements are unchanged.
2. **Replacement recovery workers** use the same handling for Hold and startup
   state/mode queries. They remain query/MIT-only: no enable, disable, mode
   changes, strikes, or guessed center. Valid MIT readback and fresh running
   feedback still precede any Hold command.
3. **Generated query scheduling:** at most two state requests per bus per tick,
   prioritized fairly, skipping joints with recent command/MIT feedback. A
   200 Hz child can query eight drives in 20 ms; the 50 Hz parent in 80 ms. The
   parent yields to the ready live child only while received states remain
   fresh, and resumes if the child dies/stops supplying replies. Mode readbacks
   during startup are also paced. Existing 120/300 ms receiver checks remain.
4. **TX is not RX:** query-send exceptions report the bus/joint and transmit
   failure without setting the receive-failure latch. Real receive, motor-fault,
   overtemperature and stale-feedback failures still latch; state queries
   continue so recovery is possible after healthy feedback returns.

No new user steps, readiness buttons, watchdog extensions, background heartbeat,
alternate motion controller, trajectory smoothing, or setting changes were added.

## Fresh verification

- **313 automated tests passed** (`regression_test.txt`), including generated
  sequences/controller/transport, left and right encoder display, normal
  hybrid/snare strike behavior, MIT signs/ownership and playback transport.
  **34 new pressure/scheduling cases** are included in that total, not additive.
- The new cases run real worker loops with transient/persistent ENOBUFS/EAGAIN,
  ENETDOWN, stale feedback, startup-query pressure, genuine motor/temperature/RX
  faults, late-send rejection and integrator rollback. Healthy Hold command
  bytes match the original math for both arms.
- A discrete **ten-frame-per-bus queue model** runs the real 200 Hz shared
  Controller, actual parent/child decoders and send gate, and both powered Hold
  controllers. Independent receiver copies and injected concurrent queue
  saturation are modeled. Both selected-arm cases complete the original exact
  curve; MIT accepted-send gaps remain under 20 ms and all receiver ages under
  120 ms. No enable/disable/mode/drive-setting packets are allowed. This is a
  synthetic congestion test, not a physical bandwidth measurement.
- Actual Tk control tests: **10 passed** in `controller-ui/ui_report.json`:
  full outward/return, interrupted cached return, real isolated 200 Hz writer,
  active GUI-stall fault, retry, startup cancellation, Relax and Stop.
- Actual Tk recovery tests: **7 passed** in `recovery-ui/ui_report.json`:
  repeated encoder faults, pending Return, cancellation, missing cache, saved
  camera-scene prefix validation and fresh planning, right-then-left return and
  release at captured rest. Held-owner heartbeat models stayed below the
  unchanged 500 ms limit (maximum **69.598 ms**). No real devices were used.

## Preservation

`preservation.json` confirms **260 previously existing files unchanged** across
the inventoried planner, motor conventions, configurations, launchers, saved
destinations and recordings. Five existing runtime files changed:

- `camera_playback/generated_transport.py`
- `camera_playback/generated_stream.py`
- `camera_playback/generated_worker_hold.py`
- `camera_playback/hybrid_strike.py` — generated Hold branch only
- `camera_playback/snare_beat.py` — generated Hold branch only

One generated-only helper was added: `camera_playback/generated_can.py`.
AST comparison confirms both normal strike files are identical to their saved
pre-change versions after removing their generated Hold branches. The original
shared `plan_drums` controller, planners, timing, limits, calibration, models,
recordings and non-generated playback transport were not edited. Existing
Emergency Relax/Stop and calculated-return selection logic were not changed.

Source snapshots are in `before/`. Unit tests are in
`tests/test_generated_can_pressure.py`; the recovery GUI test gained only an
output-directory argument so historical evidence is not overwritten.

The reproduced software failure is fixed and regressions passed. This does not
promise recovery through genuinely missing feedback, persistent CAN failure,
drive faults, or an invalid return route. Physical acceptance requires a
separately authorized test; none was performed here.
