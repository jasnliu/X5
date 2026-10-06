# MIT readiness repair — physical verification, 2026-09-30

## Implementation
- 40 ms position history; entry requires every position within 0.15 degrees of
  the fixed anchor, range <= 0.05 degrees and fitted trend <= 0.03 rad/s.
- Ready-state hysteresis doubles range/trend thresholds, not anchor tolerance.
  Noisy instantaneous drive velocity no longer vetoes stationary readiness.
- Raw position/velocity remain unfiltered for predictive catch and return control.
- A fresh encoder sample is rechecked against the same history before each press.
- A late idle update refreshes the hold and receives its reply before admitting
  a press. The last received point is retained so refresh does not create a
  fictitious hole in position history. Idle feedback/history gaps >40 ms require
  a new stable window. Active-motion deadlines and strike freshness remain 20 ms.
- GUI phase/button rendering shares a snapshot with a separate 100 ms UI budget.
- Center + Relax uses current worker readiness instead of a stale GUI phase.

## Offline evidence
- **284 regression tests and all three offline GUI checks passed** on the final code.
- Replay of the previous real hold capture: old raw-velocity gate ready for
  893/8679 observations (10.29%); new encoder gate ready for 8679/8679 (100%).
- Regression tests cover raw velocity noise, real drift/oscillation, hysteresis,
  stale samples, short idle pauses with aged replies, retained intermediate
  readings, strict active deadlines, actual spawned-process independence, and
  single-snapshot UI behavior.
- Test logs: `right_camera_playback_mit_settling_tests.txt` and
  `right_camera_playback_mit_settling_ui.txt`.

## Final physical run: PASS
The normal launcher stack ran with physical CAN/motors, RViz, Y2 camera and ST7.
The recording was **recordings/record3.json**, unchanged. Existing ntfy alerts
were delivered before every movement run. An external temporary audit subclass
skipped human stick loading and visual acceptance because no stick was loaded;
production vision/loading gates were not removed.

- Record3 completed; a 60-second endpoint hold was observed.
- After initial settling, **1617 stationary UI/worker checks**, **0 disabled
  button observations**, **0 unready observations**.
- Included a deliberate **25.14 ms worker pause** only during stationary hold.
  The drive continued holding; readiness remained stable through recovery.
- Three actual normal-button requests, each for an unloaded 1-degree strike,
  completed once each; readiness returned after every rebound.

| Strike | Measured maximum drop | Return overshoot |
|---|---:|---:|
| 1 | 0.945 degrees | 0.088 degrees |
| 2 | 1.011 degrees | 0.088 degrees |
| 3 | 1.099 degrees | 0.066 degrees |

- No motor-control fault during the successful run.
- Normal Center + Relax completed. All 16 drives reported disabled, independently
  rechecked with state-only postflight queries. Both CAN interfaces ERROR-ACTIVE,
  transmit/receive error counters zero; no test/control processes left running.

Evidence: `right_camera_playback_mit_settling_verified.json`,
`right_camera_playback_mit_settling_physical.txt`,
`right_camera_playback_mit_settling_can.txt`,
`right_camera_playback_mit_settling_idle_pause.json`, and
`right_camera_playback_mit_settling_postflight.txt`.

The first three readiness audits are separately preserved as `...first...`,
`...second...` and `...third...`. They found residual idle-timing/history drops
and are NOT counted as passing. The final strict audit passed without weakening
its zero-flicker acceptance criterion. A non-executable snapshot of the audit is
saved as `right_camera_playback_mit_settling_harness.py.txt`.

Scope: stable readiness and accepted unloaded strikes, not loaded-stick sound
calibration or identical impact dynamics. The existing brief mode-entry dip was
not eliminated by this readiness change. Camera KeyboardInterrupt / exit -2 in
the log occurs during normal launcher teardown, not during motor control.
