# Ride swing depth boost: +0.5 degrees

## Behavior

Normal `start_beat.sh --hardware` still searches using the same one-degree
steps and the same ride detector. After the accepted attempt completes its
original return, continuous swing uses **detected displacement +0.5 degrees**
from the same J7 anchor. Example: detect at 10 degrees, swing at 10.5 degrees.
The boost is calculated once from the detected depth, not accumulated per hit.

`_finish_hit_strike_attempt()` asks the hardware adapter for the boosted target.
The existing session validator checks its already validated geometric corridor,
J7 bounds and hybrid hard-depth/acceptance limits BEFORE starting the hi-hat or
swing. Limits are not widened. If +0.5 cannot fit, the existing controlled
finish-return / Center + Relax path is used; no off-center relax is added.
12 -> 12.5 is supported when it fits the existing 13-degree hardware corridor.

Continuous depth storage/validation/logging now preserves fractions instead
of truncating them to integers. Existing status/terminal text reports the
boosted depth accurately. UI layout and controls are unchanged.

The search, ride model, beat grid, hi-hat synchronization, experiment tuning,
controller algorithm, recordings and pure `--test`/manual `--hardwaretest`
behavior are unchanged. No model training or physical actuation was performed.

## Checks

- Eight focused boost tests pass: 10 -> 10.5, 12 -> 12.5, fixed anchor, no
  cumulative increase, unchanged search target, nonhardware bypass, invalid
  numeric values, corridor/hard-limit rejection and fractional log output.
- Actual hybrid controller + simulated plant executes 16 swing strokes at
  10.5 and 12.5 degrees after integer-depth searches, then settles back at the
  original anchor. No controller strategy or parameter changes were needed.
- Actual Tk hardware-mode routing test passes with all CAN access forbidden
  and motor/audio/ESP32 I/O mocked. One RUN follows the same workflow; accepted
  5-degree search becomes 5.5-degree swing, then controlled center/relax.
- The first full run passed 532 checks and found one outdated assertion that
  still expected a 5-degree swing after a 5-degree detection. That test now
  expects 5.5-degree swing while explicitly checking the search remains at 5.
  Its five-test audio timing group passes. **Final full suite: 533 tests
  passed in 136.250 s** (`full_tests_final.log`).
- Protected-file SHA-256 verification: 348 files unchanged, including the
  hybrid controller, both hi-hat sync modules, detector bridge, launchers,
  experiment/config files, recordings and ST7 models.
- AST comparison confirms `_process_sound_hit`, `_begin_strike_attempt`,
  `_finish_no_hit_attempt` and `_begin_test_striking` unchanged.

Backups: `before/`. Focused results: `boost_tests.log`,
`fractional_swing_simulation.log`, `ui_check.log`, `protected_verification.json`.
This is offline verification; the previous physical synchronization test
predates this depth boost and is not presented as physical evidence for it.
