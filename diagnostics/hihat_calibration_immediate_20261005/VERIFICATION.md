# Immediate automatic hi-hat calibration startup

Removed the calibration-specific ntfy notifier, worker thread, notification result gate, and 10-second countdown. Startup now begins on the first ready tick after the controller, compatible firmware telemetry, and hi-hat detector are ready. No button press is required. Readiness checks and startup timeout remain.

Unchanged: 90 through 115 degrees in 5-degree steps, 2-second closed/open holds, fault after the 115-degree miss, learned swing angle, arm wait before ride calibration, center-before-relax fault handling, ride behavior, hi-hat timing adjustment, UI layout, firmware, models and data.

## Verification

- Focused hi-hat tests: 71 passed (`focused_tests.log`).
- Full offline regression suite: 571 passed in 136.612 s (`full_tests.log`).
- Actual Tk normal-hardware App wiring with mocked devices: automatic startup and calibration selection succeeded with no RUN press; first opening command 0.059 s after simulated detector readiness. Network notification calls were forbidden in this test. Synthetic hit at 95 degrees selected 95, arm remained disabled, app returned READY (`ui_verification.json`, `ui_calibrated.png`).
- 523 protected files verified unchanged, including firmware, calibration engine, application, ride/timing code, models and collected recordings (`protected_verification.json`).
- No physical device I/O, motor movement, notification or firmware flash was performed for this change. Mocked UI evidence is not a physical calibration test.

`before/` contains the four pre-edit files; `changes.patch` records the exact changes. Historical diagnostics are preserved.
