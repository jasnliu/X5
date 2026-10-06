# Automatic hi-hat calibration — 2026-10-05

## Implemented behavior

- Normal `start_beat.sh --hardware` starts hi-hat calibration without any button press. It waits for the actual hi-hat detector and calibration-v2 firmware, sends a confirmed ntfy warning, then waits 10 seconds before motion.
- Search is exactly 90, 95, 100, 105, 110, 115 degrees. Both host and firmware reject configured angles outside 90..115. No 120-degree attempt and no fixed-angle fallback.
- Each attempt reaches its encoder target, holds closed for at least 2 seconds, returns/releases, settles, and stays open for at least 2 seconds. An early detection never shortens these intervals.
- The first real hi-hat-model HIT whose acoustic onset belongs to a closure selects that angle, without a boost. Delayed delivery is handled using onset timestamps and ST7 finalized-audio progress. Old, opening, ride-tagged, invalid and future events cannot select an angle.
- Finishing the 115-degree attempt without a HIT is a latched fault. Missing acknowledgments, stale encoder data, model failure, missing audio progress and failed movements are faults too. Restart is required after a cancelled/failed calibration.
- RUN can center/close/play the arm recording concurrently. At the endpoint, ride calibration is gated until hi-hat success. Waiting preserves the normal feedback/stall checks, but a successfully held pose is not subject to the normal movement-only 30-second deadline. Once ready, the existing camera alignment is freshly checked before ride search resumes.
- The existing arm fault/stop paths remain center-before-relax. Hi-hat cleanup is independent; closing the application waits for its bounded return even if the arm never ran. Serial disconnection cannot trap the software in cleanup indefinitely.
- Swing uses the learned hi-hat angle. Hi-hat advance, opening/closure beat numbers, ride search, ride +0.5-degree boost, ride timing, and detector/model files are unchanged.
- No added buttons or layout changes. Existing status text now describes calibration. Synchronous recording preflight is blocked during calibration movement to avoid starving serial supervision, including a modal file-picker race.
- `--test`, `--hardwaretest`, offline preview and recording-only mode do not automatically calibrate.

## Firmware / compatibility

- Added non-moving `Q` protocol/encoder readback; bounded acknowledged `A<degrees>\n` configuration; `B` calibrated closure. `O` keeps the existing zero-return/release behavior.
- `C` and `J` remain fixed at 100 degrees, so legacy keyboard/collection commands do not inherit a previous learned angle. Motor gains, pins, half-speed return and 400 ms watchdog remain unchanged. A new 2-second movement timeout prevents an unfinished calibration move from driving forever; motor faults cannot be cleared by another beat command.
- Calibration requires the v2 readback and fails closed with older firmware. Startup itself never flashes the board.
- Successfully compiled, flashed and read back on the connected ESP32-D0WD-V3, revision v3.1. Upload hashes were verified. Final image: 286715 bytes sketch, 22864 bytes globals.
- 115 degrees is the maximum commanded target (172 counts), not a promise of zero encoder quantization or physical overshoot.

## Actual hardware evidence

The production calibration runtime and serial controller were exercised with both **unmodified ST7 ride v2 and hi-hat v1** listeners on TONOR. The test had no arm/CAN access (audit hook rejects CAN creation), no synthetic audio events, no independent sound labeling, and no forced angle selection.

Final live test (`physical_result.json`, `physical_summary.json`, `physical_events/events.jsonl`):

- ntfy warning delivered; first OPEN command followed **10.0017 s** later.
- First tested angle: **90 degrees** (134 encoder counts).
- Actual hi-hat-model HIT: score **0.99836**, normality **90.4**.
- Acoustic onset **578194.243675289**, detector notification **578194.848931595** (monotonic seconds).
- Verified closed hold: **2.0090 s**. Verified open dwell: **2.0081 s**.
- Selected **90 degrees**; reached READY with **no faults**.
- Confirmed zero-return arrival, released/open state and settled encoder (final rest position **1 count**). STOP acknowledged, serial closed, no remaining serial owner or detector processes.
- Robot arm was **not moved**. This is physical hi-hat calibration evidence, not a new full physical swing-beat or physical arm-wait test.

### First test finding and correction

The first live test also detected the 90-degree closure, but exposed an incorrect new assumption in the return check: existing firmware releases at zero, after which the passive mechanism can coast. It was observed settling at -19 counts, so demanding that it keep holding exactly zero falsely faulted. The test stopped safely; no arm was involved. The firmware now reports a real zero-arrival latch, cleared by each close/open operation and set only upon the controlled zero return. The host verifies that latch, released state and encoder settling without altering the motor's return behavior, retuning the model, fabricating a return, or resetting the encoder during calibration. Regression tests cover passive coast and reject release without a verified return. Original evidence is preserved in `physical_run1/`.

## Offline / UI evidence

- Full regression suite: **571 tests passed** in 137.965 seconds (`full_tests_final.log`).
- `focused_tests_final.log` and `integration_tests.log`: deterministic search, delayed hits, all six misses/115-degree fault, success at 115, bad/stale feedback, missing acknowledgment/progress, cancellation, unplugged serial, arm gating/resume, long stationary hold, retained stall detection, and file-picker safety.
- The actual ESP32 `.ino` is compiled into a native C++ harness testing the parser, angle bounds, legacy 100-degree commands, return latch, movement timeout and fault latching; not a reimplementation of the parser.
- Actual normal-hardware App/Tk startup wiring was exercised with in-memory motors and synthetic test audio. It calibrated automatically at 95 degrees without a button press and left the simulated arm disabled; `ui_verification.json`, `ui_check.log`, `ui_calibrated.png`. This UI test is explicitly offline, distinct from the physical ST7 evidence above.
- 518 protected files are SHA-256 unchanged, including ride controller/workflow, timing synchronizer, audio bridge, model assets, recordings, datasets, configuration and sibling ESP32 project files. `protected_verification.json` documents the check.

## Review artifacts

`changes.patch`, `before/`, `source_hashes.json`, `compile.log`, `upload.log`, `firmware_verification.json`, `notification.json`, `rest_preparation.json`, and the evidence named above. No training, dataset relabeling or model replacement was performed.
