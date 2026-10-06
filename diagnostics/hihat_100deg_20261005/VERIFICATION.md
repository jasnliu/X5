# Hi-hat closure target: 100 degrees — 2026-10-05

## Implemented
- X5 ESP32 beat C and keyboard J closure targets: 110 -> 100 degrees (149 encoder counts).
- Open target remains encoder zero. Existing gains, return-speed setting, heartbeat, direction flags and command protocol retained. Target-derived safety bounds follow the new target automatically.
- Host status, future hi-hat collection configuration, tests and documentation synchronized.
- Ride depth, ride timing and hi-hat adaptive timing modules unchanged. Historical recordings, labels, model files and capture metadata unchanged.

## Checks
- Firmware compiled successfully for esp32:esp32:esp32; 284875 bytes sketch, 22824 bytes globals.
- Full offline suite: 535 tests passed. Focused serial tests: 8 passed.
- All 518 protected files matched before/after SHA-256 hashes.
- ntfy warning delivered before any hi-hat motion, followed by 10 seconds warning time.
- Hi-hat received OPEN with 1.5 seconds of serviced return time, then STOP acknowledged. No arm access or motion.
- Uploaded with verified flash hashes to the connected ESP32-D0WD-V3, revision v3.1.
- Fresh startup readback: `Motor2 targets: keys 100 deg=149 counts, beat 100 deg=149 counts`.
- Final STOP acknowledged; serial connection closed with no remaining owner.

## Evidence boundary
This verifies configuration, firmware deployment and protocol acknowledgements. No closing command was sent during verification, and actual output-shaft travel was not independently measured. The open preparation was command/settle/ACK-based, not encoder-position readback. No full beat or robot-arm test was performed for this angle-only change.

Evidence: notification.json, rest_preparation.json, compile.log, full_tests.log, serial_tests.log, upload.log, firmware_verification.json, protected_verification.json and changes.patch. Original edited files are in before/.
