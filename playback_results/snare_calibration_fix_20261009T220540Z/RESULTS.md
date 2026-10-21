# Snare calibration stop fix and physical acceptance — 2026-10-09

## Root cause and changes

The snare runtime sampled `now`, then polled its detector socket. Receiving a
heartbeat timestamps it with a later `time.monotonic()`. `ready(now)` therefore
saw a negative age and falsely reported the healthy detector unavailable. A real
Unix-datagram regression reproduces the original exact error (see
`heartbeat_reproduction.log`). The runtime now polls before taking the time used
for health checks. Heartbeat freshness checks and all safety limits are unchanged.

Snare calibration stops at 15 seconds from stage startup, including the initial
one-second scheduling lead. A timeout is explicitly uncalibrated, prevents swing,
and uses the existing full return -> verified center -> relax path. Motor or
actual detector faults remain faults, not accepted timeouts. Automatic stops now
show their cause; unavailable detectors identify their instrument/status.

An opt-in `--verify-calibration` hardware acceptance mode invokes the normal Run
button and stops after a real snare lock or the ordinary timeout, without swing.
It does not fake sound/camera/encoders or alter stroke tuning.

## Physical run — PASS under the user's timeout criterion

Command: `./start_beat.sh --hardware --camera /dev/video4 --verify-calibration`

- Read-only initial query confirmed all 16 drives disabled; camera frame inspected.
- Ten-second motion warning issued before launch; no competing controllers.
- Normal startup centered both arms, prepared snare mode at center, played both
  recordings, confirmed the pink zone, and performed the real ride-depth search.
- Ride detected at 10 degrees; normal +0.5 degree boost retained.
- Ride/hi-hat timing locked at +96.779 ms after two consecutive good pairs.
- Ride/snare timing ran for **15.010969 seconds** and completed **19 powered 11
  degree strokes**, each with its full return (measured peaks 10.70–10.86 degrees).
- No false detector-unavailable stop and no application/motor faults.
- Snare timing **did not lock**: 2 valid pairs, 16 skipped/ambiguous/missing pairs.
  This verifies the requested bounded timeout path, NOT acoustic accuracy or a
  learned snare offset. No uncalibrated swing was played.
- On timeout, strikes stopped, the left recording reversed, both arms centered,
  both final relaxation guards passed, and fresh feedback confirmed all 16 drives
  disabled. All launch processes exited cleanly, exit status 0. No intervention,
  emergency relaxation, or controller takeover was needed.
- Passive CAN: 120,072 feedback frames, zero motor fault frames, maximum 36 C.

### CAN audit qualification

The generic collector audit (`diagnostics/snare_data_collection/audit_can.py`)
assumes every individual disable is a whole-arm centered relaxation. It flagged
one right-J7-only disable at the recording endpoint. Inspection confirmed the
**existing, unchanged** hybrid MIT-entry sequence in `mit_strike.py:_prepare`:
disable -> confirmed mode write -> enable in 35.038 ms, then running feedback.
It occurred before search, not at timeout/return. It is explicitly retained in
`run1/existing_j7_mode_handoff.json`; do not describe this run as having zero
off-center individual-joint disables. Final arm relaxations were independently
center-verified. `audit_run.py` checks this exact setup sequence and rejects any
other unproven disable; the original raw audit is preserved.

## Regression and preservation

- Full offline suite: **911 passed in 238.35 s** (`full_tests.log`).
- 540 protected recording/model/motion files unchanged (`protected.json`).
- Tested production-source hashes match the physically verified versions
  (`tested_source.json`). Previous snare path tuning, 11-degree powered stroke,
  ride controller, model weights, thresholds and recording files are unchanged.
- Both complete physical record/return endpoints and the final disabled state
  were independently checked; see `run1/summary.json` and `audit_run.py`.

## Evidence

- App/CAN logs: `run1/app.log`, `run1/can.log`.
- Physical telemetry/result:
  `../calibration_verification/20261009T221240-1cf1de4c/`.
- Snare timing:
  `../snare_sync/20261009T221415-15ef7f7c/timeline.jsonl`.
- Source backup/diff: `before/`, `changes.diff`.
