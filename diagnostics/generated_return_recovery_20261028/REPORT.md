# Generated beat Return recovery — 2026-10-28

## Scope and result

Fixed normal `start_beat.sh --hardware` Return without changing its outbound
planner, destinations, camera calibration, audio models, joint/drive settings,
collision margins, or the `plan_drums.sh` controller. All verification here was
**software-only**. No CAN devices, cameras, microphones, ESP32 connections,
physical arm enable/hold/disable, or physical movements were used.

Return uses the cached checked route where possible. If unavailable, it checks a
measured connector/prefix; if that fails, it uses the existing workspace planner
and Balanced timing to calculate RIGHT then LEFT from measured Hold to the
**original captured rest**, with the other arm's actual/returned pose. It never
substitutes the historical center. Arms stay powered while calculating. Existing
native full-tool collision/zone checks and joint constraints remain in force.

## Reproduced failure chain

1. **Parent GUI blocked while starting the trajectory child.** Multiprocessing
   previously sent the large curve/geometry payload through its bootstrap pipe.
   `Process.start()` could wait for the child's imports/unpickling. The *existing*
   powered MIT strike workers still needed GUI heartbeats every 500 ms, even
   though the new trajectory worker had its own ready/acknowledgement handshake.
   Actual Tk replay with simulated held-owner watchdogs reproduced a right-owner
   heartbeat failure immediately after return startup. See
   `heartbeat-reproduction.log` and `heartbeat-reproduction.json` (expected fail).
   This is consistent with the terminal's `Hybrid GUI heartbeat lost` followed by
   repeated encoder failures. It is not evidence of faulty physical encoders.
2. **Permanent false feedback ownership after worker exit.**
   `Joint7Session.owns_feedback` accepted any retained sample, including a dead
   worker's last one. `Motors.poll` then omitted right J7 read-only queries forever.
   The real polling code reproduced `Encoder feedback lost` with fake devices.
   Before the fix, three of four initial regression cases failed; see
   `before-reproduction.log`.
3. **No usable fallback.** Return required the cached path/nearby pose; a stopped
   MIT owner could not hand back control. Repeated error text could also prevent
   a new moving writer from being halted after retry.

The initial failure message alone does not prove every individual physical
incident had this entire chain. These are reproduced software faults, not a
claim that hardware failures are impossible. `last_fault.json` now records the
phase and each joint's feedback age/state for future diagnosis.

## Changes

- Feedback ownership requires a live process and a sample younger than 100 ms.
  Parent state queries resume before the unchanged 300 ms feedback watchdog;
  this does not transfer command ownership.
- Large immutable worker inputs use a private mode-0600 local IPC file rather
  than the multiprocessing startup pipe. Child load/cancellation removes it.
  Heavy new-return Curve construction/validation stays in the child. No watchdog
  extension and no background heartbeat disguising a frozen GUI.
- A single explicit Return survives temporary feedback loss and proceeds when
  healthy stationary Hold is reestablished. Hold cancels that intent. A repeated
  identical fault stops a newly started writer as well as the original one.
- For explicit Return, a dead MIT writer is joined before a Hold/return-only
  replacement. It reads actual MIT mode and fresh running feedback first; no
  enable, disable, drive-mode change, strike or generic center is performed.
  Existing healthy writers are retained. A failed replacement is reported rather
  than endlessly respawned. Existing Hold follower/gains and signs are reused.
- Missing cache / off-path feedback invokes the same bounded planner child, with
  CAN prohibited and the existing 3 GiB memory cap. Fresh paths use conservative
  half-speed recovery limits (in these cases V=0.3, A=0.3, jerk=0.75 per joint).
  Actual scene self-masking remains at the original acquisition poses.
- Tiny measured right-J2 offsets below acquired rest use the same private,
  checked connector allowance already present in baseline recovery. No feedback
  clipping, lowered physical stop, or edit to the original model/plan.
- Peripheral cleanup failures cannot prevent arm Hold. Failed/cancelled planning
  clears the job for retry; late results cannot start motion. Explicit emergency
  torque-off/Stop remains unchanged and always accessible.

## Fresh verification

### Automated regressions

**279 passed** in `regression-tests.log`: generated sequence/controller/transport,
new recovery cases, real MIT loops on fake channels, left/right sign handling,
worker ownership, startup, Stop/Relax, and both-arm encoder display regressions.
Final focused recheck: **45 passed** in `targeted-tests.log` (subset, not additive).

New cases cover live/stale/dead feedback ownership, re-query recovery without
reenabling, exclusive dead-writer replacement, no competing healthy writer,
wrong MIT mode, disabled/stale/faulted feedback, bounded retries, pending Return
and cancellation, repeated faults, missing cache, failed planning/retry, stale
result rejection, and nonblocking large cold worker startup. Fake channels
assert that recovery emits no enable/disable/mode-setting frames.

### Actual Tk and shared playback controller

- `existing-ui/ui_report.json`: **10 checks passed**, including full left→right
  outbound, cached full/partial returns, real 200 Hz delegate, intentional active
  GUI stall, Hold/retry, startup cancellation, stale-feedback Relax, and Stop.
- `recovery-ui/ui_report.json`: **7 checks passed**, including a Return click
  during repeated encoder errors, identical fault during a new return, Hold
  cancellation of pending intent and calculation, missing cache, fresh replanning,
  right→left execution, and release at captured rest.
- Held J7/J6 owner heartbeat deadline models stayed alive through actual GUI
  planning/startup: **maximum gap 0.068199 s**, unchanged limit **0.500 s**.
- The new GUI regression failed with the old large-pipe startup behavior and
  passed after the fix. It does not replace feedback-fault checks with mocks that
  always return healthy: failures are injected and must stop advancement.
- Both GUI suites used ideal motors and prohibited physical CAN. Every played
  segment reported executor `plan_drums.baseline.motion.Controller`.

### Native collision checking on actual saved camera data

Primary saved mission:
`planner_runs/generated-beat-20261028T165028-904eb7ef/plan-443f6066/mission.json`.
No camera recapture and no physical motion. Held goal poses are test inputs, not
claims of current hardware position.

| Software scenario | Result | Native samples + edges | Calculation wall time |
|---|---|---:|---:|
| Return-cache file deliberately absent in a copied test manifest | New checked measured prefixes, right→left | 6,504 right + 8,432 left | 7.03 s |
| No usable progress/prefix (saved goals, progress metadata set to zero) | New workspace paths, right→left | 2,993 right + 4,016 left | 19.86 s |
| Held right J2 0.02° below a negative captured rest | Exact measured connector accepted, original model unchanged | 352 right + 352 left | See `measured-j2/job/job.json` |

All native proofs passed. Fresh routes lasted **5.983222 s right + 8.029845 s
left**; these are movement durations, not calculation time. The actual GUI rerun
independently recalculated/checks/played both fallbacks: 7.57 s for prefixes and
18.67 s for new routes. See `native_report.json`, `measured-j2/return.json`, and
paths in `recovery-ui/ui_report.json`. Cancellation was also exercised while
planning; it did not release the arms or queue delayed motion.

`preservation-audit.json` verifies original mission input hashes, acquisition
self-mask, held starts, captured-rest endpoints, opposite-arm geometry, collision
margins, CAN-denied/3-GiB containment, and removal of temporary IPC inputs.
`before/` preserves the task's starting source; `changes.diff` records this change.

## Limits of the result

This removes reproduced software dead ends; it cannot guarantee physical return
with genuinely absent feedback, lost power, a drive fault, an obstructed route,
or failed hardware. A frozen scene is not live obstacle avoidance. Existing
freshness, stationary feedback, collision, zone and mechanical-limit safeguards
were not bypassed to produce passes. No physical tracking/torque/clearance
certification is claimed. A separately authorized hardware test is still needed
for physical acceptance.
