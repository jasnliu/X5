# Experiment fresh startup acquisition — 2026-10-24

## Requested change

Normal `./plan_drums.sh experiment --right` and `--left` launches now acquire
one new planning scene plus new measured poses of both arms. The existing
five-native-depth-frame fusion and encoder-bracketing routine is reused. The
selected arm's existing stationarity check applies; passive-arm sway is not a
new stillness veto. Both arms must already report disabled motors.

The isolated startup process uses `read_only.Telemetry` and its exact CAN
state-query packet allowlist. It closes the camera and both encoder connections
before preparation/preview. Ordinary experiment workers still cannot open CAN.
No motor enable, disable, hold, relax, gripper-control or motion command is sent.

Startup failure does not load historical inputs. Explicit `--offline` or a
complete `--state`/`--scene` pair retains reproducible saved-input experiments,
with an OFFLINE label. `--capture-start` aliases complete fresh startup now.
Programmatic `app.prepare()` keeps its device-free study default; CLI explicitly
passes `fresh=True`. The existing Recapture scene button remains camera-only;
relaunch to obtain new poses as well. Scene and poses remain frozen within the
session, and destination selection still triggers calculation, not startup.

## Actual read-only hardware and native UI evidence

Three real normal CLI launches completed successfully:

| Session | Selected arm | Acquisition only |
| --- | --- | ---: |
| live-right-1 | right | 3.637 s |
| live-left-1 | left | 3.685 s |
| live-right-2 | right | 3.841 s |

These times are acquisition only, not full launch/calculation time. All three
have different scene/state/raw-depth hashes and nonoverlapping fresh camera
frame timestamps. Each saved scene contains five new frames; every frame has
recorded encoder bracketing evidence. Both arms in each initial backend request
match that launch's measured state. Camera and telemetry shutdown passed. CAN
audits contain only allowed state-query packets: 1,168 / 1,184 / 1,232 packets,
respectively. Setup/initialize jobs have no AF_CAN permission; only acquisition
jobs have it. No physical movement was requested or tested.

`audit_acquisitions.py` reproduces these checks; see
`acquisition_validation.json` and each session's `acquisition/` audits. The actual
CLI GUI reports show no selected destination, no planning job, zero generated
trajectories at startup. A separate explicit offline CLI launch opened the GUI
without acquisition jobs or automatic calculation.

Native preview tests reopened the just-acquired right and left sessions,
verified the FRESH STARTUP SNAPSHOT label, selected a destination, and verified
that measured state/scene hashes and the retained scene mesh stayed unchanged
through calculation. `startup_ui.png`, `calculated_ui.png`, and
`startup_ui_check.json` are stored per tested session. The startup screenshot
precedes the final cosmetic wording change from Saved resting pose to Frozen
start pose.

### Current-scene route results are not a planning success claim

- Right `destinationR1`: existing generator returned
  `no_workspace_proposal_within_budget`; proposals were rejected by its existing
  coarse collision/zone checks. Endpoint checks and joint-limit check passed.
- Left `destinationL1`: `nominal_endpoint_invalid`, with collision reported
  between `gemini_native_depth` and `left_drumstick`. Joint-limit check passed.

The application displayed these no-route results rather than a stale cached
trajectory. No path/timing changes or relaxed checks were made to force a pass.
No valid-trajectory playback or physical feasibility is claimed by these tests.

## Software verification

- Focused startup/CLI/failure/cleanup/query-only/current-capture suites:
  **168 passed**, in `tests-startup-final.log`.
- Broad experiment/timing/hardware-regression run: **334 passed, 5 failed**;
  see `tests-regression-final.log`. All five failures are historical source-hash
  guards rejecting the
  already-changed `destinations/right/destinationR1.json` against old study
  snapshots. Its task-start and task-end SHA256 are both
  `2e6e1d08fae349ce8a81943ddd0c617db047116a37f291bb0f28d4559508ecf0`;
  old studies expect
  `bd3b74c97588c948ab99118ae052c4c7910eab0ca48a3909a32ba5bd838c43a2`.
  The destination was not edited, restored, or rebaselined by this task.
- Syntax and `git diff --check` passed. CLI help documents the fresh/default and
  explicit offline behavior. Error tests simulate acquisition/feedback/cleanup
  failure and interruption; they do not deliberately fault physical hardware.

`before.json` and `preservation.json` record the task-start 364-file source/data
snapshot and allowed changes: experiment startup CLI/app wiring, documentation,
and related tests. New files are the experiment-only acquisition helper and its
unit tests. Path generation, timing, motion limits, model/transport/control code,
destinations and production hardware programs were not changed. Historical
preservation tests were adjusted only to permit the authorized CLI/docs/test
updates, not to waive the destination mismatch or algorithm preservation.
