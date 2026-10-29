# Generated `start_beat.sh --hardware` — software verification

## Scope and authorization

Implemented normal hardware startup with a fresh paired camera/rest snapshot,
left saved destination followed by right DT1, no initial centering, and no
pink-zone alignment. The existing acoustic ride search, ride/hi-hat timing,
hybrid swing, tempo/bass selection and powered random snare pattern remain.
See [operator instructions](../../GENERATED_BEAT.md).

**No hardware tests were run.** No live camera, microphone, serial/ESP32 or CAN
connection was opened. Native planning used a previously captured real scene;
motor tests used fake transports. Native windows and the actual shell launcher
were exercised in the explicit device-free simulation mode. This does not
establish physical tracking, clearance accuracy, real acoustic calibration or
motor performance. Hardware acceptance requires separate permission.

## Implementation

- `camera_playback/generated_acquisition.py`: existing paired five-frame camera
  acquisition with both resting arms stationary and disabled, query-only. Runs
  independently of the existing startup hi-hat calibration; camera and temporary
  encoder ownership close before the control panel adopts the normal motor owner.
- `generated_planning.py`: serial current workspace/recovery generator and
  Balanced per-joint timing for both arms, including dense exact-curve checks.
  Left uses the sole saved left JSON; right regenerates DT1 on the same image
  with the left arm at its predicted checked goal. No recording interiors or
  calculated-route cache are used.
- Optional `both_tools` and `scene_robot_poses` inputs in the existing model,
  request adapter and native checker include both sticks and keep BOTH camera
  masks at the original acquisition poses. The passive arm's collision geometry
  still uses the actual pose for each leg. Defaults for ordinary planner calls
  are unchanged.
- `generated_app.py`, `generated_motion.py`, `generated_transport.py` reuse the
  existing motor owner, conversion conventions, exact Curve, PlaybackFollower,
  bounded MIT return controller and calibrated strike workers. No initial
  centering or generic gripper sweep. Closed custom grips must settle first.
- Normal hardware alone is routed through this app. Recording selectors and
  pink-zone hill climbing are absent. `--test`, `--hardwaretest` and
  `--recording-only` retain the prior app and recording behavior.
- Emergency Hold sends ESP32 `S`, cancels scheduling and asks each MIT worker to
  latch its own newest feedback; stale GUI feedback cannot block that request.
  It never intentionally disables either arm. Faults report when hold is
  unconfirmed rather than pretending the pose was safe.
- Return is explicit, first establishes stationary hold, checks retraction and
  the right-then-left traveled prefixes against the frozen scene, and releases
  only after both original rests are stationary and feedback confirms disabled.
  Repeated Hold cancels a pending/active return. Window close while active holds
  and leaves recovery available.
- `generated_launch.py` removes ROS launch's timed SIGTERM/SIGKILL escalation
  for the holding controller only. A real ROS shutdown test with a fake owner
  verified survival past both configured escalation deadlines; perception
  processes retain ordinary cleanup. Generated J7 workers leave Ctrl-C handling
  to the panel. Hard kills/power failure are not recoverable by this promise.

## Fresh numerical calculation on saved real inputs

Source capture (not recaptured during this task):

`planner_runs/20261028T023247Z-hardware-right-ae273b/calculate-7f013e4d/capture`

Results: [verified/mission.json](verified/mission.json), [calculation log](verified.log).

| Leg | Generated movement | Native samples plus connecting edges | Result |
|---|---:|---:|---|
| Captured left rest → saved L1 | 3.764860 s | 1,884 | All valid |
| Captured right rest → fresh DT1, left held at L1 | 2.743720 s | 1,373 | All valid |
| Total movement only | **6.508580 s** | | |

Fresh DT1, route generation, timing and dense checking took **29.435672 s** on
these saved inputs, without reusing a generated-result cache. This excludes live
camera acquisition and native binary compilation; it is not an end-to-end live
cold-start measurement. Movement times exclude arming, settling, hi-hat waiting
and acoustic calibration.

Limits/margins unchanged: 0.6 rad/s, 1.2 rad/s², 6 rad/s³ defaults; 0.8 rad/s
drive cap; baseline 20 mm world clearance plus existing 5 mm edge allowance;
occupied voxel volumes counted once; original side-specific zone membership;
right J2 rest-transition/operating policy and DT1 goal reserves retained.

## Tests observed in this task

- [regressions_verified.log](regressions_verified.log): **308 passed** in 56.32 s.
  Covers generated motion/acquisition failure contracts, both legacy recording
  paths, playback transport, hybrid workers/workflow, snare worker ownership and
  handoff, hi-hat calibration/sync and single-run controls.
- After final launcher/Hold changes, [hold_final.log](hold_final.log): **41
  passed** in 17.03 s. This overlaps the preceding suite; counts are not additive.
  Actual MIT worker loops run on fake channels, including worker-local feedback
  capture, signed left-J6 return, no wire NaN, no mode/enable/disable frames on
  Hold, and fault holding outside the old strike-only corridor without clipping
  back toward it. Narrow MIT setup exemptions never exempt other stopped drives.
- Final targeted rerun [final_scope_check.log](final_scope_check.log): **146
  passed** in 27.54 s (overlapping suites). Includes the internal calibration/
  sync-stop path: generated mode latches Hold and retains MIT ownership instead
  of using the legacy join-and-auto-center behavior. Also verifies the real
  generated J7 worker leaves Ctrl-C handling to its owning panel.
- [planner_regressions.log](planner_regressions.log): **249 functional tests
  passed; 3 historical source-hash assertions failed**. The failures compare
  `model.py` / `planning.py` to Oct-21 source snapshots. Both already differed at
  this task's start (see preservation audit); this task also intentionally adds
  the optional two-arm model/mask support. Historical expected hashes and tests
  were NOT altered to hide these failures.
- [ui_verified.log](ui_verified.log), [ui_report.json](ui_report.json): real Tk
  buttons, no recording selectors, explicit Start, partial-left Hold, active
  window-close Hold, one-click checked partial return, full left→right playback,
  actual ride-search handoff with fake worker/audio/ESP32, first detected depth
  plus original 0.5° swing offset, and full-goal right→left checked return passed.
  Full return checked 6,036 right and 7,961 left samples plus edges. The synthetic
  audio test intentionally does not claim physical sound calibration.
- [preview_report.json](preview_report.json): actual native Qt/VTK preview and
  measured-pose windows render the frozen real mesh and both custom tools with
  no rendering errors; scrub/play/pause leave the measured rest unchanged.
  Images: [generated goals](generated_goals.png), [captured rest](captured_rest.png),
  [control panel](control_panel.png).
- [launch_hold_report.json](launch_hold_report.json): actual ROS shutdown with
  a fake owner passed the no-timed-kill check. [launcher_simulation_final.log](launcher_simulation_final.log)
  shows the real `start_beat.sh --generated-simulate` launcher opening and closing
  cleanly on Ctrl-C, without device subprocesses.
- `git diff --check`: clean. Earlier failed development tests are retained in
  this directory; the logs named above are the accepted reruns.

### Recovery limitation verified, not bypassed

An additional native test stopped both strike joints 5° below their generated
prime positions. The **left stick already intersected the captured drum surface
at sample 0**, so the full collision checker rejected the retraction. This is
recorded in [midstrike_return_report.json](midstrike_return_report.json) and
[midstrike_return.log](midstrike_return.log). No contact exemption or unchecked
motion was added. Therefore Return is available but **is not guaranteed from a
mid-strike/contact pose**; it can require operator intervention. Partial-path
and clear prime-pose returns passed. A separate approved contact-aware recovery
policy would be needed to permit a withdrawal from intentional drum contact.

## Preservation

[before.json](before.json), [git_before.txt](git_before.txt), and
[preservation.json](preservation.json) distinguish existing user changes from
this task. All eight snapshotted recordings/destination JSONs and `start_beat.sh`
are byte-identical. Calibration still matches the frozen verified capture.
Underlying motor conversion, playback ownership, strike tuning, hi-hat firmware,
raw captures and vendor URDF were not edited. Shared checker/model extensions
are optional; no planner settings or physical joint limits were loosened.

## Reproduce without hardware

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
.venv-planner-experiment/bin/python -m pytest -q \
  tests/test_generated_beat.py tests/test_generated_worker_hold.py
PYTHONPATH="$PWD:$PYTHONPATH" /usr/bin/python3 tests/generated_beat_ui_check.py
PYTHONPATH="$PWD:$PYTHONPATH" /usr/bin/python3 tests/generated_beat_launch_check.py
PYTHONPATH="$PWD:$PYTHONPATH" .venv-planner-experiment/bin/python tests/generated_beat_preview_check.py
./start_beat.sh --generated-simulate diagnostics/generated_beat_20261028/verified/mission.json
```

Use a new output directory for fresh numerical preparation as described in
`GENERATED_BEAT.md`. The normal `--hardware` command is deliberately NOT part of
the software smoke tests: it opens devices and starts existing hi-hat calibration.
