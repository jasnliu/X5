# Generated dual-arm beat startup

Normal `./start_beat.sh --hardware` now uses generated paths instead of recordings.
The arm can be off while software is being edited/tested; an actual run needs
fresh encoder feedback from both arms before it can prepare or Start.

For a **fresh calculation without any actuation**, including no hi-hat
calibration, use `./start_beat.sh --hardware --prepare-only`. This runs the same
query-only acquisition and left/right/return planning workers, prints the saved
mission and a software-preview command, then exits. It never constructs the
motor controller or opens ESP32/audio. Both arms must already be resting and
disabled; this option cannot enable or disable them. Failure does not reuse an
old scene. Normal `--hardware` behavior is unchanged.

## Operator workflow

1. Launch `./start_beat.sh --hardware` with the robot resting, arms initially
   disabled. **The existing ESP32 hi-hat startup calibration runs automatically.**
   In parallel, one paired RGB/depth snapshot is captured (the same five-frame
   fusion and calibrated Orbbec serial as the planner), bracketed by both arms'
   stationary encoder measurements. Neither arm is enabled or centered.
2. The camera and temporary query-only CAN owner close. The normal arm owner
   takes over feedback; one bounded, separate calculation process generates:
   - **Left:** actual rest to the sole JSON destination in `destinations/left`.
   - **Right:** actual rest to a new **DT1** cymbal prime destination from that
     very same image, accounting for the left arm already being at its goal.
   No old scene/destination fallback and no recorded interior motion are used.
   The **right-then-left return is also prepared and collision-checked now**,
   stored with checksummed mission inputs before Start becomes available.
3. Inspect the disconnected generated preview and measured-arm window. Both use
   the same frozen camera mesh and calibrated custom-tool model. Preview
   play/pause/scrubbing cannot move the real robot or modify execution speed.
   The control panel shows **right and left J1–J7 encoder angles in degrees**,
   plus each gripper's existing raw motor-8 count/angle telemetry. These use the
   current feedback snapshot, not the planned destination or preview pose; no
   additional device connection or motor command is introduced by the display.
   The preview includes outward left→right, a brief goal pause (preview only),
   and the cached return right→left to the captured rest. It does not mean the
   hardware automatically stops the beat after that preview pause.
4. Once paths, sound, and the hi-hat controller are ready, click **Start**. Both arms
   are configured at their captured positions, with their established closed
   gripper targets. **No centering and no opening sweep.** The checked left
   trajectory plays first, settles, then the checked right trajectory plays.
5. If startup hi-hat calibration is still running, the arms hold at their goals
   until it finishes (as before). The left snare worker is prepared at the stationary generated left goal.
   The normal ride J7 sound search starts directly at the right prime pose:
   5 degrees, then one-degree increments, first hit plus the existing 0.5-degree
   swing offset. Existing ride/hi-hat timing calibration, tempo/bass controls,
   and the original powered 11-degree random snare rhythm follow unchanged.
   **There is no pink-zone acceptance or hill-climbing adjustment.**

The current workspace/recovery path generator and Balanced CPU timing are shared
with the latest experiment/hardware planner, for both arms. Defaults remain
0.6 rad/s, 1.2 rad/s², 6 rad/s³ and the existing 0.8 rad/s drive cap. Right J2
operating/rest policy, generated-goal reserves, DT1 geometry, original envelope
membership, calibrated full-tool collision geometry and approved margins remain.
A left DT1 generator is **not** invented: the left endpoint is the saved manual one.
Right DT1 uses the operator-approved **±25° inward aim** (destination version 8).
The green point and all joint reserves/limits remain unchanged; this restores
feasibility for the 2026-10-28 target that exceeded the former ±20° aim window.

## Exact-path 200 Hz playback

Generated outward motion and the prepared return run the **same
`drum_planner.baseline.motion.Controller` used by `plan_drums.sh --hardware`**,
through a small dual-powered-arm/sequence adapter in `generated_controller.py`.
There is no separate generated trajectory-following algorithm. Its `step`,
`send`, exact curve/recovery evaluation, discontinuity guard and measured-pose
fault Hold are reused. The parent retains left→right sequencing, settling,
percussion handoff and the prepared right→left return.

The controller runs in the existing isolated **200 Hz position delegate**,
not Tk's 20 ms callback. Absolute monotonic deadlines prevent GUI work
from stretching the command schedule. A delayed tick evaluates the current time
on the original curve; it does not burst through missed targets. The existing
80 ms missed-command guard runs in the writer. GUI heartbeat loss stops advancement
and attempts a fresh measured Hold, never an automatic return or relaxation.

Initialization uses an automatic **initialize → ready → GUI acknowledgement**
handshake, without another operator button. Import/unpickling, curve validation,
and read-only feedback/mode startup finish before the path clock or any moving
target starts. Initialization has a separate 10-second deadline. The original
**0.5-second active heartbeat watchdog is unchanged**, and starts only after the
GUI acknowledges readiness. Cancellation and parent-process loss remain checked
during initialization as well. A ready worker without acknowledgement does not
start moving. This fixes the cold-spawn race that previously reported a lost GUI
before sending even the first trajectory command.

Return startup also protects the **already-powered J7/J6 owners'** unchanged
500 ms heartbeats. Large curve/geometry inputs now travel through a private
temporary worker-input file, not the multiprocessing bootstrap pipe: the latter
could block Tk in `Process.start()` while the child imported Python modules.
The child consumes/removes that local file; cancellation removes any remainder.
New-return curve construction/validation also stays in the child, rather than
blocking the control panel for over a second. This is an IPC/scheduling fix, not
a longer watchdog or a background heartbeat hiding an unresponsive GUI.

This is **execution scheduling only**: no filtering/refitting, new path, retiming,
gain changes, speed/current changes, or mode switches. The exact saved polynomial
and monotone time law are reused, including the existing prepared return law.
Nominal movement durations are unchanged; process startup and endpoint settling
are separate. The pre-existing J7/J6 MIT owners retain their control loops and
settings during return; the path writer supplies their checked target references.

The GUI retains ownership and enabling/relaxation. Its competing position writes
are blocked while the delegate owns an arm. Hold joins the delegate and records
the latest traveled prefix before recovery. Emergency Relax/Stop also stop the
delegate before torque-off and retain their unconditional operator controls.
Feedback, tracking, calibrated bounds and original envelope checks also run in
the writer; frozen-scene collision validation remains unchanged.

Each movement logs `stream-*/trace.csv` and `result.json` inside its generated
session folder. Hardware workers additionally log CAN commands and feedback.
Results also identify the shared controller, ready/acknowledgement times, and
largest running heartbeat age, distinguishing startup from an actual GUI stall.
The software simulator uses the same spawned scheduling/evaluation loop with
explicit fake motors and a CAN-opening prohibition. This change is confined to
normal generated `start_beat.sh --hardware`; `plan_drums.sh`, recording playback,
planners, timing optimizers, and percussion/calibration algorithms are unchanged.

Software verification: `diagnostics/generated_stream_200hz_20261028/REPORT.md`.
Controller-reuse/startup regression and saved-failure UI replay:
`diagnostics/generated_controller_reuse_20261028/REPORT.md`.
No physical tracking or vibration-reduction result is claimed from these tests.

### CAN pressure during Hold / Return

Generated right J7, left J6 and replacement recovery owners now tolerate brief
`ENOBUFS` / `EAGAIN` send refusals. A refused command does not advance the Hold
controller's integrator: the next iteration receives fresh feedback and computes
a new command for the current checked goal. It does not sleep then replay an old
command. Continuous transmit blockage is limited to **20 ms**, using the existing
playback/MIT budget. Other send failures and genuine feedback/drive faults still
stop advancement. Normal striking/calibration branches and all gains are unchanged.

Generated encoder polling allows a **complete eight-drive sweep per GUI
callback**, or **two queries per 200 Hz worker tick**. The GUI's
`after(20)` waits 20 ms *after* callback work; it is not a fixed 50 Hz clock.
Its sweep must not be spread across several callbacks, including during the
confirmed startup sequence before a path worker exists. The busy 200 Hz writer
still staggers its queries, and bounded transmit-pressure handling remains.
Recent command/MIT
feedback suppresses redundant queries on each joint. The parent prioritizes the
oldest actual readings, rather than its query order before a trajectory; it does
not wait for a child to exit or for readings to become 100 ms old before resuming.
Startup mode queries are paced.
All existing receiver freshness, tracking, bounds and temperature limits remain.

A failed query **transmission** is reported as a CAN transmit failure; it no longer
latches otherwise fresh received encoders as missing. Receive failures still
latch and require fresh updates. This avoids the false `Fresh feedback missing`
follow-on fault without accepting stale feedback or bypassing return validation.
Software fault-injection and GUI verification:
`diagnostics/generated_return_can_pressure_20261028/REPORT.md`.

The original two-query GUI budget was insufficient at real callback intervals
and could fault immediately in ARMING. The correction retains the **120 ms**
feedback limit. Packet-level startup verification, including the real setup,
decoder, GUI and spawned trajectory workers with fake sockets only, is recorded
in `diagnostics/generated_startup_polling_20261028/REPORT.md`. Unlike the older
ideal-motion simulator, this test refreshes encoders only when replies arrive.

## Stop, relax, hold, and return

- **Emergency Relax / Escape:** immediately cancel arm writers and attempt
  torque-off for both arms/grippers plus ESP32 release, wherever the arms are.
  There is **no position, collision, limit, stationary-feedback or plan gate**.
  All drives are attempted even if another drive/peripheral reports an error.
  Torque-off can let an unsupported arm fall; use Hold instead when you want it
  to stay powered. Lost communications cannot guarantee physical disable; an
  unconfirmed write is reported, and the button can be retried.
- **Stop + Close, window close, Ctrl-C:** use the same unconditional explicit
  relax action and exit. No return-to-rest requirement, modal, or refusal to close.
- **Emergency Hold:** cancel pending calculations/starts and beat
  schedules, send ESP32 `S` to release all its outputs, and hold both arms powered.
  MIT workers cancel strikes and latch their *own fresh feedback* immediately;
  Hold does not wait for a full stroke to finish. No arm is automatically relaxed.
- **Return: right → left + relax:** one explicit click. During a normal beat,
  the existing strike workers first finish their current strokes at prime and
  cancel future hits. Normally play the **upfront-checked cached return**, right
  first, left second, without a new search. A partial-path stop uses the already-traveled prefix
  of the checked geometry, with the existing bounded recovery time law.
  The explicit request survives temporary encoder interruptions: the program
  keeps querying and waits for fresh stationary Hold, instead of requiring
  repeated button presses. **Hold cancels this pending return.**
  If the cache is missing or the measured pose is off the cached path, a bounded
  calculation worker checks a measured connector/reverse prefix. If that cannot
  be used, the existing workspace planner/timer calculates **new right-then-left
  routes from the measured Hold to the original captured rest**. The opposite
  arm's current/returned pose is included in each leg. Full tools, original zones,
  continuous joint limits and dense collision checks still apply; new recovery
  paths use conservative half-speed limits. No generic center or unchecked
  shortcut is ever substituted. Both arms remain powered while this runs; Hold
  cancels calculation and rejects any late result. A failed calculation leaves
  Hold and permits a new Return click in the same window.
  The camera mask remains at the original acquisition poses throughout.
  After both original resting poses are verified stationary, workers join and
  both arms/grippers are released; disabled feedback is checked.
- A stopped strike process is not a live encoder source. Once its samples age
  past 100 ms (or it exits), parent **read-only** queries resume, before the existing
  300 ms feedback watchdog. For an explicit Return, an exited MIT owner is joined
  and replaced with Hold/checked-return-only control, using the same powered Hold
  follower and gains. Fresh running feedback and actual MIT-mode readback precede
  any command; no enable, mode change, strike, or generic centering is performed.
  A healthy existing owner is retained. Recovery does not silently loop through
  failed motor-controller restarts.
- Return still needs usable live feedback, powered drives and a collision-free
  route. Genuine missing feedback, a drive fault or an impossible route cannot
  honestly be guaranteed away. These conditions **never block Emergency Relax
  or Stop/Close**. Peripheral cleanup errors also cannot veto arm Hold/Return.
  Repeating the same encoder fault during a retried motion stops its new writer,
  rather than merely updating the old fault label. `last_fault.json` records
  per-joint feedback ages/states in the current session for diagnosis.
- A failed startup can be retried with **Capture + calculate again** while arms
  are inactive, including after Emergency Relax. A moved arm invalidates Start;
  it is never driven to an old rest. A new capture replaces closed strike sessions.
  Obstacles must remain unchanged: this is not live collision avoidance.

## Scope and software verification

`--test`, `--hardwaretest`, and `--recording-only` keep their prior recording
workflows. The historical automatic normal-hardware recording/centering
`--verify-*` flags are rejected rather than silently authorizing the new workflow.
Normal hardware uses the calibrated RGBD serial, not the legacy RGB-only camera
selection option. Raw recordings, saved destinations and calibration data remain
unchanged. The old nominal-URDF RViz window is not launched in the generated mode;
its two new 3-D windows plus the control panel replace it.

Device-free replay of a **prepared generated mission**:

```bash
./start_beat.sh --generated-simulate diagnostics/generated_beat_controls_20261028/verified/mission.json
```

This mode never starts CAN, camera, microphone, ESP32 or real strike workers.
It exercises the paths/controls with ideal simulated motors and stops at the goals
rather than pretending that acoustic or physical calibration succeeded.

For an offline fresh numerical calculation on a saved paired capture:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv-planner-experiment/bin/python \
  -m camera_playback.generated_planning prepare --folder NEW_OUTPUT_DIRECTORY \
  --capture PATH_TO_SAVED_CAPTURE
```

Current stop/return verification: `diagnostics/generated_beat_controls_20261028/REPORT.md`.
Feedback recovery, missing-cache/fresh-return fallback and actual Tk simulation:
`diagnostics/generated_return_recovery_20261028/REPORT.md`.
Initial generation/beat integration: `diagnostics/generated_beat_20261028/REPORT.md`.
Software replay does **not** establish real tracking, physical support, clearance
accuracy, successful microphone calibration, or motor performance. Hardware
acceptance requires separate operator authorization; none was run for this update.
