# OpenArmX left/right safe-zone teaching (ROS 2 Jazzy + RViz)

## Start

Already built on this computer. Start the recorder for the arm you want to
teach:

```bash
cd /home/jason/X5
./start_left_zone.sh --hardware
# or, separately:
./start_right_zone.sh --hardware
```

`start_viewer.sh` remains as a backward-compatible alias for the left recorder.
Only run one zone recorder at a time because both verify both buses and use the
same cooperative CAN locks.

This is **query-only hardware observation**, not a powered robot controller.
Default wiring: **can1 = left**, **can0 = right**, classic CAN at 1 Mbit/s.
Optional overrides: `--left-can can1 --right-can can0`. Never put both on one bus.
The launcher does not configure CAN, enable motors, set zero positions, or launch
controller_manager, MoveIt, gravity compensation, or the vendor hardware driver.

1. Stop all other arm-control programs first. Both arms and grippers must already
   be disabled/relaxed. Support them appropriately: an unpowered arm can fall
   under gravity. These programs cannot keep either relaxed arm physically fixed.
2. Start the command above. Wait for **Paused** (fresh disabled-motor feedback).
   Both actual arm postures appear in RViz. Recording begins only when requested.
3. Before teaching, gently reposition only the arm named in the recorder window
   and verify RViz follows the correct arm and direction. Verify your physical model/mount and encoder
   calibration match the depiction. This setup check requires your observation.
4. Click **Start recording**. Guide the selected tool center around the region's outer
   limits at several heights, front/back, and side-to-side. Pause before moving
   through positions you do not want included. **Capture one point** is also available.
5. Orange dots are recorded TCP points. The translucent blue surface and edges are
   the convex envelope of the extremes. At least four non-coplanar points are needed;
   a flat trace will not be given invented thickness. A red/green live TCP dot is
   outside/inside the estimate, **not a collision-safety verdict**.
6. **Pause recording**, then **Save zone…** and choose a JSON filename (default folder
   `left_zones/` or `right_zones/`, depending on the recorder). The save path is
   forced into that arm's folder even if another directory is selected in the
   dialog. Save also pauses recording. **Import zone…** replaces the displayed
   zone and pauses recording; you may then extend it with more points. Clear asks
   for confirmation. Exiting does not automatically save.
7. Close either viewer window or press Ctrl+C in the launching terminal to stop.
   Shutdown never sends a motor-state-changing packet.

## Reopen without hardware

```bash
./start_left_zone.sh                      # offline left viewer, no CAN sockets
./start_left_zone.sh --load left_zones/my_zone.json
./start_right_zone.sh                     # offline right viewer, no CAN sockets
./start_right_zone.sh --load right_zones/my_zone.json
```

Or reopen with live encoder display:

```bash
./start_left_zone.sh --hardware --load left_zones/my_zone.json
./start_right_zone.sh --hardware --load right_zones/my_zone.json
```

Offline mode shows an explicitly labeled zero-posture robot illustration;
recording is blocked. Use the **Import zone…** button to browse files instead of
passing `--load`. The synthetic diagnostics file is for testing, NOT a taught zone.

## Live zone status and camera controls

After **saving or importing** a valid 3-D zone, when recording is paused, a large
indicator in the panel and matching RViz text show:

- **Green: INSIDE SAFE ZONE (selected arm TCP estimate)**.
- **Red: OUTSIDE SAFE ZONE (selected arm TCP estimate)**.
- **Amber: UNKNOWN** if encoders are stale/missing/faulted, the viewer is offline,
  or the zone has no 3-D volume. It never treats the offline pose as live feedback.

A fixed **5 mm outward tolerance** is applied in the background to each hull face
for this status check, including for existing imported files. The **displayed
surface, edges, recorded points, volume and saved hull remain exactly the original**;
no enlarged boundary is drawn. A TCP slightly outside the visible boundary can
therefore still show green. This is a per-face tolerance (at corners it is not
an exact rounded 5 mm distance shell), not additional physical collision clearance.

The live TCP dot uses the same colors. Recording suspends classification. Adding
new points changes the zone; save it again before checking against the new boundary.
An imported zone is ready immediately; you do not need to save it first.
This is still only a TCP-envelope test, **not whole-arm collision detection**.

In the RViz window, select **Move Camera** (enabled by default):

- **Hold the left mouse button and drag:** rotate/orbit the view.
- **Hold the middle mouse button and drag:** pan.
- **Mouse wheel:** zoom.
- The **Views** panel also exposes Orbit yaw, pitch and distance controls.

These controls change only your viewpoint; neither the robot's physical position
nor the saved zone coordinates change. Restart the viewer to load updated controls.

## What the zone means

The left recorder tracks `openarmx_left_hand_tcp`; the right recorder tracks
`openarmx_right_hand_tcp`. Each is derived from that arm's seven encoders using
the upstream URDF. Coordinates are meters in the robot's fixed
`world` frame (not relative to the starting tool position). The gripper opening is
an approximate visual mapping from upstream and does not change this fixed TCP.

This is a **convex TCP workspace estimate**, not a whole-arm movement safety
system. Convex hulls fill concavities, holes, obstacles and untraced gaps. Even a
TCP position inside the hull might be unreachable or put an elbow, gripper,
carried object or another link into a collision. Outside means **not labeled**,
not necessarily physically dangerous. No future motion restriction is enforced.
Do not feed this file into a controller as a certified collision-free region.
Re-teach if the robot mounting, tool geometry or encoder zero/calibration changes.

JSON contains the selected TCP name, an arm-specific left/right schema, input
points, hull extreme vertices, plane inequalities
`nx*x + ny*y + nz*z + d <= 0`, units, frame, TCP name, model SHA-256 and a
`certified_safe: false` flag. Import validates schema/model/finite bounded points
and rebuilds the hull instead of trusting stored planes. Files are written via
atomic replacement. Flat/incomplete recordings can be saved and resumed, but
have no valid 3-D interior. Minimum consecutive point spacing: 5 mm. Maximum:
50,000 points. Geometry cannot compensate for calibration or model errors.

## Read-only safety boundary

`safe_zone/encoder.py` is the only CAN transport. Its only outgoing packet is:

- extended arbitration ID `0x0200fd00 | motor_id` (IDs 1–8, each bus);
- data `01 00 00 00 00 00 00 00`, DLC 8;
- exact state query checked against the installed OpenArmX SDK.

Both buses are queried at up to 20 Hz for fresh states; the selected arm's TCP
points are processed/displayed at about 10 Hz. The other arm's data is **only**
for posture display and checking disabled/fault status. Mode comes from feedback ID bits 23:22,
not the SDK's hardcoded state-pattern field. Enabled status, fault, missing
feedback, bus error or transport failure latches an error and ends observation.
Recording stops, the live point disappears, and the existing zone remains
available to save. Correct the cause and restart; there is no automatic recovery.

The program deliberately does **not disable** a motor that is already enabled:
that could cause a supported load to drop. It reports the problem without
changing hardware state. Cooperative bus locks prevent another instance or the
X4 controller from sharing the buses, but unrelated programs may ignore these
locks. Separate ROS domains (85 for left, 86 for right) help isolate these displays from normal ROS
controllers; it is not a hardware interlock. The program cannot prevent another
process, external controller, gravity or a person from moving the arms.

## Files, dependencies and rebuild

- `safe_zone/app.py`: shared arm-selectable recording panel, ROS joint states and RViz markers.
- `safe_zone/encoder.py`: audited query-only transport and disabled-state checks.
- `safe_zone/geometry.py`: URDF FK, convex hull and validated JSON persistence.
- `launch_viewer.py`: only robot_state_publisher, RViz and the panel.
- `start_left_zone.sh`, `start_right_zone.sh`: separate left/right launchers.
- `left_zones/`, `right_zones/`: enforced arm-specific save folders.
- `config/safe_zone.rviz`: visualization configuration.
- `model/openarmx.urdf`: generated bimanual description without ros2_control.
- `VERIFICATION.md`, `tests/`, `diagnostics/`: checks and evidence.

Uses system Python, NumPy, SciPy, Tk, ROS Jazzy rclpy, sensor_msgs,
visualization_msgs, geometry_msgs, launch_ros, robot_state_publisher, RViz and
xacro. These dependencies are installed on this machine. No pip environment is
needed. `./build.sh` rebuilds **only the description package**, regenerates the
control-free model and runs offline unit tests; it never accesses CAN.

Upstream references (checked out locally, original licenses preserved):
- https://github.com/openarmx/openarmx_ros2/tree/6.0_basic
  at `bc45d1a6c8e0474e0530a7ba924ae14c2b43cc5a`.
- https://github.com/openarmx/openarmx_description/tree/6.0_basic
  at `425bf9f708e5c49092a5920a3f38a8719a6f2b07`.

The upstream hardware activation explicitly enables motors, so it is **not used**.
The vendor description/meshes retain their upstream non-commercial/share-alike
license; see `vendor/openarmx_description/LICENSE` before redistributing.

## Separate left-arm centering test

The button-triggered motor-zero test is documented in [CENTERING.md](CENTERING.md).
Run `./start_centering.sh` for a **no-CAN preview**, or `./start_centering.sh --hardware`
for encoder display with explicitly button-triggered physical centering.
It controls only left IDs 1–7, keeps the left gripper disabled, and provides
Emergency Relax. Close the recorder first. Unlike the recorder it can move the
arm after Center is pressed; **this test does not enforce the taught safe zone**.

## Center then move to the tested goal

See [GOAL_MOTION.md](GOAL_MOTION.md). Run `./start_goal.sh --hardware` and click
**Run: Center → Goal → Hold 2 s → Center → Relax**. This sequence puts
left joint 1 at +45°, joint 4 at +80°, joint 7 at −80°, and the others at
zero, holds the measured goal for 2 continuous seconds, then automatically
returns all seven left joints to zero before disabling torque. It uses a
physically tested 0.4 rad/s firmware speed setting and retains
encoder correction, zone1 monitoring, lenient stall detection, live RViz/angles,
and immediate Emergency Relax. A zone breach attempts to center before relaxing;
a stall or other fault relaxes immediately without further commanded movement.

## Left and right Cartesian gripper goals

See [CARTESIAN_GOAL.md](CARTESIAN_GOAL.md). Run either
`./start_left_cartesian.sh --hardware` or
`./start_right_cartesian.sh --hardware`, enter X/Y/Z offsets in meters relative
to that arm's all-zero gripper TCP, and click **Run Cartesian Goal**. Each program
loads its matching `left_zones/zone1.json` or `right_zones/zone1.json`, rejects
an unsafe target/path, converts an accepted point to seven joint angles, then
uses the same measured-goal controller. The left program holds for two seconds
and then automatically recenters. The right program holds indefinitely until its
right-only **End** button is pressed, then recenters and relaxes. After the first
right goal settles, changing the coordinates to another inside-zone point enables
a right-only **Update** button. It validates the new IK, live transition, and
center returns before moving and holding at the updated goal. The Cartesian layer
controls endpoint position, not orientation or a straight-line trajectory.
Before Run is clicked, its RViz target follows valid field edits live: magenta
inside zone1, red outside, and hidden for invalid/incomplete input. This preview
does not enable motors or run IK. The bottom encoder panel includes joints 1–7
and the selected arm's gripper encoder (motor 8). The left gripper remains
display-only and relaxed.

The right center is J1–J6 at zero and J7 at the model's `+1.4 rad`
(`+80.214°`) limit, corresponding to the right motor's reversed negative
direction. Cartesian `(0,0,0)` nevertheless remains the all-zero-joint TCP.
The configured right center and the checked 5 cm upward example path are inside
`right_zones/zone1.json`; the all-zero Cartesian origin itself is outside and
cannot be selected as a safe goal. The custom gripper's extra physical tool
length is not present in the current URDF TCP.

The right Cartesian program additionally opens its custom gripper to −3° while
centering, pauses at center for the user to load an object, and enables a
right-only **Continue** button. Continue closes the gripper to +7°, then performs
the goal motion and holds that goal indefinitely while continuously holding +7°.
A standalone-right IK constraint keeps J2 at its centered 0° value and uses the
other six joints for the same requested TCP point; this avoids the measured
small-negative-J2 tracking failure without changing the left or camera program.
A right-only **Update: Move to Changed Goal** button becomes available only after
the current goal has settled and the coordinate fields contain a different
inside-zone point. It may be used repeatedly after each updated goal settles.
Update planning uses the current posture first and a bounded path preflight, so
ordinary updates validate promptly instead of repeating a full return-path test
at every 20 ms motion sample. Initial and updated goals use the same 3-degree,
0.6-second joint-settling rule as the standalone left Cartesian program.
A right-only **End: Return to Center → Relax** button then returns the arm to its
special center with the gripper still closed and disables all eight right motors.
It never sends another open command before that relaxation. The left Cartesian
workflow remains the automatic two-second hold sequence.

## Right-arm camera Cartesian hill climber

See [CAMERA_SEARCH.md](CAMERA_SEARCH.md). The separate
`start_right_camera_cartesian.sh` program combines the original right-arm
center/open/load/close workflow with Y2's pose detector. It uses the fixed
Cartesian goal `(0.250, 0.000, 0.350)`, then increases only Y by 0.010 m through
the preflighted right-zone coordinates until a fresh drumstick plus YOLO-tip
detection occurs. If no detection occurs, it returns to the customized right
center and relaxes. Its redundant IK keeps right joint 2 centered while
preserving each requested TCP coordinate.

Start and Continue require a fresh cymbal detection. After a drumstick plus
directly observed YOLO tip is detected, the arm runs a deterministic coordinate
hill climber in `+X, -X, +Y, -Y, +Z, -Z` order. It accepts only moves that reduce
the normalized image distance from the tip to the cymbal's center ninth, using
0.010, 0.005, and 0.0025 m step levels. Each candidate, anchor return, and center
return is zone/IK checked before movement. Intermittent missing-tip frames make
the arm hold rather than fabricate a tip or abort immediately. Success requires
the tip to remain in the target for two seconds and a fresh inside confirmation;
the arm then centers and relaxes.

Because the customized center puts J7 exactly at its URDF maximum, the candidate
planner tolerates up to three encoder counts of measured limit overshoot and
clips only its SciPy initial seed inside the bounds. It continues to validate
paths from the unmodified measured anchor and rejects larger limit violations.

Program failures and every non-stall fault center before relaxing; a detected
stall remains the sole automatic immediate-relax exception. Support-process
exits are reported to the controller so they use the same recovery instead of
forcing an arbitrary-pose shutdown.

The program launches RViz, one control panel, and only Y2's processed camera
window. The processed preview draws a 3-by-3 grid over the strongest cymbal box
and highlights its center rectangle as `CYMBAL TARGET`; that center cell is now
the visual alignment goal. The original `start_right_cartesian.sh` is unchanged.

## Query-only left/right motion recorder

See [RECORDING.md](RECORDING.md). Select exactly one arm and opt into hardware:

```bash
./start_recording.sh --right --hardware
./start_recording.sh --left --hardware
```

The selected arm must already be disabled and physically supported. Right mode
queries only `can0` and saves under `recordings/`; left mode queries only `can1`
and saves under `left_recordings/`. The recorder never enables, disables, holds,
centers, or commands a motor. Launching without `--hardware` is an offline
preview. Joint-limit violations abort and discard the complete active take.

## Left/right recording video editor

See [RECORDING_EDITOR.md](RECORDING_EDITOR.md). Run:

```bash
./start_recording_editor.sh
```

The editor accepts compatible recordings from both `recordings/` and
`left_recordings/`, and previews the arm identified by each JSON file. It can
play, scrub, and crop the timeline, then atomically overwrite that exact selected
file after confirmation. It never opens CAN or commands physical hardware.

## Recorded-path right camera alignment

See [RIGHT_CAMERA_PLAYBACK.md](RIGHT_CAMERA_PLAYBACK.md). This is a separate
program; the original `right_camera_cartesian` program remains unchanged.
Defaults: right `recordings/record3.json`, left `left_recordings/record1.json`.
Normal beat playback centers both arms, plays the left recording first, holds its
endpoint, then plays the right recording and continues the right-only beat
workflow. Each selector has a persistent preflight cache. Center + Relax returns
both arms together and disables each after its own verified center arrival.
The left gripper retains its centered closed position; recorded gripper values
are not replayed. This two-recording addition is verified offline only.

```bash
./start_beat.sh --hardware \
  --recording recordings/right_motion_YYYYMMDD_HHMMSS.json
```

### Editable beat tempo

Set **Beat BPM** in the control panel before pressing **RUN** (or **Start** in
`--test`). The default is **100 BPM**; decimal values from **20 to 180** are
accepted. Blank, nonnumeric, nonfinite, or out-of-range entries disable starting.
The box is locked from Start/RUN until the arm has completed **Center + Relax**;
change the tempo before the next run, not in the middle of a stroke.

Both the ride swing grid and the hi-hat use `quarter_note_seconds = 60 / BPM`.
The ride retains its pickup and 2:1 triplet swing. The hi-hat still **opens on
beats 1 and 3** and **closes on beats 2 and 4**, at its calibrated angle. Its
continuous acoustic timing adjustment still shifts both CLOSE and following
OPEN together; the ride has no offset. At faster tempos the signed advance is
capped at the smaller of 250 ms and half a beat, with a proportionately bounded
ride-onset matching window. At 100 BPM all timing defaults are unchanged.

**Estimated practical upper tempo: roughly 120 BPM**, not a hardware-verified
limit. The shortest ride spacing is `20 / BPM` seconds (167 ms at 120 BPM), and
the existing short rebound alone takes 110 ms before allowing for the next
stroke. The 180 BPM input ceiling is not a promise that the arm/hi-hat can keep
up. Existing rebound, motion, encoder, and missed-deadline safeguards remain;
an excessive tempo can still produce a controlled stop. Calibration holds,
recording playback speed, strike depth/boost, model settings, and motor tuning
are not scaled by BPM. Manual `--hardwaretest` and recording-only modes have no
BPM box because they do not play a continuous beat.

A pure simulation test is also available:

```bash
./start_beat.sh --test
```

This test mode never constructs the real motor controller, never opens CAN, and
does not start or require the camera, ST7/TONOR microphone, or ESP32 hi-hat. An
in-memory arm is animated in RViz through centering, the selected recording,
and the safe 10-degree J7 target in the same triplet-based, user-tempo swing ride (100 BPM by default)
used by hardware mode. It performs no strike-depth search and sends no hi-hat
commands. The
same **Stop [BPM] striking: Center + Relax** sequence centers and relaxes only
the simulated arm. The three modes are mutually exclusive.

A manual powered test mode is also available:

```bash
./start_beat.sh --hardwaretest
```

`--hardwaretest` starts the physical right-arm controller, RViz, processed Y2
camera, and TONOR/ST7 detector. The ESP32 hi-hat remains disabled. It restores
the original hardware-test preparation: select and preflight a recording, press
Start to center with the gripper open at −3°, load the stick, press Continue to
close the gripper to +7°, play the recording, and use one fresh stick-tip
detection inside the visible pink rectangle to accept the cymbal pose. ST7
events are logged for information only and never trigger or retry a strike.

At the accepted pink-zone pose, J7 switches once to MIT hold. The text box
accepts a **maximum drop goal** in degrees (default 5°), relative to a **fixed
reference anchor**, not a new cached position on every press. A dedicated spawned
process handles timestamped J7 position/velocity feedback
independently of the Tk/ROS interpreter: 500 Hz during a strike and 100 Hz during
stationary hold. Active motion retains a 20 ms deadline. A late idle send refreshes
the same hold without discarding valid encoder stability; old feedback or a gap
in the position history resets readiness. Idle gaps over 250 ms still fault.
The descent is gravity-driven with lighter damping (default KD=0.04); a
predictive catch accounts for velocity, latency and the blended trajectory's
braking distance. Upward acceleration continues through the lowest point into
withdrawal, rather than gently stopping and restarting at the bottom. The return
still decelerates to zero desired velocity and acceleration at the upper anchor.
No contact detection is used. J7 remains in MIT between strikes: no
per-strike disable/mode-switch or slow CSP correction. Another press is enabled
only after a fresh 40 ms encoder-position window is stable near the anchor.
Readiness uses position variation and movement trend with hysteresis, not noisy
instantaneous drive velocity. Raw velocity still drives predictive catching.
Results show measured peak drop and return overshoot. Center + Relax restores CSP only when leaving the session.

The dynamics defaults are initial tuning, not a claim of physical calibration.
`--mit-fall-kd`, `--mit-inertia`, `--mit-brake-accel`, and `--mit-latency-ms` expose
the main hardware-test calibration values. See [MIT hardware-test details and
calibration](RIGHT_CAMERA_PLAYBACK.md#hardware-test-calibration). No physical
accuracy or sound-consistency claim follows from the offline tests alone.

Before sending any center target, `--hardwaretest` performs a powered-state
preflight at the live encoder pose. Arm drives J1–J7 are explicitly disabled,
configured for CSP mode, preloaded with their current positions, enabled, and
required to report powered operation. After those arm drives confirm, the
gripper is configured with the requested safe −3° open target rather than
replaying a manually positioned encoder angle that may be outside its narrow
command envelope. The mode/enable sequence is retried up to three times. Only
after **every** drive, including J7, confirms operation can the arm begin moving
toward center. If any confirmation fails, the program prints that motor's
run-mode readback, feedback operating state, and encoder position, disables only
that named drive, leaves already powered drives holding their starting poses,
and never starts a partial-arm center move.

On leaving the MIT session, CSP mode and powered-state confirmation may be
retried for two seconds. If J7 is known to be in CSP mode but still reports relaxed, the program repeatedly sends
both enable and anchor-hold commands rather than declaring success. If
confirmation or another MIT step fails, the program locks out further strikes
and keeps requesting the safest hold supported by the confirmed mode. Other
hardware-test faults also lock the workflow instead of automatically centering
and relaxing: a fault naming one right drive disables only that drive and holds
the others, while a fault without a reliable drive number preserves the last
commands and keeps the healthy drives powered. A detected motor stall is the
intentional exception and immediately disables the entire right arm because the
motor may be pushing into an obstruction and overheating. The MIT rebound has a
0.5-second J7 stall monitor. **Emergency Relax** remains an explicit user action.

Normal `--hardware` has one **RUN: CENTER + CLOSE → RECORDING → HYBRID SWING**
button instead of separate Start and Continue steps. With the stick already
secured, press RUN once: the arm centers while the gripper closes to +7°, then
automatically moves to the recording's first right-arm pose after center and
gripper closure are confirmed. There is no gripper-opening or loading pause,
and simply opening the application does not start motion. Recording, camera,
ST7, hi-hat, and encoder readiness gates remain in place. `--test`,
`--hardwaretest`, and `--recording-only` retain their existing startup workflows.
Normal hardware temporarily raises only recorded-path J1–J7
motion to a 0.8 rad/s firmware limit, follows its time-scaled path at up to 90%
of that limit, and settles at the recorded endpoint. It restores the ordinary
0.4 rad/s setting after the endpoint is physically reached and before any
alignment, recovery, or centering. Camera observations collected during
playback reorder but never eliminate the six Cartesian search directions.
Post-playback alignment
checks one fresh tip and accepts the recording endpoint immediately when it is
anywhere inside the larger visible pink rectangle. If outside, Cartesian
correction keeps using robust outside-zone measurements to choose its next
move, but the first fresh tip detection anywhere inside the visible pink
rectangle immediately completes alignment. There is no smaller invisible goal,
second confirmation, or timed hold. The launcher starts the scoring-enabled ST7 cymbal-sound
detector from the sibling `/home/jason/Proyectos3/st7`, explicitly selecting
`models/v2/config.yaml`, `models/v2/best.pt`, and the matching
`models/v2/normality/reference.npz`. Missing V2 assets stop startup; there is no
fallback to V1 or `/home/jason/st7`. This applies to every audio-enabled mode
(including `--hardware` and `--hardwaretest`); pure `--test` still runs no audio
model. ST7's own default launcher and the Y2 camera model are unchanged.
The launcher also starts **`models/hihat_v1`** (its own config, checkpoint and
normality reference) on the same TONOR source. A separate **Hi-hat sound v1**
panel flashes green for each detected closure and shows its count, last score
and normality. The panel/layout is unchanged. In normal **--hardware**, hi-hat
audio now selects the startup closure angle and then adjusts **only hi-hat
timing** during swing. It never selects a J7 depth or retimes the ride.
During swing, missing/stale/ambiguous hi-hat detections freeze the last valid
advance. During startup calibration, detector failure or exhaustion of the
90°–115° search is a fault and prevents ride search/swing. Status wording in the
existing panels reflects calibration rather than claiming “visual only”. ESP32
motor faults still use the existing controlled-stop safety path.

Ride and hi-hat use independent listener processes and Unix sockets, with
instrument-tagged messages checked at both the bridge and receiver. Each
listener is explicitly pinned to TONOR through PipeWire and uses bounded
single-thread CPU pools to avoid oversubscription. Independent listeners keep
optional hi-hat failures out of ride control; the models, decoder thresholds,
lookahead and normality references are not changed or retrained. Both audio
models run in audio-enabled modes; **`--test` still opens no microphone and
starts neither model**. No new launch flags or workflow steps are needed.
See [dual-detector verification](diagnostics/dual_sound_20261005/VERIFICATION.md).

X5 explicitly routes audio input through PipeWire
to the connected TONOR TD510 microphone, and will not enable the sequence until
the **ride** detector reports that normality scoring is ready. After the
single accepted camera frame, normal `--hardware` starts the **same hybrid
method and hardware tuning as `experiment.sh`**: a smooth 0.3 Nm / 40 ms
downward torque pulse, low-impedance coast, predictive catch, and smooth powered
return. Search attempts remain complete round trips:
anchor→−5°→anchor, anchor→−6°→anchor, anchor→−7°→anchor, in 1-degree
increments. Goals are bounded by the safe-zone path, joint limits, and the
beat's 13° hard corridor (integer search goals through 12°). The experiment
itself retains its 12° default; copying that default into the beat had wrongly
prevented the next 12° attempt, which ST7 detected in physical testing.
After each settled return, the controller waits until ST7 has finalized the
entire strike's audio interval. Hits use PortAudio ADC capture timestamps, not
the time the listener printed its startup message. A three-second audio backlog
causes a controlled stop rather than falsely declaring a miss. In `--hardware`,
the swing depth is now **the first detected depth + 0.5°** of J7 displacement:
for example, detection at 10° produces 10.5° swing hits. Search attempts remain
at their original integer depths and complete their original return before the
boost is applied. Fractional depths are retained through the worker and logs.
The existing validated corridor and joint/experiment limits still apply; an
unsafe boosted target causes a controlled Center + Relax, not widened limits.
No normality-based selection, detector changes or additional search are used.
`--test` and `--hardwaretest` are unchanged, as is hi-hat synchronization.

That boosted depth then plays hybrid strokes in the existing triplet
swing at the selected BPM (default 100): opening pickup, beat 1, beat 2 + extra, beat 3, beat 4 + extra. Target
spacing at the default 100 BPM from the pickup is 0.200, 0.600, 0.400, 0.200, 0.600, 0.400 seconds.
A separate 500 Hz J7 worker reuses `strike_lab.methods.hybrid` and
`config/experiment_hardware_tuned.json`; Tk never clocks the catch. It predicts
release timing from measured position/velocity and the hybrid impulse/load
model, with coast inertia identified from encoder velocities for scheduling
only, not a constant position-mode speed. For 200 ms pairs, a 110 ms smooth
partial rebound targets 4.4° above the selected low target, braking upward
momentum before the next impulse. Another stroke requires encoder clearance
of at least 4° above that fixed target. The original camera anchor and
absolute strike depth remain fixed. Complete search hits match the experiment
method; short swing returns intentionally use the same quintic/tracking law
with a shorter rebound endpoint. Longer returns retain the full experiment
profile. Late/unachievable deadlines fault instead of firing catch-up bursts.
Physical acceptance now requires an actual **ST7 HIT accepted by the search**,
followed by at least five seconds of fault-free swing and verified center/relax.
No independent waveform analysis or repeated-HIT quota substitutes for ST7.
See [ST7 physical verification](diagnostics/st7_handoff_20261004/VERIFICATION.md).
The normal return keeps J7 powered in MIT while centering (0.35 rad/s bounded
reference); it does not disable J7 to change modes at the cymbal. An off-center
CSP handoff was found to drop J7 and produce an unintended, correctly rejected hit.
Normal hardware now guards every whole-arm relax, including window close and
Ctrl-C: all seven joints must be within 0.20° of center and settled for 0.6 s
before disabling. A detected stall/overheating remains a major-fault exception.
`--test` retains its existing powered simulation; `--hardwaretest` retains its
separate manual gravity/catch controller.

The integrated
115200-baud ESP32 hi-hat remains on the main
quarter-note beats only; it receives no command on a ride extra. Its main-beat
commands in normal hardware mode are now `O`, `B`, `O`, `B`…: **OPEN on
1/3, CLOSE on 2/4**. `B` uses the startup-calibrated **90°–115° close/hold** target;
`O` retains the **0° return/release** target. Legacy `C`/`J` commands remain
fixed at 100° for other programs and existing collection procedures. Commands use the unchanged ride grid minus one signed hi-hat
advance (positive = earlier, negative = later), initially zero each run. The host never sends motor 1's `K` command because right J7 replaces
motor 1. The ESP32 code is copied under
`esp32_hihat/`, and the launcher defaults to the connected CP2102's stable
`/dev/serial/by-id/` path.

### Automatic hi-hat angle calibration (normal hardware only)

Immediately after launch, without pressing RUN, the program starts calibration
as soon as hi-hat ST7/TONOR and the ESP32 calibration-v2 protocol are ready.
There is **no ntfy alert or countdown**. The arm remains under its existing
RUN control. No startup calibration runs in `--test`, `--hardwaretest`, offline
preview, or recording-only mode.

The hi-hat first returns to encoder zero. It then tries **90, 95, 100, 105, 110,
115 degrees**, each time verifying encoder arrival, holding closed for **2 s**,
returning to zero and remaining open for **2 s**. A valid hi-hat-model HIT whose
audio onset belongs to that closure selects the first successful angle, but
does not shorten the hold/open cycle. No angle boost is applied. Late detection
notifications can still select their original trial; opening sounds and old
hits cannot select the next trial. Before a miss is declared, ST7 must finalize
the closure's audio interval. Stale feedback, missing acknowledgments, audio
processing stalls, detector failure and a completed **115° attempt with no HIT**
are faults, never permission to move farther. There is no 120° attempt or
fallback to a fixed angle. Calibration restarts at 90° on the next launch.
The return is verified by the firmware's encoder-zero arrival latch followed
by a settled released encoder. A passive motor may coast after release; it is
not incorrectly required to keep holding exactly zero while unpowered.

RUN may center/close/play the arm recording while the hi-hat is calibrating.
At the recording endpoint the arm waits in its feedback-supervised powered
hold if needed. Once the hi-hat is ready, a fresh camera alignment check precedes
the unchanged ride search (+0.5° swing boost retained). Calibration failure
uses the existing **center-before-relax** arm recovery. Cancelling or closing
the application also cancels calibration and supervises the hi-hat's return;
a stalled/faulted hi-hat output is released rather than held indefinitely.
This dedicated stationary wait retains feedback/stall checks without timing out
solely because a successfully held pose lasts longer than a normal 30 s move.

This requires the updated `esp32_hihat/motor_beat` firmware; normal launch never
flashes it automatically. Unsupported firmware fails closed before calibration
motion. `playback_results/hihat_calibration/<session>/events.jsonl` records
trials, encoder arrival, selected HIT, angle and faults.
The recording picker is temporarily disabled during calibration motion. In normal
hardware mode, recording validation runs in a calculation-only background worker;
ESP32 heartbeats/status queries and detector/encoder polling continue on the GUI
loop. RUN stays unavailable until validation finishes, then requires a fresh ESP32
reply before enabling the arm. A failed validation cannot reuse an older recording.
Real missing-feedback/controller faults remain latched; their timeouts are not
weakened. The UI layout and RUN workflow are otherwise unchanged.

### Visual-only TONOR sound timeline

`start_beat.sh` also opens a separate sound-debug window whenever the live
detectors run (not in microphone-free `--test`). It shows a **fixed 10-second
rolling waveform**, not an ever-growing recording. **Red** vertical lines label
ride cymbal detections; **blue** dashed lines label hi-hat detections. Both use
the ST7 event's original ADC-monotonic onset time. A delayed detection therefore
appears over the earlier sound, then scrolls left with that waveform; it is not
placed at notification time. Overlapping ride/hi-hat labels occupy separate rows.

The viewer has its own read-only TONOR capture stream and uses the same
ADC-to-monotonic mapping as ST7. It consumes nonblocking, one-way copies of
existing detector events, without changing model output, calibration, offsets,
ride timing, or motor control. Visual gain is display-only. Audio gaps are not
stitched together; only a bounded in-memory history is retained, and **no audio
files are saved**. Closing or losing this window leaves the beat program and
both detectors running normally. Exiting the beat program closes it too.

### Background hi-hat synchronization (no ride offset)

Only `--hardware` has the new servo. `--test`, `--hardwaretest`, data collectors,
model assets/thresholds, the ride search and hybrid motion controller are unchanged.
Both detectors' ADC-derived monotonic acoustic onsets share one internal timeline;
notification delays never become timing offsets. The read-only ride pickup/grid
starts a 5 ms host callback scheduler; no command is sent on the pickup or extras.

A closure on beat 2/4 is matched against one unambiguous ride onset within
a window around that scheduled beat (the smaller of 140 ms and one third of
a beat), plus a neighboring ride-grid detection. One hi-hat onset must occur within 450 ms of its actual close command. Both detector
watermarks must finalize the entire matching window. Duplicates, missing or
ambiguous events, stale sources and corrections outside the signed safety
bound (the smaller of 250 ms and half a beat) do not change the advance.
This cannot eliminate classifier cross-talk:
a missing real ride plus a false ride response to a hi-hat can still be ambiguous
in reality even when the detector output looks unique. Better models may be needed.

The estimate uses the **actual advance used for that historical command** plus
`hihat_onset - ride_onset`, avoiding repeated integration of delayed errors.
After three valid pairs, a rolling five-estimate median drives smoothed changes
(maximum 10 ms per update, 15 ms deadband). Every close and following open share
a latched advance, preserving their one-quarter-note separation (600 ms at
100 BPM). New pairs change by at most 10 ms; already scheduled edges never jump. Missed deadlines cause the
existing controlled stop, never a burst or an airborne arm release. No guarantee
of perfect acoustic synchronization is made at the models' finite time resolution.

Each swing writes `playback_results/hihat_sync/<session>/timeline.jsonl` with
common-clock detector onsets, beat IDs, actual commands, applied advances,
accepted timing errors and rejected-pair reasons. No new UI or launch action is
required. These logs are diagnostics, not training labels.

This automatic arm-and-hi-hat loop belongs to `--hardware`; its arm and hi-hat
continue until the user presses **Stop [BPM] striking:
Center + Relax**. Stop finishes any
active hybrid return and settled anchor, tells the hi-hat to return to 0°,
joins the worker, restores and confirms position mode at normal speed, then
centers and relaxes. Hybrid controller/transport faults block automatic movement
and retain a fixed-pose hold where feedback/transport permits; use Emergency
Relax or the physical stop and restart. Emergency Relax and window close join
the worker before disabling motors. The camera box-motion
overlay is retained only as a visual diagnostic and never decides a hit. This
behavior is confined to the recorded-path playback program.

## Separate hi-hat sound data collection

See [HIHAT_DATA_COLLECTION.md](HIHAT_DATA_COLLECTION.md) for the finite collector
`collect_hihat_data.sh`, its offline-only `--check`, and separate offline labeling
program `label_hihat_data.py`. The collected hi-hat WAV/CSV pairs live under
`X5data/hihat/recording` and `X5data/hihat/timestamp`; existing ride data has moved
unchanged to `X5data/ride/recordings` and `X5data/ride/timestamps`. Mixed ride/hi-hat
closures remain positive for the new hi-hat target. No model training or merging
into ST7 is performed. The first batch needed verified post-capture center
recovery; see the linked verification report before rerunning physical collection.

## Isolated right-J7 ten-degree test

See [RIGHT_JOINT7_TEST.md](RIGHT_JOINT7_TEST.md). The separate test panel has no
camera or recording dependency. **Start** centers only the right arm at the
custom J7-at-maximum center and opens the right gripper to -3 degrees.
**Continue** closes the gripper to +7 degrees, waits for encoder-confirmed full
closure, moves only J7 down by 10 degrees at its separate 2.0 rad/s test speed,
and immediately returns J7 to the custom center. It then
restores the normal 0.4 rad/s J7 setting and disables all right motors.

```bash
./start_right_joint7_test.sh --hardware
```

The complete outbound/return TCP path is preflighted against buffered right
zone1 before any motor is enabled. The left arm and left gripper remain relaxed
and query-only. Running without `--hardware` is an offline UI/safety preview and
cannot open CAN or move the arm.

## Standalone strike experiment

`./experiment.sh` opens a separate J7 strike laboratory with control buttons and
RViz. Simulation is the default; physical transport requires `--hardware`.
Preparation closes the gripper, centers the arm, plays `record3`, and holds its
endpoint. Nine selectable modes share an editable J7 strike depth below that
endpoint (default 10°; UI entry or `--degrees 3`). The gripper stays closed while
enabled. No camera, sound detection, stick loading, or ESP32 is used. Every attempt preserves its source,
parameters, full-rate trace, and result. Existing `start_beat.sh` modes are unchanged.
See [EXPERIMENT.md](EXPERIMENT.md) for scoring, saved methods, configuration,
finite tuning campaigns, reanalysis, and the explicit physical-testing boundary.
The unloaded real-arm comparison and calibrated profiles are documented in
[EXPERIMENT_RESULTS.md](EXPERIMENT_RESULTS.md); those centered/no-stick results
are not a calibration of the new record3 posture or attached stick. The graph
measures encoder-zone timing, not actual cymbal contact or sound quality.
