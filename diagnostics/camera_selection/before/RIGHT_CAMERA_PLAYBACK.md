# Right-arm recorded camera Cartesian alignment

This is a new program. It does not replace or modify
`start_right_camera_cartesian.sh` or the `camera_search/` implementation.

The program combines a saved right-arm motion recording with the existing Y2
camera detector and a separate post-playback Cartesian hill climber.

Both hardware modes default to the configured **Orbbec Gemini 2 RGB**, selected
by USB identity. `--camera N` remains an explicit override. Close OrbbecViewer
before launch; see [shared camera defaults](CAMERA_DEFAULT.md).

## Start

First create a recording with [RIGHT_RECORDING.md](RIGHT_RECORDING.md), then
close the recorder and every other arm-control program. Start playback with a
specific file:

```bash
cd /home/jason/Proyectos3/X5
./start_beat.sh --hardware \
  --recording recordings/right_motion_YYYYMMDD_HHMMSS.json
```

The default recording is `recordings/record1.json`, so the normal command is:

```bash
./start_beat.sh --hardware
```

### Pure simulation test mode

To simulate playback and striking without any physical hardware, replace
`--hardware` with `--test`:

```bash
./start_beat.sh --test
```

`--test` is mutually exclusive with `--hardware` and `--hardwaretest`. It never constructs the real
motor controller, never opens SocketCAN, and cannot send commands to the arm or
gripper. The launcher also does not start Y2, ST7/TONOR, or the ESP32 serial
controller. Arm and gripper movement exists only as in-memory joint state shown
in RViz.

Test mode uses an in-memory motor substitute behind the same recording loader,
path preflight, centering, gripper load/Continue sequence, recorded playback,
strike controller, 100 BPM loop, safe-zone checks, and Stop/Center/Relax path as
normal hardware mode. Once the simulated recorded endpoint settles, it assumes
that pose is the correct cymbal anchor, safely derives the 10-degree J7 target,
skips camera alignment and every hit-search attempt, and immediately repeats
exactly 10 degrees in the same triplet-based 100 BPM swing ride as hardware
mode. It starts with the extra pickup before beat 1. It sends no hi-hat command.

### Playback-aligned MIT hardware test mode

To run the loading/playback/camera-alignment workflow and then request one J7
MIT strike at a time, use:

```bash
./start_beat.sh --hardwaretest
```

The launcher starts the arm controller, RViz, the Y2 processed camera, and the
TONOR/ST7 detector. It intentionally does **not** open the ESP32 hi-hat
controller. Select and preflight a recording before Start. The mode then follows
the original hardware-test preparation sequence: center with the gripper open at
−3°, wait for the user to load the drumstick and press Continue, close the
gripper to +7°, play the recording, and use the camera-guided Cartesian
alignment. One fresh directly observed stick-tip detection anywhere in the
visible pink rectangle accepts the cymbal pose immediately.

Start does not immediately send the arm-center target. It first explicitly
disables J1–J7, selects and confirms CSP mode, preloads each arm drive's live
encoder position, enables it, and requires a fresh powered-state feedback frame.
After all seven arm drives confirm, the gripper is configured with the requested
safe −3° open target. This avoids replaying a manually positioned gripper angle
that may lie outside the program's deliberately narrow command envelope. The
mode/enable operation is retried up to three times. The arm begins its center
movement only after every drive has confirmed powered operation. If any drive
fails confirmation, the terminal reports its run-mode
readback, feedback operating state, and encoder position; that named drive is
disabled in isolation while the already enabled drives retain their preloaded
starting-pose holds. No partial-arm recenter is attempted.

After the single pink-zone observation, J7 switches once to MIT hold. The accepted
pose stays the **fixed reference anchor** throughout the session. The button is
available only after fresh feedback shows position within 0.15° and speed below
0.03 rad/s for 30 ms. Repeated/cached samples cannot satisfy that confirmation.

The degree entry is a **maximum drop goal**, not a late reversal trigger; its
default is 5°. For an anchor of 55°, entering `2` requests a lowest point near
53°. Decimal goals and their paths are checked against the existing J7 limits
and right safe zone. One press requests one attempt; there are no automatic
retries, depth search, continuous swing, or audio-triggered moves.

The hardware-test-only controller in `camera_playback/mit_strike.py` follows:

1. **Gravity-driven fall:** zero position gain/feed-forward torque, with lighter
   damping (`KD = 0.04` by default, previously 0.08), rather than a powered downstroke.
2. **Predictive catch:** measured position, velocity, feedback age, command delay,
   and braking distance initiate the catch before the requested lowest point.
   The predicted stopping distance includes the blended catch/return curve.
   Velocity passes through zero with positive upward acceleration, so withdrawal
   starts without a zero-acceleration restart or a bottom settling wait.
   Loaded-inertia feed-forward supports the requested acceleration.
3. **Smooth return:** a quintic trajectory inherits the catch's upward acceleration
   and ends at the fixed anchor with zero desired velocity and acceleration.
   Its peak speed is at most 2 rad/s and
   acceleration at most 20 rad/s²; small strokes do not reach that peak speed.
   MIT tracking gains default to KP=40, KD=1.8.
4. **MIT hold:** a small, bounded load-compensation adjustment operates only near
   a stationary anchor and is frozen during motion. Readiness requires
   a full **40 ms encoder-position window** near the anchor: every position must
   be within 0.15°, travel range within 0.05°, and fitted position trend below
   0.03 rad/s. Once ready, range/trend limits double (0.10° / 0.06 rad/s) to avoid
   chatter; the anchor tolerance stays 0.15°. Noisy raw drive velocity does not
   veto stationary readiness. It remains unfiltered for freefall/catch control.
   The same history is rechecked against fresh feedback on every strike request;
   feedback/history gaps over 40 ms and each new strike clear the window. The result reports
   measured peak drop and upward return overshoot.

J7 has a private filtered SocketCAN receiver in a **separate spawned Python
process**, independent of the GUI interpreter/GIL. The target rate is **500 Hz
while striking** and **100 Hz in stationary hold**. MIT replies supply position,
velocity, torque and powered state, with Linux kernel receive timestamps.
Active motion still faults on feedback or command gaps over **20 ms**. In idle
hold, a late command (20–250 ms) refreshes the same hold target before accepting
another press and collects the hold reply. Valid position history is retained
through a short scheduler slip; it does not by itself mean the arm moved.
During idle hold, feedback age or a history gap over 40 ms clears readiness and
requires a new window. A strike still requires feedback no older than 20 ms;
old feedback never starts a strike or newly establishes readiness.
A command gap over **250 ms** still faults.
Errors report the phase, measured gap and threshold. This is not hard real time.

The Tk loop displays one status snapshot per update with a 100 ms UI freshness
budget. This is not permission to strike on 100 ms old motor data: the worker
rechecks feedback against its unchanged 20 ms limit before accepting a press.
The Tk loop only submits a button request and displays status; status delivery is
nonblocking so a stalled GUI cannot stall the controller. Other joints retain
their existing controller and feedback rate. GUI-owned J7 writes are excluded
throughout the session; ordinary state queries continue during process startup
until the child supplies feedback. Entry confirms disabled state and MIT mode
before enabling, confirms running feedback, and retries ignored mode writes or
enables only while arming. No mode switching or enable retries occur during a strike.

J7 remains enabled in MIT between attempts. There is no per-strike disable,
mode switch, or slow CSP correction. **Center + Relax**, after J7 is ready,
joins the worker, restores CSP at the same stationary anchor, and confirms mode
and fresh powered feedback before centering. Emergency Relax and shutdown join
the worker before disabling, so it cannot subsequently re-enable J7.

The existing independent J1–J6 drift, zone, motor-fault and stall behavior is
retained. A generic session fault joins the worker and enters the existing
powered hold; a named drive fault isolates that drive. A detected stall remains
the existing whole-right-arm relaxation exception. ST7 is informational only.

Physical verification of recording playback and idle MIT hold (not loaded
strikes): [deadline repair test report](diagnostics/right_camera_playback_mit_deadline_verification.md).

Readiness was physically verified with record3, a steady button, a short idle
controller pause, and three unloaded strikes: [settling verification report](diagnostics/right_camera_playback_mit_settling_verification.md).
That run predates the lighter damping and continuous-acceleration turnaround.
The turnaround changes have offline regression/dynamics coverage only; no new
physical tests were performed. They use encoder-based trajectory prediction,
not cymbal contact or audio detection, and do not guarantee absence of contact force.
See the [offline turnaround verification report](diagnostics/right_camera_playback_mit_turnaround_verification.md).

### Hardware-test calibration

The defaults are **initial tuning, not physical calibration**. Effective inertia,
braking capability and delivery delay depend on the loaded arm. Offline dynamics
tests exercise synthetic plants, delays and repeated strikes; they do not prove
a particular real-world maximum drop or identical sound. These optional flags
apply only to `--hardwaretest`:

| Flag | Default | Meaning |
| --- | --- | --- |
| `--mit-fall-kd` | `0.04` | Gravity-descent damping |
| `--mit-inertia` | `0.02` | Effective loaded J7 inertia, kg m² |
| `--mit-brake-accel` | `40` | Achievable catch acceleration, rad/s² |
| `--mit-latency-ms` | `4` | Command/actuation delay estimate, ms |

Do not assume increasing the braking-acceleration estimate improves accuracy:
an overestimate can start the catch too late. Calibrate against measured peak
drop/return overshoot. The goal is near the entered depth, not a guaranteed hard
mechanical stop. `--hardware` and pure `--test` retain their separate controllers.

The launcher prefers the scoring-enabled `/home/jason/st7` checkout and falls
back to the current `/home/jason/Proyectos3/st7` sibling checkout, starts its
selected `runs/cymbal-fmn-tcn/best.pt` checkpoint, requires
`runs/cymbal-normality/reference.npz`, and routes the detector through the
stable TONOR TD510 PipeWire source
`alsa_input.usb-TONOR_TONOR_TD510_Dynamic_Mic_0000KT5a300000135-00.analog-stereo`. If the
stable name changes but exactly one source containing `TONOR` is connected, it
is selected automatically. ST7 is given a two-second microphone/model warm-up.
Normal `--hardware` keeps Start disabled until its heartbeat reports that the
normality scorer is enabled and ready; `--hardwaretest` treats ST7 as
informational and does not make its heartbeat a motion prerequisite.

With `--hardware`, the same launcher also opens the connected classic ESP32
through its stable CP2102 path
`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
at 115200 baud. Use `--esp-port /dev/ttyUSB0` only if an explicit override is
needed. Start remains disabled until this hi-hat connection is ready. Normal
playback never flashes the board; the copied firmware is under
`esp32_hihat/motor_beat/`.

`--hardwaretest` requires a recording and processed camera because it restores
the center/open/load/Continue/playback/pink-zone sequence. The TONOR/ST7 process
is started for informational hit logging, but its hit decision does not control
motion. The ESP32 connection remains unused.

In every mode, use `--recording` to start with a different file, or use **Select
right-arm recording…** in the control panel before pressing Start. The recording
selector remains available in `--hardwaretest`.

The control panel has two side-by-side stop controls. **Emergency Relax**
immediately disables the right arm and gripper. **Center + Relax** cancels the
active playback/alignment/strike workflow, restores the ordinary motor speed if
playback or a strike was active, moves from the current pose to the customized
right center, and disables the right arm and gripper only after the center is
reached. During continuous striking its label changes to **Stop 100 BPM
striking: Center + Relax**; an active stroke finishes its return before the
center command.

A temporary status appears directly underneath the title while something is
loading. Camera startup identifies the processed camera/model operation. A
selected recording reports each active preflight stage, including JSON parsing,
recorded TCP validation, path checks, recovery-to-center checks, and playback
timing. The status is removed completely whenever no loading operation is in
progress.

With `--hardware`, this is powered playback; with `--hardwaretest`, it is a
powered playback/alignment-and-MIT test. Keep the path clear and be prepared to
use the physical emergency stop or Emergency Relax. Both powered modes enable and
command the right arm and gripper after Start is pressed. `--test` is the
pure-simulation exception and has no physical motor connection. The
implementation has been verified offline only; no physical test was run during
development.

## Normal `--hardware` sequence

1. Select and preflight a right-arm JSON recording while motors remain disabled.
2. Start only when encoder feedback is fresh and a cymbal is visible.
3. Move to the customized right center, with J7 at +1.4 rad, while opening the
   right gripper to −3 degrees.
4. Wait indefinitely for the user to load the drumstick and press Continue.
5. Close the right gripper to +7 degrees and retain that command through the
   remainder of the sequence.
6. Move from center to the recording's first pose.
7. Temporarily set right J1–J7 to a 0.8 rad/s firmware speed limit and follow
   the complete recorded path. The recording's gripper sample is retained as
   metadata but is not replayed because the drumstick gripper must remain
   closed.
8. Settle at the recorded final pose, restore J1–J7 to the ordinary 0.4 rad/s
   setting, and wait for one fresh simultaneous cymbal and directly observed
   stick-tip frame.
9. If that one tip is anywhere inside the larger visible pink rectangle, accept
   the recorded endpoint immediately. If it is outside, run the playback-informed
   Cartesian hill climber. At every settled hill-climber pose, the first fresh
   detected tip anywhere inside the visible pink rectangle is accepted
   immediately. Outside detections are still collected into robust measurements
   to score and choose the next move. There is no smaller invisible target,
   second confirmation, or timed hold.
10. Preserve the accepted joint pose, quickly derive consecutive zone-bounded
    J7-only strike targets at −5, −6, −7 degrees and so on, set the fast J7
    speed, and start the first strike without another camera-availability gate.
11. Every attempt is a complete fast round trip from the preserved pink-zone
    pose: anchor → J7 −5° → anchor, then anchor → J7 −6° → anchor, then
    anchor → J7 −7° → anchor, and so on through the safe targets.
12. For each attempt, accept only a timestamped ST7 `HIT` whose sound onset
    belongs to that attempt. Camera-box movement is not part of this decision.
    Because ST7 finalizes onsets after lookahead, hold the returned anchor for
    up to 1.25 seconds before declaring that depth unsuccessful.
13. If no hit is reported during discovery, advance to the next safe cumulative
    1-degree depth. If every safe depth is exhausted, report failure, restore
    normal speed, return to center, and relax.
14. The first detected hit fixes the strike depth `D`. Do not test above or
    below `D`, do not run a fine-tuning phase, and do not use `normality_score`
    to select the depth.
15. Repeat that exact first-detected depth in a triplet-based swing ride at 100
    BPM. In `--hardware`, start immediately with
    the extra pickup before beat 1, then play beat 1, beat 2 + its swung extra,
    beat 3, and beat 4 + its swung extra. Starting at the pickup, the intended
    strike-target deadlines are 0.200, 0.600, 0.400, 0.200, 0.600, 0.400, and
    0.200 seconds before repeating. Return toward the anchor at 3.5 rad/s. When
    a deadline requires it, interrupt that return after at least 4° of rebound
    and command the same strike target at 3.5 rad/s. Only the four main
    quarter-note beats advance motor 2's existing alternating hi-hat sequence:
    close to 110° on beats 1 and 3, and return to 0° on beats 2 and 4. The
    ride extras send no hi-hat command. No motor-1 `K` command is sent.
16. Continue indefinitely until the user presses
    **Stop 100 BPM striking: Center + Relax**. If Stop is pressed during a
    stroke, finish its fast return, restore J7 to 0.4 rad/s, move to the
    customized center, and relax the right arm.

In `--test` mode, steps 8 through 14 are replaced by one transition: after the
recorded endpoint settles and normal playback speed is restored, the endpoint is
accepted as the strike anchor and the safe 10-degree target is passed
directly to the same step-15 100 BPM loop. Camera, ST7/TONOR, and ESP32 readiness
are not required.

In `--hardwaretest` mode, steps 1 through 9 are retained: recording preflight,
center/open, user-authorized gripper close, playback, and one-frame visible-pink
acceptance all run normally. Step 10 is replaced by the editable manual MIT
panel. The entered maximum-drop goal is measured from the fixed pink-zone
anchor, and the path is checked against the J7 limit and sampled safe zone. Each
press requests one damped gravity fall, predictive catch and smooth MIT return.
J7 stays in MIT hold between strikes, and another press is accepted only after
encoder positions stabilize. There is no automatic depth search, sound-triggered
retry, continuous swing, or ESP32 hi-hat use.

The recording may begin and end at arbitrary poses. Its samples are absolute
right-arm encoder joint positions, so the program always moves to the first
recorded pose before starting the timed path.

## Recording preflight and timing

The loader requires schema `openarmx-right-motion-recording-v1`, right-arm joint
order, finite strictly increasing timestamps, compatible units, and the current
URDF SHA-256. It recomputes TCP coordinates from the joints instead of trusting
the file's TCP values.

Before Start can enable anything, the program validates:

- all recorded joints against current URDF limits;
- every recorded TCP sample and interpolated sample-to-sample segment against
  buffered `right_zones/zone1.json`;
- center-to-recording-start and recording-end-to-center transitions;
- representative direct center returns from throughout the recording.

Tiny encoder-sized limit discrepancies are clipped only to legal command bounds;
larger violations are rejected. Recorded-path motion alone uses a temporary
0.8 rad/s firmware speed limit. If the fitted recording asks any joint to move
faster than 90% of that limit (0.72 rad/s), the complete timing is slowed by one
global scale factor. This preserves the recorded geometric path and relative
timing while providing tracking margin. The normal 0.4 rad/s setting remains in
effect while moving to the recording start and is restored only after the arm
physically reaches the final pose, before alignment, recovery, or centering.
Actual sample timestamps remain available in the JSON.

During playback, a tracking monitor stops the sequence on a stalled joint or a
15-degree error sustained for one second. Ordinary failures use the camera
program's center-before-relax recovery; a detected stall retains the existing
immediate-relax behavior.

## One-degree J7 cymbal-hit search and continuous striking

The single accepted visible-pink frame establishes the strike anchor. Before
changing motor speed, the program quickly generates J7 targets starting at a
5-degree reduction and increasing by exactly 1 degree: 5°, 6°, 7°, 8°, and so
on. All other arm joints remain at the anchor values. It does not perform a
second camera check and does not wait for a two-second camera hold.

The strike planner retains only consecutive targets whose sampled J7 strike
path remains inside buffered `right_zones/zone1.json`. The accepted recording
or Cartesian-correction path has already validated the anchor and its normal
center return. Limiting this final calculation to the path that will actually
be struck removes the previous long all-depth center-return preflight.

Planning stops before the first unsafe increment or the J7 lower URDF limit. If
even the first 5-degree attempt is unsafe, the strike phase is rejected and the
normal center-before-relax failure path is used.

Only right motor 7 is permitted to use the fast strike speed. Every downward
cymbal leg and every return toward the preserved pink-zone anchor uses
3.5 rad/s. Ordinary Cartesian motion remains 0.4 rad/s.
The strike controller sends the exact target without the ordinary goal
controller's additional error overshoot. It considers the J7 endpoint reached
within 2 degrees and reverses immediately, without the normal 0.6-second
settling wait. It also has a 0.5-second stall monitor and a 3-second per-leg
timeout.

During an active outbound or return leg, right-J7 encoder requests run at the
20 ms control-loop limit (50 Hz). The other 15 arm/gripper feedback channels
retain their existing 50 ms interval (20 Hz), and J7 returns to that ordinary
rate while waiting between strikes. At reversal, the already-selected 3.5 rad/s
limit is retained and one motor-7 position frame sends the anchor immediately;
the return is no longer delayed by another speed frame or J1–J6 position frames.

ST7 listens continuously on the TONOR microphone. Its bridge requires the
normality scorer and preserves both the
estimated sound-onset time and notification time, so model lookahead cannot make
a late event from one depth count as the next depth. A hit is accepted only
after strike motion has begun and only when its onset falls within the current
outbound/return interval (with small timestamp tolerances). The returned anchor
is held for up to 1.25 seconds so ST7 can deliver its delayed decision.

A detected hit is latched rather than interrupting a fast leg: the arm reaches
that attempt's target and performs its fast return to the exact preserved
anchor first. In `--hardware`, it then immediately selects that first detected
degree value and plays it in the 100 BPM swing ride. There is no fine-tuning phase, no test above
or below the detected value, and `normality_score` does not participate in
target selection.

The ride begins with the extra pickup immediately before beat 1. Its repeating
bar is beat 1; beat 2 plus the third triplet partial; beat 3; beat 4 plus the
third triplet partial. At 100 BPM a quarter note is 0.600 seconds and one
triplet partial is 0.200 seconds, producing the repeating
0.600/0.400/0.200/0.600/0.400/0.200 spacing after beat 1.

The integrated ESP32 controller remains on the main quarter-note beats and is
not advanced by the pickup or either ride extra. The firmware's encoder zero is
the open position. Beat 1 sends `C` to close and hold at its exact 110° target;
beat 2 sends `O` to return to encoder zero and release; beats 3 and 4 repeat
`C`, `O`. Thus the hi-hat still closes every other main beat even though right
J7 adds the swung ride notes. The controller sends a 100 ms `H` heartbeat for
the firmware's 400 ms watchdog and never sends `K`, so firmware motor 1 stays
released and is rhythmically replaced by right J7.

The opening pickup establishes the timing grid when its strike target is
reached. For each later ride event, the controller treats the musical time as a
strike-target deadline. During the return, it continuously measures the live J7
encoder distance from the strike target and estimates the required outbound
time as distance divided by 3.5 rad/s plus a 40 ms command/control margin. When
that time reaches the remaining time before the deadline, the controller
reverses direction from the 3.5 rad/s return into the 3.5 rad/s downstroke. It requires at
least 4° of real rebound first, twice the strike controller's 2° reached
tolerance, so a reversed stroke cannot be accepted immediately without moving.
The partial-return path is a subset of the already validated target-to-anchor
path and never moves past the anchor.

On 0.400 and 0.600-second gaps the arm will normally reach the full anchor and
wait until the predicted downstroke start. On a 0.200-second pickup-to-downbeat
gap, or whenever the physical return is slower, it can reverse partway instead
of delaying the next strike until a full return. The hi-hat command remains on
the main beat deadline and is not sent early with a predicted J7 reversal. The
control panel's **Center + Relax** button changes to **Stop 100 BPM
striking: Center + Relax** during the loop. Stop latches during an active
stroke, disables deadline reversal, finishes a full 3.5 rad/s return, restores
0.4 rad/s, centers, and relaxes. Camera or
sound-process loss after first-hit depth selection is not needed to drive the
learned loop, although all encoder, zone, motor, Emergency Relax, and
program-close safety paths remain active. If restoring normal speed itself
fails, the program disables the right motors instead of issuing a center
movement at strike speed.

Stop also immediately sends `O` so the hi-hat returns to 0° while the arm
finishes its safe return and center sequence. Closing the application sends `S`
to release the ESP32 outputs. An ESP32 serial or firmware fault prevents Start;
if it occurs during continuous striking, the program stops the arm loop and
uses the same center-and-relax path.

## Camera evidence during playback

For every fresh simultaneous cymbal and directly observed YOLO drumstick tip,
the program pairs the image-center error with the arm's actual encoder-derived
TCP `(X,Y,Z)` coordinate. Near the end of playback it fits the observed change
in camera error against Cartesian motion.

That estimate only reorders the six first directions:

```text
+X, -X, +Y, -Y, +Z, -Z
```

It never removes a direction. For example, if increasing Y near the end of the
recording made the image error worse, `-Y` is preferred and `+Y` is tried later.
If there are too few observations, too little motion, or nearly unchanged camera
error, the deterministic original order is used. Normal 10, 5, and 2.5 mm step
levels, safe candidate transitions, anchor returns, and center returns remain.

The Cartesian candidate solver retains the recorded endpoint's J2 angle and
uses the other six joints. This avoids forcing an arbitrary recording endpoint
through the original camera program's centered-J2 assumption.

## Single-frame visible-pink acceptance

The visible pink rectangle remains the complete acceptance zone for the
recorded endpoint:

- The first fresh simultaneous cymbal and directly observed tip after playback
  is enough. If that tip is inside the visible pink rectangle, striking begins
  without a hold or Cartesian correction.
- If that tip is outside, hill-climber candidates use smooth distance to the
  rectangle's center and the existing robust candidate measurements to choose
  each next move. At every settled endpoint or candidate pose, one fresh direct
  tip detection anywhere inside the visible pink rectangle starts striking
  immediately. The detection does not need to persist, and there is no smaller
  invisible target, confirmation frame, or two-second hold.

## Windows, options, and files

With `--hardware` or `--hardwaretest`, the launcher opens RViz, one right-arm
control panel, and the processed Y2 camera window; the panel includes the live
ST7/TONOR line. Hardware test additionally shows the editable MIT strike panel
and omits the ESP32 status row. `--test` opens RViz and the control panel but no
processed camera window. Camera options apply to either powered mode, for example:

```bash
./start_beat.sh --hardware \
  --recording recordings/my_motion.json \
  --camera 1 --conf 0.50
```

The processed camera preview also shows a playback-only cymbal-box diagnostic:

- `CYMBAL BOX: BASELINE` for the first detected cymbal box;
- `CYMBAL BOX: STABLE` when width and height changes from the preceding
  processed detection are each at most 4 pixels;
- `VISUAL CYMBAL MOVEMENT (NOT HIT TRIGGER)` when combined width/height change
  exceeds the display threshold;
- `CYMBAL BOX: NOT DETECTED` when no valid cymbal box is available.

It also continuously shows the directly observed stick tip's membership in the
visible pink rectangle:

- `STICK TIP IN PINK ZONE: YES` when the observed YOLO tip is inside it;
- `STICK TIP IN PINK ZONE: NO` when the observed tip is outside it;
- `STICK TIP IN PINK ZONE: UNKNOWN` when a valid cymbal or directly observed
  YOLO tip is unavailable.

The second overlay line shows the current width, height, and signed width/height
changes in pixels. The overlay explicitly labels visual movement as **not the
hit trigger**. It remains display-only and adds nothing to the camera datagram;
only ST7 sound events decide hits. This does not alter camera target geometry,
playback, direction ordering, or alignment. The original right-camera Cartesian
camera process also remains unchanged.

Key new files:

- `start_beat.sh`
- `launch_right_camera_playback.py`
- `camera_playback/app.py`
- `camera_playback/audio.py`
- `camera_playback/audio_bridge.py`
- `camera_playback/camera.py`
- `camera_playback/trajectory.py`
- `camera_playback/visual.py`
- `camera_playback/hill.py`
- `camera_playback/planner.py`
- `camera_playback/strike.py`
- `camera_playback/hihat.py`
- `esp32_hihat/motor_beat/motor_beat.ino`
- `esp32_hihat/flash_beat.sh`

The existing `start_right_camera_cartesian.sh`,
`launch_right_camera_cartesian.py`, and `camera_search/` files remain unchanged.
