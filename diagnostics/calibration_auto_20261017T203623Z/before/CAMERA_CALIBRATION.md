# Gemini 2 → OpenArmX calibration (right arm, no motor control)

This is **camera-to-robot extrinsic calibration only**, not neural-network training,
MoveIt, a depth obstacle mapper, or a trajectory program. It is separate from
`start_beat.sh`. Existing recordings, custom centers, gripper models and playback
programs are not changed.

## Start

From any directory:

```bash
/home/jason/Proyectos3/X5/calibrate_camera.sh preview
```

This opens a setup window. Press **Start camera only** to check board visibility.
Preview never opens a CAN interface. Close it before starting calibration:

```bash
/home/jason/Proyectos3/X5/calibrate_camera.sh
```

No camera or CAN device opens until you confirm the checklist and press **Start
camera + read-only right encoders**. This program does **not** automatically move
the arm, enable/disable torque, hold it up, center it, open/close the gripper, or
configure CAN. It does not launch ROS or `start_beat.sh`.

**The motor electronics must be powered and `can0` available, but the right motors
must already be torque-disabled. Support the right arm by hand throughout
repositioning and capture. Do not let it drop.** Closing the program or a fault
does not change motor state. The left CAN bus (`can1`) is never opened. There is
no automatic centering/relaxation fault routine here because this utility never
takes control of torque or movement.

Stop other robot controllers, `start_beat`, Orbbec Viewer and camera applications
first. Never run this utility alongside another motor controller. An existing
per-bus lock prevents cooperating X5 tools from sharing `can0`; it cannot stop
unrelated software that ignores that lock.

## Mounting your printed board

Use the supplied board, not a newly generated random AprilTag:

- A4: [`output/pdf/openarmx_charuco_6x4_40mm_a4.pdf`](output/pdf/openarmx_charuco_6x4_40mm_a4.pdf)
- US Letter: [`output/pdf/openarmx_charuco_6x4_40mm_letter.pdf`](output/pdf/openarmx_charuco_6x4_40mm_letter.pdf)
- Machine-readable definition: [`output/pdf/openarmx_charuco_6x4_40mm.json`](output/pdf/openarmx_charuco_6x4_40mm.json)

Print at **100% / Actual size**, not Fit to page. Measure several printed squares:
each must be **40 mm**, including in both paper directions. The complete pattern
is **240 × 160 mm**, with 28 mm ArUco markers. A printer scale error becomes a
calibration scale error; a good reprojection score alone cannot detect that.

Bond the paper flat to a light, rigid backing. Attach that backing rigidly to
the **right hand/gripper motor housing that moves with the last wrist joint**:
the structure represented by `openarmx_right_hand`, or the rigid `link7` structure
after joint 7. Do **not** attach to the moving/sliding claws, the stick, a rubber
band, a flexible bracket, or a forearm section before the last wrist joint.
Do not obstruct joints, cables or the stick, and do not make a heavy lever on the
wrist. Keep the entire board and its white margin visible where practical.

You do **not** need to measure the board's exact offset or orientation on that
housing. The solver estimates the fixed board-to-hand transform simultaneously
with the camera-to-base transform. The attachment must not slip or flex between
poses. The custom finger STL placement and the stick's length are not needed
for this calibration; they are needed separately for future collision geometry.

The camera stays secured to the central pole; the robot base stays clamped.
If the camera/base relationship or board mounting changes during capture,
**start a new session**. Let table/pole vibration settle. The board can be
removed after a successful completed calibration without invalidating the saved
camera-to-base transform. Remounting the camera or changing the robot's base
geometry/joint-zero convention requires rechecking/recalibrating it.

## Capture workflow

1. Confirm all three setup checks and press **Start**. Wait for live corner
   overlays and healthy disabled-motor feedback. Missing/faulted/enabled right
   motors stop capture; the utility does not try to reset them.
2. Support and manually move the right arm into a clear pose where the board
   faces the camera. Keep your body, board and stick clear of drums/cymbals/table.
   Do not force a joint through a mechanical stop. No safe-zone or collision
   guard is active in this utility.
3. Stop moving. Press **Capture training** (or Space with the window focused),
   then hold still for about **2 seconds**. Rejections explain the problem;
   rejected samples are not added to the fit. Do not move until the saved/rejected
   message appears. Support a stable rigid part rather than bending the board.
4. Repeat for **at least 15 training poses**; 20–25 well-spread poses are preferable
   to many almost-identical ones. Change position, height and camera distance
   within safe reach. Also tilt and turn the wrist around **at least two distinct
   axes**—translations alone, or spinning about just one axis, are insufficient.
   Aim for several visibly different tilts rather than keeping the board
   perfectly face-on. Explore the visible workspace, not just one tiny patch.
5. Press **Solve / freeze training**. The fit must pass consistency thresholds
   and the rotation/position diversity checks. A failed fit writes
   `training_report.json` for diagnosis, not an approved calibration. Undo is
   available before solving; excluded captures and their images are retained.
6. After a passing fit, collect **at least 5 NEW validation poses** using
   **Capture validation**. Spread these across positions and tilts, too. These
   observations are **never used to fit or refine** the camera or mounting pose.
7. Press **Validate / export**. Read the displayed **PASS/FAIL** and held-out
   errors. A finalized session cannot collect more samples; start a new session
   if validation fails. Support the arm while quitting and returning it manually
   to a physically supported rest. The program will not hold or lower it for you.

Do not use the usual centering motion just to run this utility; no center is
assumed and the tool will not send one. The resting stick need not be visible:
only the board must be visible **at captured calibration poses**.

## What is measured and saved

Every session gets its own directory:

```text
calibration_sessions/<UTC timestamp>-<unique suffix>/
  session.json              # metadata, joints, poses, phase, frozen fit, validation
  target.json               # exact printed-board definition
  model.urdf                # snapshot of the FK model used
  images/0001.png ...       # unannotated captured images; SHA-256 recorded
  can_query_audit.jsonl     # every allowed outgoing state-request packet
  camera_stderr.log         # SDK failure diagnostics
  orbbec_sdk_logs/          # SDK warnings, including frames dropped before Python receives them
  camera_decode_report.json # encoding, frame totals, bounded JPEG errors, frame gaps and stop reason
  training_report.json     # produced on a numerical training solve
  calibration.json         # produced on validation; check validated, not just existence
```

`calibration.json` contains:

- `T_base_color`: `world` ← Gemini color optical frame.
- `T_hand_board`: `openarmx_right_hand` ← printed board.
- `T_base_depth`: `world` ← raw Gemini depth optical frame, derived using the
  SDK's factory depth-to-color extrinsics, with millimetres converted to metres.
- Active color intrinsics/distortion, depth profile metadata, serial, firmware,
  depth work mode, URDF/target/encoder-source hashes, capture confirmations and
  per-sample training/held-out residuals.
- `validated`: whether this dataset passed the configured consistency checks.
- `motion_authorized: false` and `depth_mapping_verified: false`, even on a pass.

Matrices use column vectors, metres, and `p_destination = T_destination_source
@ p_source`. `world` coincides with `openarmx_body_link0` in the current model.
The exported optical frame names are names assigned by this standalone tool,
not automatically published ROS TF frames. Check frame names when integrating
with a future ROS camera driver; do not apply the raw-depth transform to an
already color-aligned point cloud as though they were the same frame.

The equations are `A_i X = C B_i`, where `A_i` is measured base←hand FK, `B_i`
is observed color←board, `X` is the unknown board mounting pose and `C` is the
unknown camera pose. We reuse `safe_zone.geometry.Model` and the established
`safe_zone.encoder.encoder_to_joint` convention, including the right J5
−8.5° correction. No zero is rewritten. There is no assumption that the board
origin sits on the wrist origin.

## Camera/depth details

The bridge selects **only serial `AY6R463004K`**, not an arbitrary webcam, and
requests **1280×720 uncompressed YUYV color at 15 FPS** using the installed Orbbec
SDK's LibUVC backend, then converts complete frames to BGR for OpenCV. It does
not silently fall back to cropped 640×480 RGB. Actual-frame factory intrinsics
are used; FOV angles are **not** used to guess a camera matrix. The board layout
is checked against the PDF JSON, including the non-legacy even-row pattern.

The default avoids the camera's unreliable MJPEG stream rather than accepting
damaged JPEGs. Each YUYV frame must contain exactly **1,843,200 bytes**; a partial
frame is never padded or used. The SDK can discard incomplete frames before
Python receives them; the requested rate is not a guarantee of that many usable
frames. SDK warnings remain available in `orbbec_sdk_logs/` for diagnosis.

An explicit `--color-format mjpeg` option remains for diagnostics, not automatic
fallback. In that mode, incoming JPEGs use a strict libjpeg wrapper. **Any decoder
warning or corrupt-image error rejects the entire frame**; partially recovered
pixels are never shown to the board detector or saved as calibration samples.
This is not terminal-output suppression and does not repair/re-encode damaged
images. Occasional bad frames are skipped, with a single **Damaged JPEGs skipped**
counter in the preview rather than repeated terminal warnings. Each accepted
frame header records the cumulative checked/rejected counts. On exit, a
best-effort `camera_decode_report.json` saves totals, up to 20 error examples,
last received/accepted/rejected timestamps, maximum frame gaps and stop details.
If the newest usable image is over **1 second old**, pose capture is paused and
any in-progress pose is canceled; stale pixels are never used for detection or
capture. Fresh frames returning before **3 seconds** restore capture availability
without discarding the session. The interrupted pose must be captured again.
A **3-second usable-frame outage** after streaming begins stops the session.
Startup, bridge/protocol and encoder failures retain their separate checks.
Error text distinguishes absent incoming frames from a current run of rejected
JPEGs: earlier isolated JPEG warnings are not presented as the cause of a later
stream stall. Stationary-window frame/encoder gap checks remain unchanged.
Resolution, requested frame rate and field of view are unchanged by the default
encoding switch. Intrinsics/distortion are always read for the actual profile;
encoding and rate mismatches stop rather than silently selecting another mode.

Calibration sees the printed pattern in **RGB**. The bridge also reads the
**1280×800, 10 FPS depth profile** and its factory transform to the active color
profile. **Depth streaming is not enabled during this utility.** This avoids
needing simultaneous full-resolution RGB/depth bandwidth just to solve the
rigid mounting calibration. It does not crop or narrow the intended depth
mapping profile. The current device work mode is recorded, not changed.

This is not a validation of depth accuracy, depth range/FOV coverage, reflective
cymbals, unseen space, or a collision map. Before trajectory execution, verify
the depth overlay/robot-frame geometry with independent physical measurements,
model the custom gripper and stick, and establish how unknown/occluded space is
handled. Passing calibration residuals cannot prove that an unseen route is clear.

Factory RGB intrinsics are used rather than recalibrated. Unsupported distortion
models fail explicitly instead of being treated as OpenCV Brown-Conrady. A future
intrinsics-calibration step may be needed if the factory intrinsics do not fit
the active image accurately.

## Acceptance limits (consistency checks, not a safety guarantee)

- Capture: ≥10/15 ChArUco corners, non-collinear/well-sized image coverage,
  board reprojection RMS ≤1 pixel, no strongly ambiguous planar pose.
- Stationary window: ≥5 usable images and ≥10 encoder batches over ≥1 second;
  encoder samples bracket the selected image sequence; no large time gaps;
  maximum joint spread ≤0.35° and common-corner movement ≤2.5 pixels.
  Camera receipt/encoder times are retained. This is **not hardware-synchronized
  moving-arm capture**; keeping the arm/board still is essential.
- Similar poses are rejected unless they differ by >2 cm or >7° from existing
  samples. Training needs multiple rotation axes, a ≥25° maximum relative
  rotation, and ≥8 cm aggregate position span.
- Both training and held-out validation must satisfy translation RMS ≤5 mm /
  max ≤10 mm, rotation RMS ≤1° / max ≤2°, and reprojection RMS ≤4 pixels /
  max ≤8 pixels. No outliers are silently removed from these reports.

The translation/rotation errors compare camera-observed board poses with poses
predicted through FK and the fitted mounting transform. They are **internal
consistency errors**, not measured absolute obstacle-location accuracy. Incorrect
robot link geometry, joint zero errors, paper scale error and mount flex can
still produce systematic errors. Do not simply enlarge collision margins or
loosen thresholds to conceal a failed calibration.

## Offline commands and troubleshooting

```bash
./calibrate_camera.sh --help
./calibrate_camera.sh build                     # compile only; no device access
./calibrate_camera.sh test                      # synthetic / mocked tests only
./calibrate_camera.sh solve calibration_sessions/<session-directory>
```

`solve` uses saved observations and the frozen fit if present; it does not connect
hardware, move the robot, or mix held-out poses back into training. A partial
capture session is preserved on exit, but live capture does **not** resume across
program restarts: that would risk silently combining different board mountings.
Close the GUI before using the offline solve command on its session.

Optional camera settings, without changing the FOV (the default is YUYV at
15 FPS; alternatives are not necessarily more reliable):

```bash
./calibrate_camera.sh preview --fps 10
./calibrate_camera.sh capture --fps 10
./calibrate_camera.sh capture --serial ACTUAL_GEMINI_2_SERIAL
./calibrate_camera.sh preview --color-format mjpeg --fps 15  # diagnostic only
```

- **Camera not found/busy/no frames:** close other camera users; inspect
  the on-screen **SDK detail**, `camera_stderr.log` and `orbbec_sdk_logs/`. A bridge startup error
  is not necessarily a disconnected camera. Check the USB connection when the
  error actually indicates detection/streaming trouble. The tool never substitutes a
  different webcam or modifies device firmware, laser/protection settings or
  depth work mode. No software can guarantee the requested field of view if
  the device is configured incompatibly; profile/parameter mismatches fail.
- **Missing fresh feedback / enabled motor / fault:** stop and diagnose the
  existing robot/CAN setup separately. All eight right motors, including the
  gripper, must report healthy and disabled. Reading gripper feedback does not
  open the claws. The utility never runs `sudo`, configures a bus, or resets motors.
- **Damaged JPEGs skipped:** a few rejected frames can be tolerated; those images
  are not used for calibration. If capture stops because no usable frames arrive,
  support the arm, close the program, and investigate the camera/USB connection.
  Check `camera_decode_report.json`, `camera_stderr.log` and `orbbec_sdk_logs/`.
  The SDK can discard incomplete frames before Python sees them, so the Python
  JPEG counter alone does not count all camera/transport damage. The counter is not
  proof of a particular cable/device fault. Do not disable integrity checks just
  to obtain a fit. Start a new capture session after restarting the application.
- **RGB preview paused / frame delivery stalled:** a brief gap pauses capture
  automatically and cancels any pose being collected. If fresh frames return
  within the grace period, retry that pose. A sustained outage still stops the
  session. The report's earlier JPEG count is historical, not proof that JPEG
  corruption caused the delivery gap. Keep supporting the arm throughout.
- **Board not detected / ambiguity / poor reprojection:** use diffuse lighting,
  remove glare, flatten the backing, expose more corners, and use a modest tilt.
- **Motion rejected:** support the arm and rigid wrist housing more steadily;
  wait for vibration to settle, then press Capture again. A helper may make
  support/capture easier. Do not enable a separate holding controller alongside
  this disabled-arm-only tool.
- **Fit/validation failed:** inspect per-sample residuals and board mounting/
  scale, collect more diverse poses, and check robot FK/encoder zero conventions.
  A low image reprojection error alone does not prove correct robot calibration.
- **Session failure/quit:** sockets and camera process are closed; motor state is
  never changed. Because no motor torque was enabled, there is no torque to
  release. Keep supporting the arm yourself.

The launcher uses the installed **system Python 3** (OpenCV 4.6 with ArUco,
NumPy, SciPy, Pillow and Tk), not the separate Y2 environment. The SDK bridge
builds with `g++` against `/home/jason/Applications/Gemini2/sdk`. The strict JPEG
decoder builds with `cc` and the already-installed libjpeg development headers
and library. Both binaries live under `camera_calibration/build/` and rebuild
automatically when their sources change. No dependency installation or unrelated
environment changes are performed.

Development verification includes 62 offline tests: synthetic known transforms/
unknown board mounts, noise/degeneracy/outlier checks, raster target detection,
stationary capture rejection, mocked right-only CAN auditing, session persistence,
held-out validation, startup-error propagation, and a device-free regression
against the installed SDK's actual coordinate-transform math. JPEG tests verify
clean-image pixel parity, rejection of truncated scans even when an end marker
exists, no native warning flood, recovery on subsequent clean frames, dimension
checks, and a hard stop for persistent corruption. GUI regression tests cover
brief-outage capture cancellation/recovery, no stale-frame detection or hotkey
capture, correct training/validation/preview button restoration, sustained-outage
shutdown and correct distinction between old JPEG warnings and delivery stalls.
SDK transport logs are retained separately from the JSON stream and terminal;
failure details include bounded log tails rather than unbounded warning output.
Raw-color tests check YUYV pixel ordering/colors, exact frame length, the real
camera-reader path with mocked bytes, rejection of format/rate mismatches, and
the CLI's default format/rate without opening devices.

On 2026-10-16 local time, the first live preview exposed a bridge bug: the SDK's
3D transformation helper rejects source points with zero depth. The converter
now uses a positive-depth translated basis and subtracts the anchor correctly;
SDK errors are displayed rather than being masked as camera disconnections.
Startup metadata is fully assembled before being written to the protocol stream.

After this fix, camera-only checks received 77 distinct 1280×720 RGB frames and
the actual Tk preview displayed live frames. The checks opened **no CAN sockets**,
sent **no robot commands**, and stopped/released the camera afterward. Evidence:
[`diagnostics/camera_calibration_preview_20261017T053738Z/report.json`](diagnostics/camera_calibration_preview_20261017T053738Z/report.json)
and [`gui_report.json`](diagnostics/camera_calibration_preview_20261017T053738Z/gui_report.json).
The board was not detected in that resting-view test. **Physical pose capture,
real-board calibration accuracy and depth obstacle mapping remain unverified.**

The later JPEG-warning investigation reproduced warnings on 5 of 120 camera
frames, despite OpenCV returning an image for each. Replaying three saved damaged
JPEGs through the strict decoder rejected all three without terminal warnings;
the saved clean frame was accepted. A subsequent 30-second camera-only Tk preview
received 452 clean frames, with empty application/SDK stderr logs, no CAN socket
attempts and no observer constructed. The camera was released afterward. This
verifies the rejection policy and working preview, **not a repair of the
underlying intermittent camera/USB corruption**. Evidence:
[`strict_replay_report.json`](diagnostics/camera_calibration_jpeg_20261017T054945Z/strict_replay_report.json)
and [`after_fix_ui_report.json`](diagnostics/camera_calibration_jpeg_20261017T054945Z/after_fix_ui_report.json).

The subsequent capture-stopped investigation found that historical JPEG counts
were misleadingly attached to later delivery outages. The SDK also discards
incomplete frames upstream, so its warning log is now retained. MJPEG failed
through both SDK backends and direct RGB capture; changing backend or requesting
30 FPS did not resolve it. Uncompressed 10 FPS still had recurring gaps and was
**not** accepted as the default. No camera firmware, USB settings, robot models,
existing captures or playback code were changed.

With the final **native YUYV / 1280×720 / requested 15 FPS** default, the actual
production Tk preview ran continuously for **120.03 seconds**. It received
**1,018 usable frames**, displayed 1,016, and had **no application error, capture
pause or terminal/bridge stderr output**. The largest received-frame gap was
0.201 seconds. The board was visible with 15/15 corners and 0.38-pixel reprojection
RMS at the end. No observer was constructed, CAN socket creation was blocked,
and the camera was released afterward. The active factory intrinsics, distortion,
depth-to-color transform, resolution and requested rate match the earlier profile.

**This is not a fault-free USB-link claim:** the SDK still logged and discarded
incomplete USB frames. The measured usable rate was about **8.5 FPS** on the
current USB 2 link. These incomplete images never reached the detector; the
stationary-window checks still apply. A known-good direct USB 3 connection remains
worth testing, especially before simultaneous RGB/depth work. Live camera testing
does not verify a complete physical calibration or authorize arm motion.

Evidence: [`live_idle_report.json`](diagnostics/camera_calibration_timeout_20261017T060620Z/final_native_idle/live_idle_report.json),
[`profile_comparison.json`](diagnostics/camera_calibration_timeout_20261017T060620Z/final_native_idle/profile_comparison.json),
and the session's `orbbec_sdk_logs/` alongside the saved frame-gap samples.

Reference APIs: [OpenCV hand-eye calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
and [Orbbec SDK application guide](https://orbbec.github.io/docs/OrbbecSDKv2_API_User_Guide/source/3_Application_Guide/ApplicationGuide.html).
