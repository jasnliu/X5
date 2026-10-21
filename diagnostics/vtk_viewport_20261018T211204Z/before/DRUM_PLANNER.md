# Gemini / MoveIt drum trajectory planner

**Default camera and preview commands never actuate motors. `live` automatically
queries current joints and captures a real depth planning scene. Only explicitly
armed `live --execute --commissioning PATH`, followed by a typed confirmation,
can command the right arm. The left remains query-only. Physical commissioning
is NOT complete; the execution adapter has only been tested offline.**
It does not change `start_beat.sh`, recordings, calibration captures, shared URDF,
encoder conversion, centering or existing playback programs.

**Current collision policy: detected obstacles only.** New `live` and `plan`
runs check the whole arm, custom gripper and drumstick against the captured
Gemini obstacle voxels. A shoulder/wrist outside the camera view is **not** an
automatic rejection. Unobserved space is assumed available for planning, not
verified empty. Joint limits, self-collision, detected-obstacle collision checks,
clearance buffers and existing execution/fault handling remain in place.

## Commands

Run from X5, or use the launcher's absolute path:

```bash
./plan_drums.sh build                 # compile only; already built here
./plan_drums.sh                       # live 2D + interactive 3D depth; no CAN
# S saves a snapshot. Q/Esc saves and closes; never changes motor state.

./plan_drums.sh camera --view 2d      # legacy 2D-only window, if desired
./plan_drums.sh camera --seconds 65 --headless
./plan_drums.sh demo                  # real MoveIt, fictional empty world/pose
./plan_drums.sh preview planner_runs/<demo-run>/plan.json
./plan_drums.sh simulate planner_runs/<demo-run>/plan.json
./plan_drums.sh test                  # offline math + real MoveIt tests
```

Camera capture prints the resulting `scene.json` path. Each command uses a new
output directory; `--output PATH` selects a **new** directory explicitly.
`preview --png picture.png` renders offline without opening a window or camera.
Interactive `preview` captures one initial depth frame, stops the camera, and
shows a frozen surface mesh with trajectory Play/Pause and a timeline.
These controls affect only the display, never the robot.
Use `--no-camera` for offline playback. No RViz/controller launch is required.

The robot overlay uses the **URDF visual model**, with the same body/arm DAE
assets and material colors used by `start_beat.sh`'s RViz, plus the planner's
custom right-gripper STL halves and a solid, correctly sized tan drumstick.
It is opaque, not a scattered sample of collision triangles or a skeleton.
Body, arm-link and custom-gripper faces are retained in full. For display speed,
only the very dense original hand/finger visuals use cached topology-preserving
surface simplification, with material boundaries retained; faces are never
simply skipped. This optional optimization uses the already installed Python
VTK package; without VTK all original visual faces are drawn instead.
Camera surface meshing is separate. These rendering changes do not modify
URDFs, calibration, collision geometry, planning, recordings or motor commands.
Existing saved plans use the corrected display when reopened; no replan needed.

### Automatic planning from the current arm and camera

With power available, **both arms already disabled/relaxed and stationary**,
and all other controllers/recorders stopped:

```bash
./plan_drums.sh live
# Calculate/save without opening the preview:
./plan_drums.sh live --headless
# Reopen the EXACT planning scene and its trajectory, without any device access:
./plan_drums.sh planned-view planner_runs/<current-plan-run>/plan/plan.json
```

This is now a complete **measure → capture → MoveIt → preview** pipeline; no
hand-written state/obstacle files are needed. It uses the existing encoder
convention, original record3 **final pose only**, calibrated isolated URDF,
custom right claw and full drumstick collision geometry. The left measured pose
is a static robot obstacle. The original center and the recorded route are NOT
used. A successful plan is outward → 2-second hold → reversed route to the saved
start. Playback is still only simulation unless explicitly armed below.

Acquisition needs a short stationary window (normally a few seconds, including
camera startup), **not a one-minute test**. Unlike the display-only one-frame
mesh, collision planning retains **five full-resolution 1280×800 native depth
frames** (~0.4 seconds at 10 FPS), nearest valid returns, and conservative
free-space evidence. Every frame is bracketed by stationary encoder batches.
CAN ownership/feedback continues through planning to detect movement or lost
feedback; then both devices close before rendering. The mesh stays frozen and
represents these same planning frames, not a newly captured unrelated scene.

There is no fallback to fictional start angles, recorded motion or manually drawn
obstacles. The normal `observed_obstacles` policy does not require camera-proven
free space around every link. Missing, occluded, too-near and out-of-FOV regions
do not themselves block a route; obstacles absent from the map are not accounted
for. Detected collisions (including a detected table), self-collisions and joint
limit failures still block the route. A blocked result shows its actual reason
and disables Play. No geometry is removed merely to make a route pass.

Each new `planner_runs/*-current-plan-*` folder contains `measured_start.json`,
CAN TX/RX and encoder batch audits, `camera/scene.json` with raw five-frame depth,
`plan/plan.json`, MoveIt requests/results/logs and `current_plan_report.json`.
The original single-frame `preview ...` and plain `./plan_drums.sh` behavior is
unchanged. Use `planned-view` for the *matching planning* mesh.

### Separately armed physical execution (commissioning required)

```bash
# NOT ready to run until the physical review below is actually completed:
./plan_drums.sh live --execute --commissioning /path/to/verified-commissioning.json
```

This command does a NEW acquisition/plan, not execution of `software_demo` or an
arbitrarily saved file. It requires an attended terminal, a hash-bound physical
review, an extra exact-polynomial detected-obstacle/self-clearance check, and typing
`MOVE RIGHT ARM`. It rechecks current encoders immediately before enabling. No
long soak or automatic centering is performed. The camera scene must be at most
30 seconds old at enable; taking too long to confirm requires a new run.

This is **supervised static-scene execution**, not dynamic obstacle avoidance.
The camera is released after capture, so the surrounding kit, people and objects
must stay out of the swept volume for the complete cycle. No real-time camera
monitoring or automatic collision detection is claimed. Moving environments
need an additional live safety layer, not a checkbox override.

The commissioning JSON must use `schema: x5-drum-planner-commissioning-v1`,
matching `calibration_sha256`, `urdf_sha256`, `srdf_sha256`, `recording_sha256`,
seven measured `supported_right_rad`, and detailed `verification_notes`.
These fields must represent **real physical checks**, not merely be set true:

- `depth_alignment_verified`: actual depth-to-robot alignment checked.
- `right_tool_mount_verified`: CAD mount/yaw and stick length/axis checked.
- `left_attachment_clearance_reviewed`: account for the physical left stick;
  it is not yet modeled as an attached link. Camera self-filtering must not erase
  its observed points near the left hand.
- `supported_rest_verified`: this exact start is mechanically supported, so
  removing torque at that pose cannot cause a drop. Stationarity alone is NOT
  evidence of support. Colliding table contact is still not exempted.
- `static_scene_access_controlled` and `emergency_stop_available`.

No completed review is supplied or fabricated by this implementation. Hashes
can be obtained from the generated model and actual calibration/recording using
`sha256sum`; any geometry/calibration/goal change invalidates the review.

The right-only adapter reuses existing X5 CSP packets, encoder conversion,
exclusive locks, current limits and feedback decoding. It preloads the measured
start before enabling; it never issues startup disable/retry commands. Gripper
targets are **only customized CLOSED +7°**, never open. The gripper must already
be within 1° of closed; the program refuses to force a displaced mechanism.
Targets follow the exact checked quintic at 100 Hz; no extra PID correction,
gain tuning, safe-zone transition or center movement is added.

Execution checks use ≥50 mm world padding and ≥40 mm non-exempt self clearance,
plus the existing sampling/voxel margins. Total seven-joint tracking error is
limited to 0.01 rad (sum, not per-joint), feedback age to 120 ms, scheduling gaps
to 80 ms, and left movement to 0.35°. These conservative limits may reject a
route that was acceptable as a visualization. Do not weaken them to make a
physical test pass; tracking/geometry need actual commissioning.
Execution rechecks use the **same `observed_obstacles` policy** as the new plan;
they do not reintroduce the shoulder/FOV veto. The preceding extra execution
clearance and fault thresholds have not been changed by the policy update.

At the goal, the arm settles and holds 2 seconds, then retraces the exact route
with a smooth scalar time warp. A cancelled outward movement retraces only its
executed prefix. At the saved start it must settle within 0.2° with ≤0.12° range
for 0.6 seconds before the pre-verified supported-rest release (including the
gripper). Left-arm enable/configure/disable commands are blocked at the wire.

**Fault handling:** Ctrl-C/SIGTERM requests a checked return, not a drop. Tracking
deviation/stall, stopped motors, CAN/feedback faults or left-arm changes make a
return unsafe: no further path, center or relax command is issued. The last
motor-native CSP setpoint remains; bus ownership stays held while the operator
supports the arm. Support it/use the physical stop as needed, then type
`SUPPORTED EXIT` to release software ownership **without a relax command**.
No software can promise holding torque during power loss, drive faults or a
killed process. Do not use kill -9 as a normal stop.

Logs: `plan/execution_clearance_*`, `plan/execution_can_audit.jsonl`,
`plan/execution_events.jsonl`, and `plan/execution_report.json`. Success in
offline tests does NOT prove actual motor tracking, safe mounting or support.

### Read-only hardware integration: measured pose and alignment

This is the first hardware stage, **not execution**. With the robot powered on
but both arms already disabled/relaxed, and **all other robot controllers,
recorders and calibration sessions stopped**, run:

```bash
./plan_drums.sh hardware-check
```

The program does **not** relax an enabled arm to make this check work. If any
motor reports enabled or faulting, or feedback is missing, it stops without
sending enable/disable, motion, gripper, zeroing or configuration commands.
Do not switch off support or run another controller to work around a failure.
It does not bring up/reconfigure CAN interfaces or ask for sudo.

What this does:

1. Acquires the existing exclusive X5 locks for `can0` (right) and `can1` (left).
2. Queries motors 1–8 on **both** arms using the existing state request and
   `encoder_to_joint` convention. Every outgoing packet passes an exact-byte
   query-only allow-list; requests, responses and complete batches are logged.
3. Observes the stationary resting pose (default about three seconds), captures
   one full-FOV depth frame, and requires disabled-motor batches before and after
   that frame. All 14 joint angles must stay within a 0.35° spread during capture;
   batch gaps over 250 ms, batches over 200 ms, missing camera bracketing, or
   nearest-batch midpoint skew over 100 ms are rejected. This is stationary
   host-timestamp synchronization, **not hardware-triggered synchronization**.
4. Saves actual measured joint angles, then closes both CAN sockets and releases
   the camera before generating/displaying the static mesh and alignment view.
   Closing never sends a disable/relax command.
5. Displays the calibrated model at those **captured** angles, not a fictional
   starting pose or a trajectory. The model remains frozen even if the physical
   arm later changes position; restart the check to measure again.

The 2D depth image includes a **predicted FK skeleton/stick projection**, not
camera-detected features. Toggle **Show encoder-derived robot / projected
skeleton** to inspect the original depth underneath. The same surface/points/
both, color and rotate/pan/zoom controls remain available in 3D. There is **no
Play or Execute action** in this view. S saves `preview_view.png`; Q/Esc closes.

Compare visible robot/tool surfaces with the prediction. `coverage.json`
reports whether hand/stick landmarks project inside the depth image and whether
those pixels contain depth. **In-image does not establish visibility, alignment
or clearance**; joint centers are not observed surface points. Occluded/outside-
FOV regions, table contact/support, tool mounting and the left stick remain
unverified. A passing acquisition report never means the robot is motion-ready.

Other commands:

```bash
# Longer encoder-only diagnostic; no camera or GUI, no motor-state changes:
./plan_drums.sh hardware-check --headless --no-camera --query-seconds 65

# Save synchronized measurements/mesh without opening a window:
./plan_drums.sh hardware-check --headless

# Timed read-only capture + 65-second frozen alignment window:
./plan_drums.sh hardware-check --seconds 65

# Reopen an existing snapshot OFFLINE: no CAN, no camera, no motor power needed:
./plan_drums.sh hardware-view planner_runs/<readonly-hardware-run>
```

Use `--output NEW_DIRECTORY` to choose the output; existing directories are
never overwritten. `--query-seconds` controls acquisition time (1–600 seconds),
whereas `--seconds` controls only the frozen GUI duration. No CAN queries or
camera frames continue in that window. Query-only access is still CAN traffic,
not passive listening; the left arm receives state requests but no commands.

Each run saves `measured_start.json`, complete `encoder_batches.jsonl`,
`can_query_audit.jsonl` (attempt/sent records), `can_rx_audit.jsonl`, an isolated
calibrated `model/`, `snapshot_manifest.json` with hashes, and
`preflight_report.json`. Camera runs also save `camera/` with the original raw
frame and immutable mesh, plus `coverage.json`. Offline reopening validates
model, encoder-convention, calibration and saved-file hashes.

`measured_start.json` uses the planner's existing state schema. Its right
closed-gripper field is the **future configured +7° target**, not evidence that
it was commanded or reached; actual raw motor-8 feedback is saved separately.
The left jaw opening is a legacy encoder-derived display estimate, not a
physical width measurement. Measured angles are never clipped to joint limits;
out-of-limit values are reported for investigation. Both acquisition and future
planning remain non-executable: `execution_ready`, `motion_authorized` and
`hardware_execution_available` are false. No existing demo is authorized by
collecting a measured start, and the single-frame mesh is still not a planning
scene. A real collision-checked route, physical alignment/tool/support checks
and feedback-monitored execution/recovery remain subsequent stages.

### Live 3D camera window

The default camera command now opens **live 2D depth and a rotatable 3D point
cloud side by side**. The 3D view updates from new depth frames, not only the
first captured scene. Rotate with left-drag, pan with middle-drag, and zoom with
the mouse wheel or right-drag. **Reset / fit view (R)** restores the initial
orientation and fits the observed scene. Updating data does not reset your view.

- **Freeze / Resume (Space)** freezes both displayed views for inspection;
  camera acquisition, frame counters and stale-frame checks keep running.
- **Raw depth points** shows valid returns from the displayed single frame.
- **Obstacle voxels (10 mm)** shows the same nearest-return, five-frame union
  and voxel centers saved for the planner, **before robot self-filtering and
  collision padding**. These are occupied centers, not a buffered safety verdict.
- **Color** selects camera depth or calibrated base-frame height for the 3D
  cloud. The 2D image always uses camera distance, 0.15–5 m. No RGB stream is added.
- **Save LIVE scene (S)** always saves the latest full-resolution five-frame
  window, **even while the display is frozen**. Q, Esc, window close or a timed
  run's completion saves the latest scene and releases the camera. Saved scenes
  remain static snapshots; they do not become a live execution safety monitor.

Rendering is capped at four update requests per second and can be slower,
depending on processing time. One background display job is allowed at a time;
there is no frame-job backlog. At most 18,000 points/voxel centers are displayed,
but **all valid pixels are used to generate the voxel map and saved scene**.
The window reports acquired-frame count, 3D-update count and displayed-frame
age. This is a sparse surface view, not a watertight reconstruction: missing,
hidden and out-of-range regions remain UNKNOWN, not clear. The camera-only
command has no robot overlay; the trajectory `preview` command adds an explicitly
simulated robot from the saved plan, never an inferred/measured live arm pose.

The viewer uses the already-installed PyQt5 and Matplotlib packages. Headless
capture does not create a GUI. `capture_report.json` includes visualization
statistics (rendered sequences, update count, modes and freeze/resume count).

### Play a trajectory over a frozen depth surface mesh

```bash
# Requested preview: one initial camera scene, then static mesh + animation.
./plan_drums.sh preview planner_runs/software_demo/plan.json

# Existing camera-based candidate: occupied-only, NOT a verified safe route.
./plan_drums.sh preview planner_runs/camera_preview/plan.json

# Explicit offline saved-scene playback, with no camera connection:
./plan_drums.sh preview planner_runs/software_demo/plan.json --no-camera

# 65 seconds of simulated playback AFTER the camera has stopped:
./plan_drums.sh preview planner_runs/software_demo/plan.json --autoplay --loop --seconds 65
```

Close any other depth viewer first so this program can acquire the camera.
The preview captures **exactly the first complete, validated native depth frame**
(1280×800 Y16), then **stops and releases the SDK camera process before meshing**.
The 2D image and 3D scene never update again during that run. There is no
background camera stream, processing queue, periodic mesh rebuilding or resume
capture button. Restart the preview to capture a different scene.

The combined window shows the **simulated robot, custom gripper and red
drumstick on the same calibrated axes as the frozen surface mesh**, with the
original depth image alongside. No ROS publishers or robot drivers are started.

- **Surface mesh** (default), **Depth points**, and **Mesh + points** show different
  representations of that *same* initial frame. Color selects camera distance
  or calibrated world height. No RGB stream is acquired.
- **Play/Pause animation (P)** becomes available once the mesh is ready. Playback
  uses elapsed wall-clock time, retaining the saved two-second hold and return.
- Scrubbing the **timeline**, **Go to start**, or **Go to goal** pauses playback.
  Speed (0.5×/1×/2×) and Loop affect only animation. Playback starts paused unless
  `--autoplay` is supplied, and normally stops at the end. Pausing leaves the
  window idle: neither depth acquisition nor animation redraws keep running.
- **Fit arm + kit** focuses around the trajectory. **Reset / fit view (R)** shows
  the full observed extent and arm. Rotate/pan/zoom remain available. Geometry
  outside the chosen axes is clipped **for display only**, not deleted.
- The initial raw frame and mesh are saved automatically to a new
  `planner_runs/*-snapshot-mesh-*` folder (or `--output PATH`). **Save view (S)**
  saves `preview_view.png` without accessing the camera. **Q / Esc / window close**
  exits; no recapture is performed. Original plans and models remain read-only.

**How the surface is approximated:** neighboring depth pixels are connected into
triangles on an eight-pixel grid, including the last row and column to retain
full image extent. Every original pixel between the corners is checked: cells
with missing/out-of-range data or a depth span greater than `30 mm + 2% of depth`
are omitted. Triangles with an edge longer than 120 mm are also omitted.
No holes, separate objects, unseen backsides or watertight solids are invented.
This deliberately leaves gaps and may omit thin/noisy surfaces; **it is a
visual approximation, not collision geometry**. The original raw image is kept.

**This is visualization, not collision revalidation or replanning.** The saved
trajectory was checked against its original saved/synthetic scene, not against
this initial snapshot. Synthetic and occupied-only warnings remain visible.
The robot overlay is saved-plan joint angles, not live encoder measurements;
the camera may separately see the real stationary arm. Tool mounting and depth
alignment remain physically unverified. Missing/busy/stale camera data prevents
startup instead of silently substituting an old scene. After intentional camera
shutdown, the static scene is not treated as a failing/stale live feed.

Files: `raw_depth.npz` holds one full-resolution frame and its SDK unit scale;
`surface_mesh.npz` holds base-frame vertices, triangle indices and source pixel
coordinates; `scene.json` records calibration, camera, filters and file hashes;
`capture_report.json` records camera shutdown, frame/mesh counts and animation
statistics. **This new `x5-depth-surface-preview-v1` scene is NOT a planning
input.** Use `./plan_drums.sh camera` to collect the existing five-frame native
planning scene. Its conservative voxel/free-space processing is unchanged.

PNG export is always offline. `--no-camera` uses the original saved scene and
does not connect to the camera. Neither mode can send motor commands.

A completed empty-world software example is already available:

```bash
./plan_drums.sh preview planner_runs/software_demo/plan.json
```

To plan against the saved camera scene:

```bash
./plan_drums.sh plan \
  --scene planner_runs/<depth-run>/scene.json \
  --state drum_planner/config/example_start.json
```

**The supplied state is ONLY an example**, using the first recorded `record3`
pose and a synthetic left-arm pose. It is **not the current resting position**.
With motors unpowered, their present encoder angles cannot be read. No pose was
invented and labelled as a current measurement. For an accurate later preview,
supply a saved state with this schema and both arms' actual angles:

```json
{
  "schema": "x5-planner-start-state-v1",
  "coordinate_convention": "encoder_to_joint",
  "right_rad": [0, 0, 0, 0, 0, 0, 0],
  "left_rad": [0, 0, 0, 0, 0, 0, 0],
  "left_gripper_opening_m": 0,
  "right_gripper_closed_motor_joint_rad": 0.12217304763960307,
  "preview_only": true,
  "source": "REPLACE these example angles with a measured snapshot"
}
```

Angles use existing `encoder_to_joint` coordinates, not raw motor positions.
The right gripper's +7° CLOSED convention is fixed, and its collision-model
finger joints are fixed: there is no opening trajectory. The left arm is a
static collision model; it receives no commands. No centering step is inserted.

## Intended cycle and fault policy

1. Begin at the supplied resting-state angles.
2. Plan to **exactly the final seven angles of `recordings/record3.json`**. The
   recorded route is not replayed; only its endpoint is used.
3. Hold the endpoint for **2 seconds**, without striking.
4. Reverse the checked outward geometric path back to the saved start.
5. Produce a **supported-rest/relax intent**, not an actual torque command.

Reversing is verified only against the **same static scene**. Changed obstacles,
tracking errors or a changed actual pose require rechecking/replanning before
any future recovery motion. The pure-software recovery policy permits a return
only when feedback and motion remain available and a return path is checked.
Stall, missing feedback, unsafe return or unverified support produce a
stop/hold/operator-required intent, **never unconditional relaxation**. Relaxation
is allowed only at the saved start, stationary and supported. No centering
fallback exists, and no code in this package can presently hold or relax motors.

## Architecture

- `drum_planner/cpp/depth_capture.cpp`: separate Orbbec SDK camera-only process.
  Native **1280×800 Y16 at 10 FPS**, maximum Gemini 2 depth image/FOV, no color
  alignment/crop, no automatic lower-FOV fallback. RGB is not streamed
  simultaneously, avoiding unnecessary USB bandwidth. Camera properties and
  firmware are not changed. The existing RGB calibration utility is untouched.
- `depth.py`: exact-length frames, SDK-provided depth-unit scale, frame-sequence
  and timestamp checks, a one-second delivery-staleness stop, five-frame bounded
  history, saved raw depth and diagnostics. Camera serial, profile, calibration
  parameters, work mode and factory depth-to-color transform must match the
  completed calibration. A five-frame nearest-valid union retains thin returns;
  free-space evidence requires a valid return in **all five frames**.
- `model.py`: isolated calibrated URDF/SRDF. Joint corrections are embedded in
  joint origins, making FK equivalent to `FK_original(q_saved + offsets)` while
  retaining recorded coordinate values and original command limits. No double
  application of the pre-existing J5 convention. The original URDF is untouched.
- `cpp/moveit_plan.cpp`: genuine MoveIt `PlanningScene`/FCL whole-body collision
  checking plus OMPL `RRTConnect` in seven-dimensional joint space. No broad
  vendor “Never” collision exclusions; only adjacent/rigid assembly exemptions.
  No custom TCP safe-zone constraint. MoveIt creates a logging-only ROS node in
  **domain 197, localhost discovery**, separate from X5's domain 92. There are
  no joint-command publishers, controllers, driver launches or CAN sockets.
- `planning.py`: MoveIt TOTG supplies an initial timed path. The final route uses
  X5's established endpoint-exact quintic-spline approach, with analytic peak
  velocity, acceleration and jerk checks. The original recording path is not
  used. Default bounds: **0.6 rad/s, 1.2 rad/s², 6 rad/s³**, 100 Hz preview.
  The continuous spline is sampled at ≤2 ms and each chord subdivided to ≤0.003
  rad summed joint travel for additional MoveIt collision checks. If the smooth
  spline cuts a corner into an obstacle, a C2, path-preserving quintic fallback
  stops smoothly at the few geometric path corners rather than accepting a
  collision. Failed checks remain failures, not silently relaxed tolerances.
- `workflow.py`: offline cycle/recovery verification; **not a motor controller**.
- `preview.py`, `trajectory_view.py`, `live_playback.py`: shared saved-plan robot
  renderer and wall-clock playback; explicit offline mode/PNG export.
- `snapshot_preview.py`, `surface_mesh.py`: default trajectory preview captures
  one initial frame using `DepthCamera(single_frame=True)`, releases the camera,
  and builds an immutable, discontinuity-filtered triangle surface once. No ROS
  publishing, motor API or path revalidation. The camera-only command stays live.
- `live_view.py`: camera-only Qt/Matplotlib live 2D/3D viewer, bounded one-job
  background processing, freeze/resume and raw/voxel inspection. Full-resolution
  voxel generation uses integer keys instead of a GIL-blocking structured sort;
  it produces identical voxel centers without starving the camera reader.

### Custom tool geometry: what is known and what is provisional

The two STL files are loaded in **millimetres**, with 0.001 scaling. Their upper
semicircle edges numerically match an inner radius of **7.4 mm** and outer radius
12.4 mm. The assembly aligns the bore centers at CAD coordinates
`(-14.75, -23.35)` and `(+14.75, -23.35)` mm so the halves form one continuous
cylinder. Both custom halves replace the stock right finger collision meshes.

The stick is a collision cylinder of **14.8 mm diameter**, extending along the
hand's outward Z axis: **130 mm inside the gripper region + 279.4 mm exposed**,
409.4 mm total from the assumed mounting plane, with no rear stick projection.
It participates in robot/world and nonadjacent self-collision checks—not merely
as a TCP point. A regression test proves an obstacle hit by only the stick is
rejected and would incorrectly pass if the stick collision body were removed.

**The exact STL-to-hardware mounting transform is not proven by the STL alone.**
The current generated model uses the nominal finger attachment plane at hand
Z=15 mm and a 90° CAD yaw as a **provisional placement**. The bore assembly is
verified mathematically; the mounting plane/yaw must still be checked against
the actual shared mounting face before motion. It is not claimed as an automatic
physical registration. The static left arm's actual pose and stick geometry
also need verification. These limitations remain explicit in every plan.

## Camera obstacles and unseen space

Real scenes contain **only Gemini-derived returns**, not manually drawn drum,
cymbal or table primitives. Every valid pixel is deprojected with the matching
native-depth intrinsics and transformed using `T_base_depth`. Voxelization is
10 mm; there is no point stride that simply skips thin objects. The world is a
MoveIt OctoMap. Default robot/world padding is 20 mm plus 9 mm voxel uncertainty
and 5 mm finite edge-check allowance. Buffer failure at the goal is a real
rejection, not a reason to remove the cymbal or shrink the buffer automatically.

Points inside the supplied initial robot geometry (+5 mm self-filter padding)
are removed from the obstacle cloud; other points remain. This requires an
accurate start model and is not proof that a table-contact start is safe.

New plans use `collision_policy: observed_obstacles`: FCL checks all moving
collision bodies against the retained occupied voxels, as well as self-collisions.
The additional requirement to fit every body/clearance volume inside observed
free depth is **off**. Near-camera geometry, a missing pixel, or an out-of-view
starting arm is not itself an obstacle. The robot model and encoders provide arm
position; the camera provides observed environment obstacles. Unseen space is
not proven empty, and thin/hidden objects missed by the sensor remain a limitation.

The free-depth image is still saved for diagnostics and historical replay. Native
backend requests without a `collision_policy` retain their original interpretation;
the explicit `observed_free_space` policy retains the former strict test. Neither
is selected by normal new `live`/`plan` commands. Invalid policy names are rejected.

The old `--occupied-only-preview` flag remains a **legacy preview-only flag** for
compatibility. It is no longer needed to obtain an observed-obstacle plan. Old
preview artifacts are not silently upgraded into execution candidates. A fresh
measured new-policy plan still needs the separately armed execution workflow.

## Outputs

Depth runs save `scene.json`, `raw_depth.npz`, the conservative depth images,
`points_base.f32`, `depth_preview.png`, `capture_report.json` and SDK logs.
Planning runs save the generated model, exact request, backend logs/results,
interpolated-path rechecks and **`plan.json`**. Saved-file hashes protect model,
calibration, recording and scene compatibility. Always inspect `status`, not
just whether a file exists. Every result has `motion_authorized: false` and
`hardware_execution_available: false`.

Read-only current-state capture remains available via `hardware-check`, while
`live` now connects acquisition to planning. Saved plans remain non-authorizing
preview artifacts; the separate `live --execute` adapter requires commissioning,
fresh acquisition, additional clearance checks and attended confirmation. There
is still **no live obstacle watchdog**: this is static-scene supervised execution.
A static camera snapshot is not a live collision safety system.

## Verification history on this computer

The following entries record earlier behavior and test evidence. In particular,
strict unknown-space refusals below predate the current observed-obstacle default;
they are not the current default policy. See the latest verification entry at the end.

Evidence directory: `diagnostics/drum_planner_20261018T053703Z/`.

- Real Gemini depth ran **65.88 seconds**, delivering **661 complete 1280×800
  frames** with no application error. Largest received-frame gap: 0.416 s.
  Terminal bridge stderr was empty. SDK upstream USB warnings remain diagnostic
  evidence; this is not a claim of a perfect USB transport.
- Camera test made **zero CAN socket attempts** and both CAN buses' packet
  counters were unchanged. No motor commands or queries were sent.
- **31 offline tests** pass, including real MoveIt/FCL collision checking and
  OMPL planning, calibrated FK equivalence, gripper bore/stick geometry, depth
  scale/holes/profile rejection, interpolation limits, exact endpoint/return,
  two-second hold, fault policy, and hardware-execution rejection.
- A synthetic box intersecting **only the stick's direct sweep** invalidated
  straight interpolation. MoveIt found a detour, and the smoothed route passed
  rechecking. The empty-world example took about 5.30 s outward; the demonstrated
  detour about 5.48 s, excluding the two-second hold and return. These are software
  timings, not measured robot movement.
- Against the **actual captured kit**, strict checking rejected the example
  start with `unknown_or_occluded_space` at `openarmx_right_link1`. This is an
  expected safety rejection, not a claim that the current resting pose was
  measured. The explicitly occupied-only preview produced a non-executable
  candidate; it does not establish that the unseen starting region is clear.
- The final **regular camera-window CLI**, not only the headless helper, also
  ran for **65.84 seconds / 661 frames**, with no application error and empty
  bridge stderr. The largest received gap was 0.478 s. The previous calibration
  utility's **110 regression tests** still pass. All 131 protected source,
  recording, CAD and calibration files remain byte-for-byte unchanged.

References: [MoveIt planning scenes](https://moveit.picknik.ai/main/doc/examples/planning_scene/planning_scene_tutorial.html),
[OMPL collision-check discretization](https://moveit.github.io/moveit_tutorials/doc/ompl_interface/ompl_interface_tutorial.html),
[Gemini 2 native depth/FOV specifications](https://www.orbbec.com/wp-content/uploads/2023/04/ORBBEC_Datasheet_Gemini-2.pdf),
and [MoveIt 2.12.4 logger behavior](https://github.com/moveit/moveit2/blob/2.12.4/moveit_core/utils/src/logger.cpp).

### Live 3D viewer verification

Evidence: `diagnostics/live_depth_viewer_20261018T061617Z/`.

- **46 offline tests** passed (31 planner regressions + 15 live-view tests),
  plus the unchanged calibration utility's **110 tests**. New tests cover unit
  conversion/deprojection, missing depth, display-only sampling, identical voxel
  output, no display-job backlog, error propagation, freeze/resume, raw/voxel and
  color controls, zoom/view persistence, keyboard shortcuts, saving/closing,
  headless compatibility, and stale-camera failure even with a frozen view.
- The initial live run exposed a real issue: NumPy's structured voxel sort held
  the Python GIL long enough to trip the unchanged camera freshness check. An
  equivalent integer-key sort fixed this, with **byte-identical saved voxel
  coordinates** on the prior real scene. The before/after benchmark is retained
  in `voxel_performance_parity.json`; no timing guard was weakened.
- The final regular camera-window CLI run passed **65.40 seconds / 656 depth
  frames / 206 live 3D updates**. A live control-exercise run passed **65.51
  seconds / 657 frames / 172 updates**, including a ten-second display freeze.
  It verified that camera frames continued during freeze, saving used newer live
  frames, resuming caught up without a backlog, and rotate/pan/zoom, height
  coloring, and both layers worked. Screenshots and interaction assertions are
  saved alongside capture reports. These are functional UI/camera checks, not
  physical depth-accuracy or motion-safety certification. SDK startup/property
  and rejected upstream USB-frame warnings remain in the logs; these runs had
  no application failure or bridge-stderr output, not a warning-free SDK.
- No motor commands or CAN queries were sent. The live control test rejected
  any attempted Python CAN socket, recorded zero attempts, and both CAN buses'
  packet counters stayed unchanged. The 131 protected original files remained
  byte-for-byte unchanged.

### Combined trajectory + live depth verification

Evidence: `diagnostics/live_trajectory_preview_20261018T153836Z/`.

- **62 planner/viewer tests** and **110 calibration regressions** passed. The
  added checks cover exact goal/return/hold playback, wall-clock speed and loops,
  calibration/model mismatch rejection before camera access, invalid/blocked
  plans, robot/stick overlay transforms, independent freeze/pause, timeline and
  keyboard controls, display-only cropping, delayed autoplay, live/offline CLI
  routing and camera-free PNG rendering.
- Production CLI with the saved camera-based candidate, `--autoplay --loop`,
  ran **65.44 s**, receiving **657 depth frames**, making **183 depth-display
  updates and 395 animation draws**, and completing **two animation cycles**.
  Outward, two-second hold and return phases were displayed.
- A second **65.40 s / 656-frame** actual-camera run exercised the normal
  paused-start GUI, Play/Pause, depth Freeze/Resume independently of animation,
  exact-goal scrubbing, raw/voxel layers, height coloring, rotation, save, speed
  and looping. Its assertions, screenshots and reports are retained. Both runs
  had no application failure and empty bridge stderr; SDK startup/USB warnings
  remain in their diagnostic logs rather than being claimed absent.
- All **157 protected files** (original source/data plus existing plans/models
  and planner core) stayed byte-for-byte unchanged. Both CAN buses' transmit and
  receive counters remained unchanged; the interaction test also guarded Python
  CAN socket creation and recorded zero attempts. These tests do not establish
  physical collision safety, current encoder pose or depth-registration accuracy.

### Frozen surface-mesh playback verification (current preview behavior)

Evidence: `diagnostics/snapshot_mesh_20261018T155821Z/`.

- **80 planner/preview tests** and **110 calibration regression tests** passed.
  New tests cover organized-grid triangulation, units/calibrated transforms,
  original-pixel holes and depth boundaries, long-edge rejection, full image
  extent, immutable arrays, one-frame reader termination, clean-shutdown/error
  handling, static GUI controls, and rejection of this mesh as a planning scene.
- The exact `preview planner_runs/software_demo/plan.json` command, with timed
  autoplay/loop flags, ran **65.08 seconds after the mesh was ready**, completing
  **five simulated cycles**. It acquired **one frame**, built **one mesh**, and
  submitted **zero background depth jobs**. The camera bridge exited normally
  (**exit code 0**) before meshing; no native camera process remained running.
- A second actual-camera/control test ran **65.05 seconds**. It verified all
  three layers, height coloring, reset/focus/rotate/zoom, keyboard Play, timeline
  seeking, speed/loop, saving a view, and disabled camera resumption. Mesh/image
  arrays and saved scene files were byte-identical after 60 seconds. Five paused
  seconds generated no animation redraws. Screenshots were visually checked.
- Both completed without application errors. SDK startup/property and rejected
  upstream USB-frame warnings are retained in the logs; this is **not** a claim
  of a warning-free camera transport or measured collision accuracy.
- The separate continuous camera command still ran normally: a short regression
  captured **36 frames over 3.43 seconds** and saved its usual five-frame scene.
- No motor commands or CAN queries were sent. The control test blocked CAN socket
  creation and recorded zero attempts; both buses' packet counters were unchanged.
  **157 protected files** (including recordings, calibration, original robot
  model and saved plans) remained byte-for-byte identical. The one intentional
  camera-bridge source change adds optional `--single-frame` shutdown; its default
  streaming mode is preserved. `preservation_audit.json` records this distinction.

### Read-only hardware-stage verification

Evidence: `diagnostics/readonly_hardware_20261018T161731Z/`.

- **110 planner/preview/read-only tests** and **110 calibration regression tests**
  passed. Added tests exercise the exact state-query bytes, rejection of every
  actuator command type, disabled/fault/temperature checks, exclusive ownership,
  short writes, no disable-on-close, telemetry thread cleanup, stationary camera
  bracketing, measured-state provenance/hashes, failure cleanup, projection and
  the non-animated alignment UI. All CAN devices in these tests are mocked.
- The static alignment GUI completed a **65-second offline soak** with explicitly
  **mock encoders and synthetic depth**, exercising layers, overlay visibility,
  color, fit/reset, saving, and the unavailable playback action. The scene arrays
  remained unchanged, idle display did not redraw, and no camera/CAN access was
  allowed. This is UI evidence, **not live encoder or physical alignment proof**.
- All **166 protected existing files** remained unchanged, including recordings,
  calibration, original encoder/controller code, camera bridge, saved plans and
  the existing mesh-preview implementation. Host CAN transmit counters remained
  unchanged; no real motor-state queries or motor commands were sent in this
  implementation turn.
- **Live integrated validation is pending confirmation that the robot is powered
  on, both arms are already disabled/relaxed, and other controllers are stopped.**
  This implementation does not turn a software pass into hardware-motion approval.

### Current-pose planning and guarded execution verification

Evidence: `diagnostics/current_pose_planning_20261018T190938Z/`.

- **148 planner/preview/acquisition/execution tests** and **110 calibration
  regression tests** passed. The final changed-module rerun passed **38 tests**.
- New coverage includes five-frame encoder bracketing, preserved measured start,
  exact record3 goal, real MoveIt unknown-space and minimum-self-clearance
  rejection, post-planning drift/stale feedback invalidation, scene/model hashes,
  exact quintic reconstruction and extra execution clearance checks.
- Simulated complete and cancelled cycles returned to the saved start before
  release; injected tracking/stall/CAN/left-movement/stopped-motor failures did
  not relax. A byte-level emulated CAN test confirmed left query-only traffic,
  no startup disable, exact +7° closed-only targets and no disable-on-close.
- The **3-second offline GUI check** used explicitly synthetic depth/mock
  encoders. The real MoveIt backend rejected that start; the UI kept Play
  disabled, displayed the reason, and rendered the same frozen planning mesh.
  Its saved screenshot was inspected. It is not evidence of physical alignment.
- The requested one-minute test was **skipped**. No real camera/CAN acquisition,
  motor enabling, motion or physical tracking test was performed for this change.
- Of **169 previously hashed existing files**, **163 were unchanged**; the six
  intentional source edits are listed in `preservation_audit.json`. Original
  recordings, calibration data, shared URDF, encoder and controller sources were
  preserved. New modules and documentation are hashed separately.

### Static-left encoder boundary and error-preview correction

A non-moving left joint may read up to **three encoder counts**
(`3 × 25.14 / 65535 rad`, approximately **0.06594°**) beyond its nominal angular
URDF boundary. This is a measurement-only allowance, reused from X5's existing
encoder-resolution tolerance. It is **not** a wider motion limit: right-arm
starts/goals/paths remain strictly bounded, as do finger variables. Larger left
overshoots remain blocked. The actual left angles are retained for FK and all
collision checks; nothing is clipped, re-zeroed, commanded or written to the
shared URDF. `joint_limit_check` records every accepted tiny static overshoot;
rejections now identify the specific joint, measurement and limits.

Every backend exception now includes the result schema. Historical error-only
reports missing that schema can also be displayed read-only, without changing
the saved file or admitting a trajectory. Play remains disabled for all errors
and blocked plans; the UI shows the original cause instead of a second schema
error. Unknown-space reports distinguish the near-camera exclusion, outside-FOV
coverage, missing depth and a surface occluding the required clearance volume.

Verification: `diagnostics/planner_start_limit_fix_20261018T193339Z/`.
**163 offline planner tests passed**, including real MoveIt planning/smoothing
with the exact reported left J4 value, stricter right/finger bounds, larger-left-
overshoot rejection, structured exceptions and legacy-error preview handling.
The user's saved run `20261018T193113Z-current-plan-78618c` was replayed offline;
its left J4 is −0.032969°, and all right joints are within the original limits.
The two reported software errors are resolved. The replay still correctly blocks
`openarmx_right_link1`: part of its conservative body/clearance volume reaches
camera-axis depth **0.08833 m**, inside the **0.15 m** accepted near-depth limit.
This does not prove an obstacle is there, but the camera cannot certify it free.
That safety check was not bypassed, and no safe trajectory is claimed.

Both the original error and the replanned blocked result rendered against the
actual saved depth in brief offline GUI checks, with Play disabled. No fresh
camera/CAN access, motor commands or one-minute test was performed. Original
captures, recordings, calibration, shared model and controller code were preserved.

### Whole-arm camera coverage diagnosis

`blocked_start` is a **planning safety refusal**, not necessarily a connection or
programming error. Planning still returns a nonzero exit status and disables Play
when it cannot establish the required clearance. It does not fall back to an
occupied-only route, crop the arm, erase obstacles, or assume unseen space is free.

Blocked real-scene plans now include `visibility_coverage.start` and `.goal` in
`plan.json`. They inspect **all 11 moving collision links**, not just the first
failed shoulder probe. Each link reports probe/failure counts, examples, and
nominal mesh/cylinder camera-axis depth extrema **without a clearance buffer**.
These are diagnostic model projections, not new physical measurements or a
collision clearance certificate. The original collision/visibility verdict and
all safety margins are unchanged. Terminal and frozen-view messages summarize
both endpoint coverage reports.

For saved run `20261018T195224Z-current-plan-17b383`, the nominal shoulder mesh
reaches camera-axis depth **0.08183 m**, already inside the program's **0.15 m**
accepted-depth cutoff; its first buffered probe reaches **0.08833 m**. Independent
vertex projection also places the resting links 2–7 and hand wholly below the
depth image. This is not only an oversized-buffer effect. Fresh read-only capture
reproduced the refusal, with some required clearance unverified for every moving
link at both endpoints. A different planner cannot infer these missing surfaces.
Camera coverage must be improved and physically checked; moving the camera also
requires recalibration. A new view alone is not a guarantee of complete clearance,
because missing returns and occlusions must still pass the same checks.

Verification evidence: `diagnostics/planner_coverage_20261018T195530Z/`.
**167 planner tests + 110 calibration tests passed**, including actual MoveIt/FCL
detour planning, strict visibility acceptance/rejection with artificial depth,
gripper/stick geometry, synchronization, and simulated execution/fault handling.
Live state queries, five-frame planning capture, read-only alignment, 2D/3D camera
previews and one-frame frozen-mesh playback were exercised. Playback controls and
a complete **simulated** outward/hold/return cycle passed with one captured frame
and no background camera updates. The real-scene plan remains blocked, explicitly
not an end-to-end motion success. Native SDK startup/dropped-frame warnings remain
in the logs even where acquisition succeeds; no warning-free transport claim is
made. No physical trajectory, motor enabling, gripper command or relaxation was
tested or performed. No one-minute soak was imposed.

### Observed-obstacle default: current verification

Evidence: `diagnostics/observed_obstacle_policy_20261018T201044Z/`.

- **179 planner/acquisition/preview/execution-policy tests** and **110 calibration
  regression tests** passed. Tests cover the new default, preserved legacy strict
  replay, explicit policy agreement during execution rechecking, upper-arm/wrist/
  hand/stick collisions, joint bounds, synchronization and unchanged fault handling.
- The previously blocked saved run `20261018T195224Z-current-plan-17b383` now
  produces a `planned` result against its original camera scene and exact measured
  start. No recorded angles, calibration, obstacles or collision geometry changed.
  The first spline intersected a detected stick obstacle and was rejected; the
  existing checked stop-at-corners interpolation succeeded instead. Timing and
  interpolation algorithms were not changed in this policy update.
- A fresh **regular `live --seconds 3`** run used the actual Gemini camera and
  disabled-motor feedback, generated a plan, and opened its matching frozen-mesh
  GUI without an application exception. It retained five native 1280×800 frames
  and 236 complete dual-arm encoder batches. All **3,776 transmitted packets**
  were exact state queries; interface counters matched the packet audit. No
  enable, motion, gripper or relax commands were sent.
- The fresh plan's full **simulated** outward/2-second-hold/reverse cycle completed
  in the GUI at 2× display speed. Play/Pause, goal/hold/return seeking, layers,
  coloring, fit and save worked. The same scene arrays remained unchanged; there
  was one mesh build and zero camera/CAN access attempts during this playback.
  The screenshot was inspected. This was not a physical trajectory test.
- Native SDK startup/dropped-frame warnings remain in camera logs; stderr was
  empty and acquisition passed. This is not a warning-free USB transport claim.
- The default live camera window, old saved previews, actuator packet code,
  gripper target, fault behavior and extra execution-clearance margins remain
  unchanged apart from explanatory text. Physical execution still requires its
  existing explicit arming and actual commissioning checks.
