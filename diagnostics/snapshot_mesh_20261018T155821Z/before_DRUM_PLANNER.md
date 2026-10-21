# Software-only Gemini / MoveIt drum trajectory planner

**No CAN interface, torque control, motor execution, gripper command or relaxation
backend is connected in this version. Keep the physical arm off for these tests.**
This implements planning, depth capture, model generation, visualization and the
simulated movement/recovery policy. It is **not yet a hardware motion program**.
It does not change `start_beat.sh`, recordings, calibration captures, shared URDF,
encoder conversion, centering or existing playback programs.

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
Interactive `preview` now opens the live camera by default, with trajectory
Play/Pause and a timeline; these control only the display, never the robot.
Use `--no-camera` for offline playback. No RViz/controller launch is required.

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

### Play a trajectory over live camera depth

```bash
# Existing camera-based candidate: occupied-only, NOT a verified safe route.
./plan_drums.sh preview planner_runs/camera_preview/plan.json

# Existing synthetic demonstration, also overlaid on the current depth cloud:
./plan_drums.sh preview planner_runs/software_demo/plan.json

# Explicit offline saved-scene playback, with no camera connection:
./plan_drums.sh preview planner_runs/software_demo/plan.json --no-camera

# Timed live software test; these flags animate only the on-screen model:
./plan_drums.sh preview planner_runs/camera_preview/plan.json --autoplay --loop --seconds 65
```

Close any other depth viewer first; the two windows cannot own the same camera
simultaneously. The combined window shows the **simulated robot, custom gripper
and red drumstick on the same calibrated 3D axes as the live depth points**,
with the 2D depth image alongside. It reuses the camera-only viewer and its
native full-FOV capture/freshness checks, raw/voxel layers, coloring, controls,
one-job worker and point budget. It does not start ROS publishers or robot drivers.

- **Play/Pause animation (P)** starts after depth is available. Playback uses
  elapsed wall-clock time rather than counting rendered frames; the saved
  two-second hold and return remain in the animation.
- Scrubbing the **timeline**, **Go to start**, or **Go to goal** pauses playback
  at that point. Speed (0.5×/1×/2×) and Loop affect only animation. Playback starts
  paused unless `--autoplay` is supplied, and normally stops at the end.
- **Freeze depth (Space)** freezes the camera display, NOT the animation or
  acquisition. **Pause animation** leaves depth updating. Use both to inspect
  a completely still composite view.
- **Fit arm + kit** focuses the view around the trajectory. **Reset / fit view
  (R)** shows the full observed depth extent. Pan/zoom/orientation persist during
  updates. Points outside the chosen axes are clipped **for display only**.
- **S / Q / Esc** save current live depth as in the camera-only window. Output
  goes to a new `planner_runs/*-live-playback-*` folder (or `--output PATH`), not
  into the original plan. The plan, recording and generated model are read-only.

**This is visualization, not live collision revalidation or replanning.** The
trajectory was checked against its original saved/synthetic scene, not against
each newly displayed frame. Synthetic and occupied-only warnings remain visible.
The robot overlay comes from saved joint angles, not encoder measurements; the
camera may separately see the real stationary arm. Tool mounting/depth alignment
remain physically unverified. A matching calibration and intact saved robot
model are required before starting the live overlay. Missing/busy/stale camera
data stops the live preview rather than silently substituting an old scene.

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
  renderer and wall-clock playback, live depth overlay by default, explicit
  offline mode/PNG export. No ROS publishing, motor API or live path revalidation.
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

By default, conservative bounding volumes covering every moving collision shape
must fit inside observed free depth. Missing depth, occlusion, outside FOV and
space behind a return are **not free**. This is intentionally stricter than
occupied-voxel-only collision checking and can reject usable-looking routes.
The camera cannot prove an unseen route clear, particularly at the resting arm.

For a **visualization experiment only**, `--occupied-only-preview` can show what
the occupied-point collision planner would propose while ignoring unknown space.
The console, saved report and viewer explicitly label this limitation. It never
unlocks execution. It is not an approved safety workaround.

## Outputs

Depth runs save `scene.json`, `raw_depth.npz`, the conservative depth images,
`points_base.f32`, `depth_preview.png`, `capture_report.json` and SDK logs.
Planning runs save the generated model, exact request, backend logs/results,
interpolated-path rechecks and **`plan.json`**. Saved-file hashes protect model,
calibration, recording and scene compatibility. Always inspect `status`, not
just whether a file exists. Every result has `motion_authorized: false` and
`hardware_execution_available: false`.

No current-state capture, torque ownership, live obstacle watchdog or motor
transport has been connected. Those are a later hardware-integration stage,
after mounting/depth checks and a safe strategy for the actual supported start
and finish. A static camera snapshot is not a live collision safety system.

## Verification on this computer

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
