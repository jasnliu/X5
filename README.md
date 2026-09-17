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
uses the same goal → 2-second hold → recenter → relax controller. The Cartesian
layer controls endpoint position, not orientation or a straight-line trajectory.
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
the existing goal → 2-second hold → special-center sequence while continuously
holding +7°. It never sends another open command before all eight right motors
are disabled. The left Cartesian workflow is unchanged.
