# Verification — 2026-09-14

## Completed

- Built upstream description only on ROS 2 Jazzy; generated bimanual URDF has
  no ros2_control tag. No vendor hardware launch/controller was run.
- 10 offline unit tests passed: SDK query layout, disabled-mode decode,
  enabled/fault rejection, malformed frames, interface separation, convex cube,
  degenerate/flat trace, spacing/finite bounds, JSON validation/round trips,
  and independent left/right forward kinematics. `diagnostics/build_and_tests.log`.
- SDK encoder/decoder probe compiled locally with:
  `g++ tests/sdk_probe.cpp -Wl,--no-as-needed -lopenarmx_can -lrobstride_motor -o /tmp/x5_sdk_probe`.
  Confirmed state query IDs `0x0200fd01`–`0x0200fd08`, payload
  `01 00 00 00 00 00 00 00`, and -12.57 rad lower angle bound.
  The probe does not create a CAN socket.
- Offline panel integration passed: offline recording prohibited, mock left-arm
  recording, save, import, clear, stale-input pause and fault latch. Observer
  constructor patched to fail if called: **no CAN access** in this check.
- Actual RViz GUI opened. Inspected both rendered arms and the translucent
  synthetic hull/points/edges in `diagnostics/offline_viewer.png`.
  Fixed MarkerArray config to use `Topic` before final visual verification.
- ROS verification received joint states and markers and compared the actual
  robot_state_publisher `world`→left TCP transform against local FK to 1e-8 m.
- Both host interfaces were UP, LOWER_UP, ERROR-ACTIVE at classic 1 Mbit/s.
- **Live query-only check:** 60 complete encoder batches, all 16 motor mode
  fields disabled, no reported faults. All 960 transmitted state requests were
  captured independently and matched the allowed exact packet. Evidence:
  `diagnostics/query_check.json`, `diagnostics/query_frames.json`.
- Live query-only viewer opened successfully, displayed actual encoder postures
  and `Paused`, with zero recorded points. See `diagnostics/live_viewer.png`.
  No recording or physical manipulation was performed during this check.
- Live viewer stopped; all three child processes exited cleanly. A parent-only
  SIGINT injection required launch's 5-second SIGTERM fallback (unlike terminal
  Ctrl+C, which signals the process group). App handles SIGINT/SIGTERM and
  closes the observer without sending any motor-state-changing packets.

## Not demonstrated / not claimed

- No powered movement, holding, disable, enable, zeroing or gravity compensation.
- No physical boundary tracing, encoder calibration, left/right physical wiring
  identification by manipulation, or actual workspace/collision-safety proof.
  User must verify the model follows the real left arm before teaching.
- A saved/visible convex hull is not evidence of whole-arm or obstacle clearance.
- Software cannot prevent gravity or another controller from moving the arms.

## Reproduce offline checks

```bash
cd /home/jason/X5
./build.sh
source /opt/ros/jazzy/setup.bash
PYTHONPATH=.:$PYTHONPATH ROS_DOMAIN_ID=86 /usr/bin/python3 tests/ui_check.py
```

For the offline ROS check, first launch the synthetic viewer in another terminal:

```bash
./start_viewer.sh --load diagnostics/SYNTHETIC_TEST_ZONE.json
```

Then (ROS environment sourced):

```bash
ROS_DOMAIN_ID=85 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  PYTHONPATH=.:$PYTHONPATH /usr/bin/python3 tests/ros_check.py
```

`tests/query_check.py` is explicitly **live query-only**, not an offline test.
It must not run while the viewer or another bus owner is running.

## Update: paused-zone membership and camera controls

- Added saved/imported-zone membership text to the panel and RViz, with matching
  green/red TCP colors. Classification is suspended during recording and is unknown
  for offline/stale/faulted input, incomplete hulls or unsaved point changes.
- `tests/membership_check.py` passed using synthetic joint readings and a known
  hull: actual FK positions inside/outside, UI label updates, save/import activation,
  recording suspension, stale/fault/offline rejection, clear, edits and flat/empty
  hulls. Observer construction was forbidden by a mock; no CAN was accessed.
- Existing 10 unit tests and original offline UI integration passed again.
- Added RViz Move Camera tool and Views panel. Launched offline with the synthetic
  hull and verified an actual left-mouse drag rotated the rendering: Orbit yaw
  changed from 0.7 to 0.2 and pitch from 0.35 to 0.15. Inspected before/after
  screenshots in `diagnostics/status_camera_before.png` and
  `diagnostics/status_camera_after.png`. No hardware launch for this update.
- Test viewer and child processes closed cleanly. Restart the user's viewer to
  load the new code/configuration. Live physical inside/outside crossing has not
  been tested; only synthetic encoder positions were used in this update.

## Update: invisible 5 mm membership buffer

- Membership now accepts up to 5 mm beyond each original hull face, in the
  background only. Hull vertices, sampled points, triangles, edges, displayed
  volume and saved geometry are not expanded. Existing imports get the same
  fixed application tolerance without rewriting their files.
- 14 offline unit tests passed. New tests cover all six cube faces including the
  resting lower boundary, exact 5 mm acceptance / 5.01 mm rejection, a sloping
  face, invalid input, no hull, old-format import and byte-identical saved JSON
  before/after buffered membership checks.
- Offline panel integration passed. Mock TCP 4 mm below a saved boundary changes
  from OUTSIDE without tolerance to INSIDE with tolerance; 6 mm below stays
  OUTSIDE. Published original point/surface/edge marker geometry and colors are
  identical with and without the buffer. Existing stale/fault/recording gating
  and save/import checks still pass. Observer construction was forbidden.
- Original offline UI test and Python compilation passed. No CAN access, live
  arm tests or hardware-control changes in this update. Evidence:
  `diagnostics/buffer_unit_tests.log`, `diagnostics/buffer_membership_check.log`.

## REMOVED: previous LEFT-arm centering implementation — historical checks only

Implemented `start_centering.sh` / `centering/`; the recorder/zone geometry and
existing viewer launch remain unchanged. The new hardware path is never selected
by default and never enables on launch. Center explicitly enables only left
IDs 1–7 after preflight; Emergency Relax disables all left IDs 1–8. Right CAN
routing permits exact state-query packets only. See `CENTERING.md` for limits
and commissioning requirements, including the absence of collision/zone guards.

Completed verification (all offline / no physical CAN):

- **30 unit tests passed**, including the existing 14 recorder tests and 16
  new packet, feedback-policy, cancellation and supervisor tests.
- Compared SDK-generated state queries, complete disables, enables, MIT-mode
  writes and representative motion packets for all applicable IDs against the
  new Python packet builders. Probe constructs packets only; no sockets.
- Verified routing rejects right-arm actuation, gripper enable/motion/mode writes
  and zero-recalibration packets. Verified a stop after the first enable blocks
  all later enables and still attempts disable for every left motor three times.
- Loaded numerical-plant convergence tested against actual simulated encoder
  error, bounded correction, setpoint lead and URDF limits. Stationary jitter,
  obstruction/no-progress, bad timing and out-of-limit inputs tested.
- Verified fault latch, missing GUI heartbeat, explicit relax, closed command
  pipe and disable-confirmation failure behavior. Cannot clear fault with Center.
- Separate demo subprocess reached HOLDING, relaxed on command, then relaxed and
  exited on heartbeat loss while centering. Another centered-start subprocess
  relaxed/exited when its input pipe closed. No --hardware flag was passed.
- Opened actual RViz + the OFFLINE DEMO panel. Clicked **Center Left Arm** and
  observed centered/HOLDING simulated encoders, then clicked the red **Emergency
  Relax** and observed RELAXED. The right model and both grippers stayed fixed.
  Screenshots: `diagnostics/centering_demo_before.png`,
  `diagnostics/centering_demo_centered.png`, `diagnostics/centering_demo_relaxed.png`.
- ROS display check received all 16 joint positions and DEMO RELAXED marker,
  verified right joints/grippers fixed and left simulated angles within 3°,
  and compared robot_state_publisher TF against local FK to 1e-8 m.
- Existing recorder UI and buffered-membership integration checks passed again.
- Test viewer and backend shut down cleanly; Python and shell syntax checked.

Evidence: `diagnostics/centering_unit_tests.log`,
`diagnostics/centering_sdk_packets.txt`, `diagnostics/centering_process_check.log`,
`diagnostics/centering_demo_trace.json`, `diagnostics/centering_ros_check.log`,
`diagnostics/centering_gui_demo.log`.

**Not tested:** powered enable/centering/holding/relax, real zero-position accuracy,
physical left/right adapter identification, collision clearance, actual gain/torque
suitability, or firmware communication-loss watchdog. No hardware control commands
were sent by these checks. Hardware motion remains a separate supervised step.


## Replacement: simple direct-zero centering

The previous centering backend, heartbeat, numerical plant, custom feedback policy
and its dedicated tests were deleted at the user's request. They are not the
current program; the historical checks above do not validate this replacement.

Current implementation: `centering/app.py` + `centering/motors.py`. Center selects
the SDK's native CSP position mode (value 5), sets firmware speed/current settings,
and sends position zero to left IDs 1–7. No custom correction loop or heartbeat.
The gripper is never enabled. Right CAN sends state queries only. Relax cancels
pending commands and sends complete disable to left IDs 1–8, then checks feedback.
It does not physically switch off the electrical supply.

Offline verification: 20 unit tests passed (14 recorder + 6 replacement tests),
plus a no-CAN panel check of Center, Relax and cancellation. New CSP mode/speed/
current/zero packet layouts were checked against the installed SDK's csp_* packet
helpers in an offline compiled probe. Evidence: `diagnostics/simple_centering_tests.log`.
Python compilation and launcher shell syntax passed. No powered centering or
CAN communication was performed. Native position-mode behavior on this physical
arm remains unverified. The safe-zone recorder and saved zones were left intact.

## LEFT center then 45-degree goal — physical verification, 2026-09-14

Implemented `start_goal.sh` / `goal_motion/`. The physical sequence centers left
IDs 1–7, then targets 45 degrees on IDs 4 and 7 while the other joints target
zero. It uses motor-native CSP position control plus a simple measured-error
bias capped at 12 degrees. Completion requires all seven joints within 3 degrees
for 0.6 seconds. `left_zones/zone1.json` is loaded and its original markers are shown;
the existing 5 mm classification tolerance is used only in the background.
A powered zone breach, lenient 2.5-second stall condition, stale feedback, motor
fault/temperature, or 30-second phase timeout invokes left IDs 1–8 disable.
Right CAN is state-query-only and left gripper cannot be enabled by the whitelist.

Verification:

- Query-only preflight: both `can0` and `can1` UP/LOWER_UP/ERROR-ACTIVE at
  classic 1 Mbit/s; all 16 motors reported disabled. Current-left-to-zero and
  zero-to-goal FK paths were sampled offline and were inside zone1's tolerance.
- 29 final offline tests passed: packet/routing constraints, CSP zero/goal motor
  sign, loaded-plant correction convergence, lenient stall/pass cases, runtime
  zone breach->relax behavior, exact requested goal, complete nominal path,
  CAN queue retry, and all prior recorder tests.
- First physical run reached displayed 45.0 degrees at joints 4 and 7 with all
  other left joints displayed 0.0 and `Zone: INSIDE`. Its Emergency Relax exposed
  a real ENOBUFS transmit-queue error because disables were sent back-to-back.
  The arm released, the viewer reported the disable as unconfirmed, and the test
  was stopped. A separate 60-batch query-only audit immediately confirmed every
  motor was in disabled mode.
- Fixed only that issue by adding a bounded 2 ms retry for EAGAIN/ENOBUFS CAN
  writes and fresh disabled-state confirmation in the GUI. Offline retry test passed.
- Full physical retest succeeded. An independent ROS subscriber recorded 952
  samples and the phases READY -> SETTING UP -> CENTERING -> MOVING TO GOAL ->
  GOAL REACHED in about 15.2 seconds. Final encoder angles were:
  J1 0.033, J2 0.011, J3 0.011, J4 45.025, J5 0.011, J6 0.011, J7 45.003 degrees.
- Recomputed TCP membership for all 952 samples. Maximum signed distance beyond
  the original drawn hull was 3.606 mm, inside the 5 mm background tolerance.
  There was no powered zone breach, stall, timeout, encoder loss, temperature
  fault, or CAN fault. The right-arm maximum observed encoder span was 0.022 degrees.
- Tested Emergency Relax again from the held goal. GUI confirmed `Left motors
  disabled`. Closed the viewer, then ran another 60-batch query-only audit:
  all 16 motors remained disabled; all 960 captured outgoing packets were exact
  state requests. No test/viewer processes remain active.

Evidence: `diagnostics/goal_live_encoder.csv`, `diagnostics/goal_live_retest_goal.png`,
`diagnostics/goal_live_retest_relax.png`, `diagnostics/goal_live_final_joint_states.txt`,
`diagnostics/goal_post_relax_query.json`, `diagnostics/goal_final_tests.log`, and
`diagnostics/goal_live_retest.log`.

Limit: software disable removes commanded torque but does not electrically cut
power. The zone protects only the modeled left TCP, not every arm link. Stall and
safety fault injection were verified offline rather than intentionally obstructing
or driving the real arm outside the zone.

## Opposite joint-7 goal — physical verification, 2026-09-14

Changed only the goal direction for left joint 7: joint 4 remains +45 degrees,
joint 7 is now -45 degrees, and the other five left joints remain at zero.

- A query-only preflight found both arms disabled. The measured-start-to-zero and
  zero-to-new-goal FK paths fit inside zone1's existing 5 mm tolerance.
- All 29 offline tests passed with the new target, including correction, stall,
  safety-zone, left-only routing, and nominal-path checks.
- The live sequence reached GOAL REACHED in about 13.8 seconds. Final left encoder
  angles were J1 0.033, J2 0.011, J3 0.011, J4 45.003, J5 0.011, J6 0.011,
  and J7 -45.003 degrees.
- All 860 independently recorded ROS samples remained inside the buffered zone.
  Maximum signed distance beyond the original visible hull was 1.820 mm, below
  the invisible 5 mm tolerance. Right-arm maximum encoder span was 0.022 degrees.
- Emergency Relax reported `Left motors disabled`. After shutdown, a separate
  60-batch query-only audit confirmed all 16 motors disabled and captured only
  the expected 960 state-query packets. No program processes remained active.

Evidence: `diagnostics/negative_goal_live_encoder.csv`,
`diagnostics/negative_goal_reached.png`, `diagnostics/negative_goal_relaxed.png`,
`diagnostics/negative_goal_offline_tests.txt`,
`diagnostics/negative_goal_preflight_query.json`, and
`diagnostics/negative_goal_post_relax_query.json`.

## Automatic recenter and 80-degree goal — physical verification, 2026-09-14

Normal completion now continues from the goal into a measured-error recentering
phase. It disables the left motors only after all seven left encoders remain
within 3 degrees of zero for 0.6 seconds. Emergency Relax and safety/stall faults
still disable immediately rather than trying to move through an emergency.

The recentering change was first tested physically with the prior +45/-45 goal.
That run reached J4 45.003 and J7 -45.003 degrees, returned to approximately zero
on all seven left joints, then reached RELAXED with disabled feedback. All 1,455
recorded samples stayed inside zone1.

The goal was then changed to J4 +80 degrees and J7 -80 degrees:

- All 30 offline tests passed. Dense FK sampling of measured-start-to-center,
  center-to-goal, and goal-to-center remained inside zone1's 5 mm tolerance.
- The full live phase sequence was READY -> SETTING UP -> CENTERING -> MOVING TO
  GOAL -> RECENTERING -> RELAXING -> RELAXED, with no safety, stall, timeout,
  encoder, temperature, or CAN fault.
- Goal-transition encoders were J1 0.011, J2 0.011, J3 0.011, J4 80.016,
  J5 0.011, J6 0.011, and J7 -79.994 degrees.
- Immediately before confirmed disable, J1-J6 were 0.011 degrees and J7 was
  -0.011 degrees.
- All 1,770 recorded samples stayed inside the buffered zone. Maximum signed
  distance beyond the original visible hull was 1.872 mm. The right arm remained
  disabled; its maximum encoder span was 0.044 degrees.
- A post-shutdown 60-batch query audit confirmed all 16 motors disabled and all
  960 captured outgoing frames were exact state queries. No program remains active.

Evidence: `diagnostics/recenter_45_live_encoder.csv`,
`diagnostics/recenter_45_complete.png`, `diagnostics/recenter_45_post_query.json`,
`diagnostics/goal_80_live_encoder.csv`, `diagnostics/goal_80_complete.png`,
`diagnostics/goal_80_analysis.txt`, `diagnostics/goal_80_offline_tests.txt`, and
`diagnostics/goal_80_post_relax_query.json`.

## Doubled movement speed — physical verification, 2026-09-14

Changed the shared native CSP firmware speed setting from 0.2 rad/s to 0.4 rad/s.
The +80/-80 goal, correction limits, zone tolerance, stall thresholds, current
settings, automatic recentering, and emergency behavior were not changed.

- All 30 offline tests passed, including an exact check of the SDK-compatible
  0.4 rad/s parameter packet. The measured-start-to-center, center-to-goal, and
  goal-to-center FK paths remained inside zone1's tolerance.
- The live run completed READY -> SETTING UP -> CENTERING -> MOVING TO GOAL ->
  RECENTERING -> RELAXING -> RELAXED with no fault.
- Goal-transition encoders were J4 80.016 and J7 -79.994 degrees. All other left
  joints were 0.011 degrees. Before confirmed disable, every left joint was
  within 0.165 degrees of zero.
- Button-to-relaxed time fell from 18.560 seconds at 0.2 rad/s to 11.065 seconds
  at 0.4 rad/s. Constant-rate fits through the 10-to-70-degree portions were
  0.357/0.390 rad/s outbound and 0.364/0.398 rad/s returning for J4/J7.
- All 1,318 recorded samples stayed inside the buffered safe zone. Maximum
  signed distance beyond the original visible hull was 1.698 mm. The right arm
  remained disabled; its maximum encoder span was 0.198 degrees.
- A post-shutdown 60-batch query audit confirmed all 16 motors disabled and all
  960 captured outgoing frames were exact state queries. No program remains active.

No unsafe behavior was observed for this exact path and hardware setup. This is
not a general safety certification: zone1 checks only the modeled left TCP, and
higher speed reduces physical reaction and stopping margin.

Evidence: `diagnostics/goal_80_double_speed_live_encoder.csv`,
`diagnostics/goal_80_double_speed_complete.png`,
`diagnostics/goal_80_double_speed_analysis.txt`,
`diagnostics/goal_80_double_speed_offline_tests.txt`,
`diagnostics/goal_80_double_speed_preflight_query.json`, and
`diagnostics/goal_80_double_speed_post_relax_query.json`.

## Added J1 +45 goal and zone-only recenter recovery — 2026-09-14

The current goal is J1 +45, J4 +80, and J7 -80 degrees, with the other left
joints at zero and the speed unchanged at 0.4 rad/s. A powered zone breach now
cancels the active stage and enters ZONE RECENTERING; once centered it relaxes.
A stall or any non-zone fault still disables immediately without recentering.
Stall or timeout during zone recovery also disables immediately.

- The final goal TCP is inside the original visible zone by 101.674 mm. Dense
  sampling of measured-start-to-center, center-to-goal, and goal-to-center stayed
  inside the 5 mm buffered classification volume.
- All 32 final tests passed. Fault-injection coverage verifies zone breach ->
  center controller -> relax, suppression of repeated recovery commands, and
  stall -> immediate relax with no center command. No intentional physical zone
  breach or obstruction was performed.
- The live goal-and-return cycle completed with goal-transition encoders J1
  45.025, J4 80.016, and J7 -79.994 degrees; J2, J3, J5, and J6 were 0.011
  degrees. It returned every joint to within 0.253 degrees of zero, then relaxed.
- All 1,445 live samples stayed inside the 5 mm tolerance. Maximum signed distance
  beyond the original hull was 2.373 mm. The right arm remained disabled and its
  maximum encoder span was 0.066 degrees.
- The post-shutdown query audit confirmed all 16 motors disabled and all 960
  captured outgoing frames were exact state requests. No program remains active.

Evidence: `diagnostics/goal_j1_45_live_encoder.csv`,
`diagnostics/goal_j1_45_complete.png`, `diagnostics/goal_j1_45_analysis.txt`,
`diagnostics/goal_j1_45_path_check.txt`, `diagnostics/goal_j1_45_final_tests.txt`,
`diagnostics/goal_j1_45_preflight_query.json`, and
`diagnostics/goal_j1_45_post_relax_query.json`.

## Cartesian gripper input layer — physical verification, 2026-09-14

Added `cartesian_goal/`, `start_cartesian_goal.sh`, and
`launch_cartesian_goal.py` without removing the existing joint-angle program.
Inputs are X/Y/Z meter offsets from the URDF all-zero left TCP. The layer checks
the absolute target against buffered zone1, performs bounded position-only IK,
filters the nominal out-and-back joint path through zone1, then supplies the
computed angles to the unchanged measured-joint controller.

- 38 final offline tests passed. New coverage includes coordinate parsing,
  centered-origin behavior, 5 cm upward IK accuracy, joint limits, planned-path
  membership, outside-zone rejection before motor setup, and post-IK fresh
  feedback reacquisition.
- Offline RViz/Tk preview displayed the coordinate fields, live relative TCP,
  computed joint pose, and magenta target marker.
- The first live button invocation exposed a genuine startup issue without
  moving the arm: the roughly 1.2-second IK calculation made cached feedback
  stale. Fixed it by keeping IK unpowered, resuming normal polling, and waiting
  for fresh feedback before any enable. The regression test covers this order.
- The only powered Cartesian target tested was `(0, 0, +0.050 m)`. Its goal point
  is 28.622 mm inside the original zone hull. Computed IK error was effectively
  zero and the nominal path passed zone1.
- Physical goal-transition angles were J1 23.837, J2 0.011, J3 0.011, J4 23.221,
  J5 0.011, J6 0.033, and J7 -33.639 degrees. Measured relative TCP was
  `(-0.001884, -0.000091, +0.049855) m`, 1.891 mm from the requested point.
- The arm then recentered and relaxed. All 845 recorded samples were inside the
  5 mm zone tolerance; maximum signed distance beyond the original hull was
  3.905 mm. Right-arm maximum encoder span was 0.066 degrees.
- X=2.000 m was entered after relaxation. The GUI reported the outside-zone
  error and did not initiate setup. The final 60-batch query audit confirmed all
  16 motors disabled and captured only the expected 960 state requests. No
  viewer or controller remains active.

The program reaches a Cartesian endpoint by converting it to one fixed joint
goal. It does not command a straight Cartesian trajectory or gripper orientation;
the 5 cm upward test's joint-space path had 23.424 mm maximum lateral deviation.

Evidence: `diagnostics/cartesian_goal_offline_tests.txt`,
`diagnostics/cartesian_goal_offline_goal.png`,
`diagnostics/cartesian_up_live_encoder.csv`, `diagnostics/cartesian_up_analysis.txt`,
`diagnostics/cartesian_up_complete.png`, `diagnostics/cartesian_outside_rejected.png`,
`diagnostics/cartesian_up_plan.txt`, `diagnostics/cartesian_up_preflight_query.json`,
`diagnostics/cartesian_up_retry_preflight_query.json`, and
`diagnostics/cartesian_up_post_relax_query.json`.

## Two-second goal hold — physical verification, 2026-09-14

The shared measured-joint state machine now keeps commanding the accepted goal
for two continuous seconds before it begins recentering. The normal correction,
stall detection, zone monitoring, Emergency Relax, and 30-second stage timeout
remain active during `HOLDING GOAL`. If any joint leaves the existing 3-degree
tolerance, the two-second timer restarts.

- All 39 final offline tests passed, including tests that forbid recentering
  before the timer expires and verify that leaving tolerance restarts the timer.
- The only powered target used was again `(0, 0, +0.050 m)`. The captured phase
  order was IK SOLVED -> SETTING UP -> CENTERING -> MOVING TO GOAL -> HOLDING
  GOAL -> RECENTERING -> RELAXING -> RELAXED.
- The observed `HOLDING GOAL` to `RECENTERING` interval was 2.038 seconds across
  125 hold samples. Maximum hold error was 0.090 degrees at any joint and
  1.294 mm at the TCP. At the return transition, TCP error was 0.119 mm.
- All 1,010 samples remained inside zone1's 5 mm tolerance. Maximum signed
  distance beyond the visible hull was 2.667 mm. Right-arm maximum encoder span
  was 0.066 degrees.
- The left arm recentered to within 0.429 degrees, relaxed, and the post-run
  query-only audit confirmed all 16 motors disabled using 960 exact state-query
  packets. No intentional stall or zone breach was performed.

Evidence: `diagnostics/cartesian_hold_live_encoder.csv`,
`diagnostics/cartesian_hold_live_analysis.txt`,
`diagnostics/cartesian_hold_preflight_query.json`,
`diagnostics/cartesian_hold_post_relax_query.json`, and
`diagnostics/cartesian_hold_offline_tests.txt`.

## Live Cartesian target preview — offline verification, 2026-09-14

The Cartesian X/Y/Z `StringVar` values now have write traces that update the
RViz goal marker without invoking IK or motor setup. The preview is classified
against zone1's existing 5 mm background tolerance: magenta inside, red outside,
and deleted while input is incomplete or nonnumeric.

- All 42 offline tests passed. New coverage verifies coordinate-to-marker
  updates, outside classification, invalid-input clearing, and explicit RViz
  marker deletion.
- In the offline/no-CAN viewer, changing the fields to
  `(0.020, -0.010, 0.030)` moved marker 5 to
  `(0.019426, 0.159893, 0.087089) m`, exactly the centered TCP plus the input.
- Setting X to 2.000 m changed the marker to red and the panel to `OUTSIDE
  zone1`. Invalid text produced marker action `DELETE`; restoring the default
  `(0, 0, 0.050)` restored the magenta marker.
- The viewer remained `Offline preview — no CAN`, and Run was never invoked.
  Therefore this UI verification opened no CAN sockets and issued no movement.

Evidence: `diagnostics/cartesian_live_preview_offline_test.txt` and
`diagnostics/cartesian_live_preview_offline_tests.txt`.

## Separate left/right zone recorders — offline verification, 2026-09-14

The original `zones/` directory was replaced by `left_zones/` and
`right_zones/`. The existing `zone1.json` was moved without modifying its
contents to `left_zones/zone1.json`; `right_zones/` was empty at the time of
that split and was populated by the user afterward. All
left-arm motion and Cartesian programs now load the moved left-zone path.

`start_left_zone.sh` fixes the recorder to the left TCP and ROS domain 85.
`start_right_zone.sh` fixes it to the right TCP and ROS domain 86. Both use the
same query-only implementation and observe all 16 disabled motors, but only the
selected arm contributes recorded points. Save filenames are reduced to their
basename and written beneath the selected recorder's folder, so selecting a
different directory cannot redirect a save. Left and right JSON files use
different TCP/schema fields and a cross-arm import is rejected.

- All 44 unit tests passed, including right-only FK, right-zone round-trip, and
  left/right schema mismatch rejection.
- A dual-recorder Tk integration test instantiated both variants with CAN access
  forbidden. Each recorded its selected TCP and saved a valid JSON file only in
  its assigned temporary left/right folder, even when the dialog returned a path
  elsewhere.
- The actual offline left launcher loaded `left_zones/zone1.json` and displayed
  1,197 samples, 300 hull vertices, and the LEFT recorder labels in RViz/Tk.
- The actual offline right launcher displayed the RIGHT recorder labels, zero
  samples, and no 3-D envelope, as expected for the empty `right_zones/` folder.
  Neither launch used `--hardware`, so no CAN socket or motor command was used.

Evidence: `diagnostics/dual_zone_offline_tests.txt` and
`diagnostics/dual_zone_ui_check.txt`.

## Separate right Cartesian program and asymmetric center — 2026-09-14

Added `start_left_cartesian.sh` and `start_right_cartesian.sh`; the old
`start_cartesian_goal.sh` remains a left-program alias. The shared controller,
motor transport, UI, and IK are selected by arm so the right version publishes
the right pose/TCP, loads `right_zones/zone1.json`, sends control packets only on
the right bus, and treats every left motor plus both grippers as required-relaxed.

The left center remains all zeros. The corrected right center is all zeros except
J7 at its positive URDF limit of +1.4 rad (+80.214 degrees), corresponding to the
right motor's reversed negative direction. Right Cartesian `(0,0,0)` remains the
modeled right TCP at all-zero joints, not the right center TCP. IK targets are
relative to that origin but center-to-goal path checks and normal/zone-fault
recentering use the asymmetric J7-positive center. Correction, 2-second hold,
stall handling, zone recovery, stage timeout, live preview, and Emergency Relax
remain shared.

- A 60-batch query-only preflight confirmed all 16 motors disabled and audited
  960 exact state-request frames. It sent no state-changing packet.
- All 50 offline tests pass. The right motor packet test confirms that a modeled
  J7=+1.4-rad center emits the reversed -1.4-rad motor setpoint and uses only the
  right socket. A non-relaxed left motor blocks right setup.
- The offline UI check displayed the right zone, right encoders, right goal
  marker, J7=+80.2-degree center label, and unchanged all-zero Cartesian origin.
  The left UI retained its zero center. The check forbade motor transport.
- The real right-zone IK check accepted the 5 cm upward target and solved it to
  numeric precision with the complete sampled center-to-goal path inside zone1.

After correcting the right-arm sign convention, current-zone offline preflight
confirms the J7=+80.214-degree right center is inside the original right hull.
The `(0,0,+0.050 m)` endpoint and the IK-selected center-to-target path are also
inside the buffered zone. The all-zero right TCP remains 44.083 mm outside the
original hull, but it is only the coordinate origin and not the physical right
center. Entering `(0,0,0)` as a goal remains correctly blocked. No powered
movement was attempted.

Evidence: `diagnostics/right_cartesian_preflight_query.json`,
`diagnostics/right_cartesian_geometry_check.txt`,
`diagnostics/right_center_positive_offline_tests.txt`, and
`diagnostics/right_cartesian_ui_check.txt`.

## Cartesian gripper encoder display — 2026-09-14

The shared Cartesian encoder panel now adds `Gripper (motor 8)` below joints
1–7. In hardware mode it reads motor 8's latest selected-arm encoder feedback;
it does not enable or position the gripper. Tests separately injected different
left/right gripper values and confirmed the selected Cartesian program displays
only its matching gripper. Both offline left/right UI constructions include the
new row while forbidding motor transport. All 52 offline tests pass; no physical
movement was performed.

Evidence: `diagnostics/cartesian_gripper_encoder_offline_tests.txt` and
`diagnostics/cartesian_gripper_encoder_ui_check.txt`.

## Right-only load-and-grip Cartesian workflow — 2026-09-14

The right Cartesian program now configures motor 8 in the same bounded CSP
position mode as the right arm. It opens to the encoder-panel value −3° during
the special J7=+80.214° centering stage, waits for both targets, and then pauses
with a disabled-until-ready Continue button. Continue sends +7°, waits 0.75
seconds for the 10° closing motion, and only then starts the existing Cartesian
goal stage. The +7° command is retained during goal motion, the 2-second hold,
normal recentering, and zone-fault recentering. Normal completion, Emergency
Relax, arm stall, and other faults disable right motors 1–8.

Protocol tests verify that the UI's −3°/+7° sign convention produces only
+3°/−7° native CSP setpoints, respectively. Motor 8 commands are accepted only
for the right Cartesian transport and only inside those endpoints. The left
program still configures and commands IDs 1–7 only; its motor-8 whitelist remains
disable-only. Tests also verify the pause, Continue ordering, delayed arm start,
closed-target latching, selected-bus isolation, and custom RViz open/closed
mapping. All 59 offline tests pass. No physical motors were enabled or moved.

Evidence: `diagnostics/right_gripper_workflow_offline_tests.txt` and
`diagnostics/right_gripper_workflow_ui_check.txt`.

## Standalone right Cartesian J2 redundancy fix — 2026-09-18

The exact requested live target `(0.300, 0.050, 0.250) m` reproduced the fault.
The original IK requested right J2 `-4.370°`; J2 stopped near `-1.286°` while the
other six joints settled, leaving the measured TCP near
`(0.30038, 0.02543, 0.24822) m` until the 30-second stage timeout disabled the
right arm. A separate 60-batch state-query audit then confirmed all 16 motors
disabled.

The standalone right Cartesian initial and Update solvers now keep J2 at the
custom center's `0°` value and solve the position-only target with the other six
joints. This selection is conditioned on the standalone right manual-finish
workflow: the left Cartesian program and `right_camera_cartesian` base retain
their prior solver configuration. Motor transport, measured-joint correction,
completion thresholds, zone/path checks, and gripper behavior were not changed.

- The corrected goal was approximately J1 `+5.661°`, J2 `0°`, J3 `-7.657°`,
  J4 `+18.967°`, J5 `-4.831°`, J6 `+0.890°`, and J7 `+80.214°`. Offline IK error
  was below numerical reporting precision; outbound and center-return paths were
  inside the buffered right zone.
- The physical retest entered `HOLDING GOAL` at
  `(0.30003, 0.04981, 0.25011) m`. Across the final one-second hold window, the
  median was `(0.300197, 0.049808, 0.250160) m`; maximum absolute per-axis error
  was `0.197 mm`.
- End returned to the customized right center and the program confirmed all
  right modes disabled. The live trace recorded every left mode as disabled and
  no left command. An independent post-run audit captured 960 exact state-query
  frames and again confirmed all motors disabled.
- All 104 offline regression tests passed. Additional scope construction checks
  confirmed centered-J2 IK only for `right_cartesian`, not left Cartesian or the
  camera Cartesian base.

Evidence: `diagnostics/right_cartesian_030_005_025_live_encoder.csv`,
`diagnostics/right_cartesian_030_005_025_failure_post_query.json`,
`diagnostics/right_cartesian_030_005_025_fixed_live_encoder.csv`,
`diagnostics/right_cartesian_030_005_025_fixed_live_result.json`,
`diagnostics/right_cartesian_030_005_025_fixed_post_query.json`,
`diagnostics/right_cartesian_joint2_centered_full_tests.txt`, and
`diagnostics/right_cartesian_joint2_centered_scope_check.txt`.

## Right camera Cartesian default goal update — 2026-09-18

The fixed `right_camera_cartesian` starting offset is now
`(X,Y,Z) = (0.250, 0.000, 0.350) m`. The existing no-detection behavior still
increments only Y by `0.010 m`. With the current right zone, offline preflight
accepted 20 centered-J2 goals from Y `0.000` through `0.190 m`; Y `0.200 m` is
the first rejected point. All 104 repository tests passed, and an offline Tk
construction confirmed the new preview and label with motor transport absent.
No CAN socket was opened and no physical movement was performed for this change.

Evidence: `diagnostics/camera_search_new_default_tests.txt`,
`diagnostics/camera_search_new_default_full_tests.txt`, and
`diagnostics/camera_search_new_default_ui_check.txt`.

## Right camera Cartesian infeasible-seed fix — 2026-09-18

The candidate planner no longer sends a slightly out-of-range live encoder
reading directly to SciPy as its initial IK point. The customized right center
puts J7 exactly at its +1.4-rad URDF maximum, so ordinary encoder quantization
could report J7 a few counts above that boundary. SciPy then raised `x0 is
infeasible` before solving or commanding any candidate, making all directions
appear to be skipped without visible arm movement.

The planner now permits at most three encoder counts of discrepancy and clips
only the numerical optimizer seed a negligible distance inside the bounds. It
continues to use the original measured joint vector for candidate-transition,
anchor-return, and center-return checks. A larger out-of-limit reading is still
rejected and its joint-specific cause is reported. Planner skips are printed,
and final all-skipped failure text now identifies the last actual rejection
instead of calling it a normal non-improving search.

- A regression injected a two-count J7 overshoot. The old raw SciPy call
  reproduced `ValueError: x0 is infeasible`; the corrected planner solved the
  +X candidate with effectively zero Cartesian error, kept the result within
  all URDF limits, and left the measured safety anchor unchanged.
- A separate test injected a six-count violation and confirmed it remained a
  hard error rather than being clipped away.
- All 107 repository unit tests passed. No CAN socket was opened and no physical
  arm movement was performed for this change.

Evidence: `diagnostics/camera_search_infeasible_seed_check.txt` and
`diagnostics/camera_search_seed_clip_full_tests.txt`.

## Query-only right-arm motion recorder — 2026-09-18

A standalone right-arm recorder now launches an RViz encoder visualization and
a separate Tk recording panel. Its hardware transport opens only `can0`, locks
out this workspace's other CAN programs, and sends the exact state request to
right IDs 1–8. Feedback must report every right motor disabled and fault-free.
The application has no motor-control import or path for enable, disable,
configuration, position, velocity, torque, zeroing, centering, or holding.

Recordings may begin and end at arbitrary poses. They contain timestamped right
J1–J7 radians, gripper opening, derived right TCP coordinates, units, UTC start,
duration, joint order, model hash, and schema metadata. JSON saves are confined
to `recordings/` and use flush, fsync, and atomic replacement. The 100,000-sample
limit provides about 83 minutes at the nominal 20 Hz rate.

Offline tests validate the JSON schema and atomic round trip, arbitrary first and
last poses, timestamp/value rejection, enforced save folder, exact right-only
state requests, absence of left samples, and launcher isolation. The offline Tk
integration constructs the panel with CAN forbidden, records injected synthetic
right feedback, and validates the saved file. Full RViz startup is checked
separately. All 114 repository unit tests pass. No physical CAN interface was
opened and no arm was moved.

Evidence: `diagnostics/right_motion_recording_focused_tests.txt`,
`diagnostics/right_motion_recording_ui_check.txt`,
`diagnostics/right_motion_recording_rviz_check.txt`, and
`diagnostics/right_motion_recording_offline.png`,
`diagnostics/right_motion_recording_transport_audit.txt`, and
`diagnostics/right_motion_recording_full_tests.txt`.

## Recorded-path right camera alignment — 2026-09-18

A separate `start_beat.sh` program now preflights and replays a
saved right-arm JSON trajectory, then runs camera-guided Cartesian alignment
from the recorded endpoint. It centers and opens the gripper first, waits for
Continue, closes and retains the gripper command, moves to the recording's
first pose, follows the time-scaled J1–J7 path, and settles at the final pose.

Fresh camera observations collected during playback are paired with the actual
encoder-derived TCP coordinates. Ending motion versus camera-center error is
used only to reorder the six Cartesian test directions; no direction is
removed. Alignment must reach an invisible centered box at 50% of the visible
pink target's width and height before the hold begins, while the two-second hold
continues to use the full visible pink rectangle.

Offline validation covers recording schema/model/limit checks, complete
recorded segments, center transitions, representative recovery returns, global
speed scaling, tracking faults, endpoint J2 retention, playback-derived
direction ordering, and distinct inner-acquisition/outer-hold behavior. The
separate GUI was constructed and preflighted with CAN unavailable. All 127
repository tests pass. SHA-256 comparison confirms that the original
`camera_search/` package and right-camera Cartesian launch files were not
changed. No CAN socket was opened and no physical arm movement was performed.

Evidence: `diagnostics/right_camera_playback_focused_tests.txt`,
`diagnostics/right_camera_playback_ui_check.txt`,
`diagnostics/right_camera_playback_original_unchanged.txt`,
`diagnostics/right_camera_playback_static_audit.txt`, and
`diagnostics/right_camera_playback_full_tests.txt`.

## Recorded-path default file — 2026-09-18

The recorded-path camera launcher and direct application entry point now select
`recordings/record1.json` by default. Passing `--recording` or using the file
chooser still overrides that selection. The existing safety preflight remains
unchanged. All 127 repository tests pass. The current `record1.json` was also
checked offline and is selected correctly, but its contents are rejected by
the unchanged safety preflight because sample 8 places J7 0.5710 degrees above
the URDF limit. The recording and the safety threshold were not modified. No
CAN interface was opened and no arm was moved.

Evidence: `diagnostics/right_camera_playback_default_recording_tests.txt`,
`diagnostics/right_camera_playback_default_recording_full_tests.txt`, and
`diagnostics/right_camera_playback_default_recording_preflight.txt`.

## Recorder joint-limit abort — 2026-09-18

The query-only right-arm recorder now checks every incoming J1–J7 sample against
the current URDF limits. A measurement beyond the limit and the shared
three-encoder-count tolerance produces a modal warning with the joint and angle,
stops recording, resets the complete in-memory take, clears its saved-path
state, and disables Save. Starting is also blocked when the current pose is
already outside a joint limit. The warning is recoverable after the user moves
the limp arm back inside the limits; it does not open any motor-control path.

Focused logic tests and an offline Tk integration verified the warning and that
an earlier valid sample is erased when a later sample violates J7. No CAN socket
was opened and no arm was moved. An offline regression over the existing
`record1.json` confirms the new monitor would reject its first recorded sample
for J7, preventing that unreplayable take from being saved. All 128 repository
tests pass.

Evidence: `diagnostics/right_motion_recording_limit_focused_tests.txt`,
`diagnostics/right_motion_recording_limit_ui_check.txt`,
`diagnostics/right_motion_recording_limit_full_tests.txt`, and
`diagnostics/right_motion_recording_limit_record1_regression.txt`.

## Playback cymbal-box change overlay — 2026-09-19

The separate recorded-path camera process now overlays cymbal bounding-box
dimension status in its processed preview. Consecutive processed detections are
compared by width and height; changes of 4 pixels or less are ignored, while a
change above 4 pixels in either dimension reports `CYMBAL BOX: CHANGING`.
Baseline, stable, changing, and not-detected states show current dimensions and
signed deltas where applicable.

The monitor and drawing are confined to `camera_playback/camera.py`. It delegates
the actual camera loop and datagram transmission to the unchanged original
implementation, so the overlay itself sends no new protocol fields. The later
strike feature independently applies the same threshold to existing cymbal-box
coordinates in the controller. Focused offline tests cover the threshold boundary, significant changes,
missing/reacquired detections, status text, and overlay drawing. No camera, CAN
interface, or arm movement was used. A synthetic OpenCV render confirmed the
two-line `CHANGING` overlay, and a local datagram regression confirms no visual
status fields enter the controller protocol. All 130 repository tests pass, and
SHA-256 checks confirm that the original camera Cartesian implementation remains
unchanged.

Evidence: `diagnostics/right_camera_playback_box_change_focused_tests.txt`,
`diagnostics/right_camera_playback_box_change_full_tests.txt`,
`diagnostics/right_camera_playback_box_change_overlay.png`, and
`diagnostics/right_camera_playback_box_change_original_unchanged.txt`.

## Playback J7 cymbal-strike phase — 2026-09-19

After the inner-target acquisition and continuous two-second outer-pink-zone
hold, the recorded-path program now preserves the actual reached pose instead
of immediately centering. In a worker thread it generates cumulative J7-only
targets 10 degrees below that anchor, then 20, 30, and so on. It retains only
the consecutive depths whose complete strike path and sampled direct-center
returns remain within buffered right zone1 and within the J7 URDF limit.

Only right motor 7 can receive the new 2.0 rad/s speed setting; ordinary motion
remains 0.4 rad/s, other motors cannot use the fast allowlist entry, and the
left transport rejects it. The strike controller sends exact endpoints without
correction overshoot, declares the endpoint immediately within 2 degrees, and
then commands the preserved anchor without the ordinary settling delay.

Every strike camera frame is compared using the existing greater-than-4-pixel
cymbal width/height rule, but a hit cannot latch until actual J7 motion has
started. A hit restores 0.4 rad/s before centering. Camera loss, zone breach,
timeout, ordinary failure, and no-hit exhaustion also restore normal speed
before center recovery; restoration failure disables rather than issuing a
high-speed center command.

Focused tests cover target increments and safe truncation, exact fast control,
motion-gated camera changes, immediate reversal, async speed setup, hit speed
restoration/recentering, and the two-second-hold transition. A real-model
offline zone check at the standard camera-search anchor accepted 13 consecutive
safe depths from 10 through 130 degrees and excluded the next interval. The
offline GUI check passed, all 138 repository tests passed, and original camera
Cartesian hashes remain unchanged. No CAN socket was opened and no arm moved.

Evidence: `diagnostics/right_camera_playback_strike_focused_tests.txt`,
`diagnostics/right_camera_playback_strike_ui_check.txt`,
`diagnostics/right_camera_playback_strike_real_zone_check.txt`,
`diagnostics/right_camera_playback_strike_static_audit.txt`,
`diagnostics/right_camera_playback_strike_original_unchanged.txt`, and
`diagnostics/right_camera_playback_strike_full_tests.txt`.

## Isolated right-J7 ten-degree test — 2026-09-19

Added `right_joint7_test`, `launch_right_joint7_test.py`, and
`start_right_joint7_test.sh` as a camera-free, recording-free right-arm test.
Its Start step uses the existing customized center (J1-J6 at zero and J7 at
+80.214091 degrees) and the existing -3 degree open-gripper target. Continue
commands +7 degrees and requires the gripper encoder to remain within 0.75
degree for 0.20 second before any J7 motion; a 3-second close timeout aborts.

The motion uses the same `StrikeControl` and 2.0 rad/s right-J7-only speed as the
camera-playback strike. It commands exactly one target at J7 -10 degrees from
the custom center, immediately commands the exact custom center on arrival,
restores the ordinary 0.4 rad/s speed, and disables all eight right motors.
Only right J7 changes between the two joint goals.

The real-model offline preflight sampled both directions at one-degree or finer
spacing and accepted the whole buffered-zone path. Its endpoints were:

- custom center TCP: `(0.17690550, -0.17009812, 0.20657841)` m;
- J7-minus-10-degree TCP: `(0.16889363, -0.17009790, 0.17622447)` m.

Nine focused tests cover the exact target, zone rejection, encoder-confirmed
gripper closure, no J7 motion before confirmation, close timeout, speed-before-
position ordering, immediate return, normal-speed restoration before disable,
and launcher isolation. The offline Tk check passed, the complete repository
suite passed all 147 tests, shell/Python static checks passed, and a timed
offline launch brought up the panel, RViz, and robot-state publisher and shut
them down cleanly. SHA-256 checks confirm that the original right-camera
Cartesian implementation remains unchanged. No CAN interface was opened and no
physical arm moved.

Evidence: `diagnostics/right_joint7_test_focused_tests.txt`,
`diagnostics/right_joint7_test_ui_check.txt`,
`diagnostics/right_joint7_test_full_tests.txt`,
`diagnostics/right_joint7_test_static_audit.txt`,
`diagnostics/right_joint7_test_offline_launch.txt`, and
`diagnostics/right_joint7_test_original_camera_unchanged.txt`.

## Playback visible-goal and guaranteed strike round trips — 2026-09-19

The recorded-path playback program now treats the complete visible pink target
rectangle as success. Its robust post-playback measurement starts the two-second
hold when the tip is anywhere in that rectangle. Cartesian correction begins
only when the robust endpoint measurement is outside, and it stops as soon as a
candidate enters the visible rectangle. The former smaller invisible acquisition
rectangle was removed from the playback runtime and panel text.

The strike state machine now guarantees a complete fast return to the preserved
pink-zone anchor after every depth. For an 80-degree J7 anchor, the tested state
sequence is `80→70→80`, then `80→60→80`, then `80→50→80`. The same rule continues
for later safe 10-degree depths. The previous hit callback could interrupt a
fast leg by immediately restoring normal speed and beginning center recovery.
It now latches the camera hit, allows the commanded depth and fast anchor return
to finish, and only then restores 0.4 rad/s and centers.

The 23 playback-focused tests cover visible-edge acquisition versus outside
correction, the two-second visible-zone hold, cumulative strike planning, exact
multi-attempt round-trip ordering, immediate reversal, and hit latching through
the fast return. The offline Tk integration passed and all 148 repository tests
passed. Static checks confirmed the explicit `80→70→80`, `80→60→80`, and
`80→50→80` sequence and found no playback-runtime inner-goal references. The
original `right_camera_cartesian` program and the standalone `right_joint7_test`
program retain their pre-change SHA-256 hashes. No CAN interface was opened and
no physical arm moved.

Evidence: `diagnostics/right_camera_playback_outer_roundtrip_focused_tests.txt`,
`diagnostics/right_camera_playback_outer_roundtrip_ui_check.txt`,
`diagnostics/right_camera_playback_outer_roundtrip_full_tests.txt`, and
`diagnostics/right_camera_playback_outer_roundtrip_static_audit.txt`.

## Learned 100 BPM cymbal striking — 2026-09-19

After a discovery strike produces the qualifying cymbal-box change, the
recorded-path program now remembers that attempt's cumulative J7 reduction. The
discovery attempt still completes its fast return to the preserved pink-zone
anchor. Instead of restoring normal speed and centering, the controller then
repeats that exact `anchor→learned depth→anchor` trajectory indefinitely.

Strike starts are scheduled 0.600 seconds apart, corresponding to 100 BPM. Each
leg retains the existing exact endpoint `StrikeControl`, 2.0 rad/s right-J7-only
speed, 2-degree reached threshold, stall monitor, timeout, and preflighted path.
The controller never overlaps strokes: a new outbound command is issued only
after the prior encoder-confirmed anchor return. If a physically deeper stroke
takes longer than 0.600 seconds at 2.0 rad/s, the next stroke begins immediately
after the return instead of interrupting that safety invariant.

The control panel adds **Stop 100 BPM Striking: Center + Relax**. It is disabled
until a hit depth is learned. A stop request during a stroke is latched until the
current fast return completes; a request while waiting at the anchor acts
immediately. Both paths restore J7 to 0.4 rad/s, return to the customized center,
and relax. After learning, the loop no longer depends on the camera process and
has no automatic time or strike-count limit. Encoder, zone, motor, Emergency
Relax, and program-close safety paths remain active.

The 27 playback-focused tests cover the 0.600-second schedule, learned-depth
retention, camera independence, full fast return before repetition, stop during
an active stroke, speed restoration, centering, and the previous visible-zone
and discovery-round-trip behavior. The offline Tk integration passed and all
152 repository tests passed. Static checks confirmed 100 BPM, the Stop policy,
and unchanged SHA-256 hashes for the original `right_camera_cartesian` and
standalone `right_joint7_test` programs. No CAN interface was opened and no
physical arm moved.

Evidence: `diagnostics/right_camera_playback_100bpm_focused_tests.txt`,
`diagnostics/right_camera_playback_100bpm_ui_check.txt`,
`diagnostics/right_camera_playback_100bpm_full_tests.txt`, and
`diagnostics/right_camera_playback_100bpm_static_audit.txt`.

## Right-arm recording video editor — 2026-09-20

Added a separate offline editor for JSON files created by the right-arm motion
recorder. The launcher starts only the editor panel, robot-state publisher, and
RViz on isolated ROS domain 92; a static audit found no CAN observer, transport,
or motor-controller references in the editor runtime or launcher.

The editor requires an explicit selected file (or `--recording`), validates the
right-arm v1 schema and current model identity, interpolates visual-only playback,
and renders a video-style timeline. Its two sample-aligned handles shade the
areas deleted from the start and end. Dragging either handle pauses playback and
publishes that exact retained boundary pose to RViz. Save requires confirmation
and atomically replaces the selected path; it rebases sample time, advances the
UTC recording start, updates duration/count, and creates no second JSON or backup.

Eleven recorder/editor focused tests passed, including exact in-place path use,
time/timestamp rebasing, interpolation, validation, and a one-sample crop. The
Tk integration simulated both trim-handle drags, checked both preview boundary
states, saved a crop, and verified that the selected JSON remained the only file.
All 157 repository tests passed. A bounded offline launch loaded
`recordings/record1.json`, opened the editor, RViz, and robot-state publisher,
and all three processes shut down cleanly after the timeout. The panel and RViz
were visually inspected from screenshots. No CAN interface was opened and no
physical arm moved.

Evidence: `diagnostics/right_recording_editor_focused_tests.txt`,
`diagnostics/right_recording_editor_ui_check.txt`,
`diagnostics/right_recording_editor_static_audit.txt`,
`diagnostics/right_recording_editor_full_tests.txt`,
`diagnostics/right_recording_editor_offline_launch.txt`,
`diagnostics/right_recording_editor_panel.png`, and
`diagnostics/right_recording_editor_rviz.png`.

## Playback five-degree strike discovery increments — 2026-09-20

Changed only the recorded-path camera playback strike discovery sequence from
cumulative 10-degree J7 reductions to cumulative 5-degree reductions. It now
tries complete fast round trips at `anchor−5°`, `anchor−10°`, `anchor−15°`, and
so on. Planning still stops before the first unsafe interval or J7 limit, every
attempt still returns fully to the preserved pink-zone anchor, and a detected
depth is still repeated at 100 BPM until Stop. The separate standalone
`right_joint7_test` remains its original single ten-degree test.

The shared `STRIKE_INCREMENT_DEGREES = 5` constant now drives planner radians,
UI attempt depths, hit messages, and learned-depth reporting. On the existing
real model/zone at the standard camera-search anchor, the offline preflight
accepted 26 consecutive depths from 5 through 130 degrees and rejected the next
interval. All 27 playback-focused tests and all 157 repository tests passed;
the offline playback UI check also passed. No camera, CAN interface, or physical
arm movement was used.

Evidence: `diagnostics/right_camera_playback_5degree_focused_tests.txt`,
`diagnostics/right_camera_playback_5degree_ui_check.txt`,
`diagnostics/right_camera_playback_5degree_real_zone_check.txt`,
`diagnostics/right_camera_playback_5degree_static_audit.txt`, and
`diagnostics/right_camera_playback_5degree_full_tests.txt`.

## Playback single-frame acceptance and immediate striking — 2026-09-20

Changed only the recorded-path camera playback sequence after its final recorded
pose settles. The next fresh simultaneous cymbal/direct-tip frame is now checked
once against the larger visible pink rectangle. An inside result immediately
establishes the strike anchor and baseline; there is no two-second hold. An
outside result starts the existing playback-informed Cartesian hill climber,
which now stops only at the restored invisible centered target occupying 50% of
the visible rectangle's width and height. One new larger-pink confirmation then
starts striking, again without a hold; an outside confirmation resumes finding.

The old asynchronous all-depth center-return strike preflight and its second
camera-availability gate were removed. After the accepted frame, the controller
synchronously derives only the consecutive 5-degree J7 paths that remain in
right zone1, switches J7 to the existing strike speed, and commands attempt one.
At the standard real-model anchor, the offline calculation retained all 26
attempts from 5 through 130 degrees and completed in about 0.121 seconds. The
accepted frame supplies the cymbal-box baseline used by the unchanged
consecutive-frame greater-than-4-pixel hit test.

All 26 playback-focused tests and all 156 repository tests passed. The offline
Tk integration passed, and static checks confirmed that no playback hold state,
hold timer, or post-acceptance camera recheck remains. No camera, CAN interface,
or physical arm movement was used.

Evidence: `diagnostics/right_camera_playback_single_check_focused_tests.txt`,
`diagnostics/right_camera_playback_single_check_ui_check.txt`,
`diagnostics/right_camera_playback_single_check_plan_timing.txt`,
`diagnostics/right_camera_playback_single_check_static_audit.txt`, and
`diagnostics/right_camera_playback_single_check_full_tests.txt`.

## ST7/eMeet single-hit camera playback — 2026-09-25

Replaced camera-box motion as the recorded-path program's strike-success signal
with the existing `/home/jason/Proyectos3/st7` neural cymbal-onset detector. The
launcher now starts a supervised ST7 bridge, selects the stable eMeet M0
PipeWire capture source, runs the selected `cymbal-fmn-tcn/best.pt` checkpoint,
and sends readiness heartbeats plus timestamped `HIT` events to the ROS/Tk
controller over a separate local datagram socket. Start and Continue remain
disabled until the detector has opened the input and completed a two-second
warm-up. A detector exit or stale heartbeat during the workflow uses the
existing safe failure recovery.

The cumulative strike search remains complete 5-degree round trips:
`anchor→−5°→anchor`, then `anchor→−10°→anchor`, then
`anchor→−15°→anchor`, and so on. After each return the anchor is held for up to
1.25 seconds so ST7 lookahead can finalize the attempt. The bridge preserves
estimated sound-onset timestamps, preventing a delayed event from one attempt
from being assigned to the next. A qualifying sound hit is latched until the
fast anchor return finishes; J7 then returns to normal speed, the arm centers,
and the right motors relax. The former learned-depth 100 BPM loop and its Stop
button were removed, so exactly one sound-confirmed hit ends the sequence.
Camera-box motion remains only a clearly labeled preview diagnostic.

The host audio audit found the connected source
`alsa_input.usb-eMeet_Tech_eMeet_M0_000000000012-00.mono-fallback` as the
current default, and sounddevice exposed `eMeet M0: USB Audio` plus the
PipeWire input. A live bridge check opened the eMeet route at 16 kHz, captured
2.2 seconds without overflow, and delivered the ready heartbeat. This verified
the microphone/bridge path but did not create a real cymbal `HIT` event.

The 34 focused playback tests cover audio protocol validation, stable eMeet
selection, ST7 asset selection, attempt timestamp/motion gating, delayed sound
decisions, 5-degree round trips, one-hit center/relax behavior, sound-process
failure recovery, and absence of repeating-strike runtime state. The offline Tk
integration passed. The complete repository suite passed all 165 tests. Static
compilation/audits found no 100 BPM, continuous-strike, or camera-hit decision
logic in the playback runtime. A bounded launch without `--hardware` started the
control panel, processed camera, RViz, robot-state publisher, and live ST7
listener together; it was then interrupted normally. The camera's temporary
test recording was removed after the check. No CAN interface was opened and no
physical arm moved.

Evidence: `diagnostics/right_camera_playback_st7_focused_tests.txt`,
`diagnostics/right_camera_playback_st7_ui_check.txt`,
`diagnostics/right_camera_playback_st7_audio_check.txt`,
`diagnostics/right_camera_playback_st7_static_audit.txt`, and
`diagnostics/right_camera_playback_st7_full_tests.txt`, plus
`diagnostics/right_camera_playback_st7_offline_launch.txt`.

## ST7 normality-score tuning — 2026-09-25

Updated the recorded-path launcher to use the scoring-enabled ST7 checkout at
`/home/jason/st7`, outside `Proyectos3`. Startup now requires both
`runs/cymbal-fmn-tcn/best.pt` and
`runs/cymbal-normality/reference.npz`. The audio bridge refuses to report ready
unless ST7's listening status says its normality scorer is enabled, and its
version-2 local protocol requires a finite 0–100 `normality_score` on every
forwarded hit.

The original sound-onset discovery remains complete 5-degree anchor round
trips. The first hit fixes depth `D`. Scores strictly greater than 80 finish the
current return and then center and relax. Scores at or below 80 start one fixed,
safe-zone-checked neighborhood in the exact order `D+2°`, `D+4°`, `D−2°`,
`D−4°`. The window is never shifted beyond `D±4°`. Missing hits advance after
the existing 1.25-second delayed-decision wait. The best scored depth is
remembered and retried once if the bounded candidates are exhausted; failure to
exceed 80 then uses the normal center-before-relax recovery. No accepted hit is
repeated and no 100 BPM state was reintroduced.

The 40 focused camera-playback tests passed. They include the exact
`10° → 12°, 14°, 8°, 6°` ordering, ±4-degree bound, two-degree step, strict
`>80` threshold, safe-candidate filtering, best-depth retry, timestamp gating,
full anchor returns, accepted-hit center/relax behavior, outside-ST7 asset path,
and mandatory normality protocol field. The complete repository suite passed
all 171 tests, the updated offline Tk check passed, and compilation/static
audits passed. The external ST7 suite passed all 9 tests; file-mode inference
on `r32.wav` emitted normality scores 90.8, 96.0, and 85.2.

A live bridge-only check selected the connected eMeet source
`alsa_input.usb-eMeet_Tech_eMeet_M0_000000000012-00.mono-fallback`, opened it
through PipeWire at 16 kHz, received ST7's
`normality: {enabled: true, version: 1}` status, and reached the ready heartbeat
after capturing 2.4 seconds without overflow. A bounded full launcher run
without `--hardware` also started the control panel, camera, RViz, robot-state
publisher, and scoring-enabled `/home/jason/st7` listener together. The camera
recording created by that bounded diagnostic was deleted afterward. No CAN
interface was opened and no physical arm moved.

Evidence: `diagnostics/right_camera_playback_normality_focused_tests.txt`,
`diagnostics/right_camera_playback_normality_full_tests.txt`,
`diagnostics/right_camera_playback_normality_ui_check.txt`,
`diagnostics/right_camera_playback_normality_static_audit.txt`,
`diagnostics/right_camera_playback_normality_st7_tests.txt`,
`diagnostics/right_camera_playback_normality_st7_file_check.txt`,
`diagnostics/right_camera_playback_normality_live_audio_check.txt`, and
`diagnostics/right_camera_playback_normality_offline_launch.txt`.

## TONOR default microphone for camera playback — 2026-09-26

Changed the recorded-path camera playback sound input from the eMeet M0 to the
connected TONOR TD510. The launcher and audio bridge now default to
`alsa_input.usb-TONOR_TONOR_TD510_Dynamic_Mic_0000KT5a300000135-00.analog-stereo`,
fall back to the sole non-monitor source containing `TONOR` if that stable name
changes, and report TONOR consistently in the control panel and process status.
The detector checkpoint, normality artifact, thresholds, HIT protocol, and arm
behavior were not changed.

The host source list and default source both identified that exact TONOR input.
A live bridge-only check selected it, opened it through PipeWire at 16 kHz mono,
loaded normality version 1, reached the ready heartbeat, and stopped normally
after capturing 2.2 seconds. No recording was saved. Shell/Python compilation,
launcher help, static stale-name checks, and the offline Tk integration passed.
The launcher retains `/home/jason/st7` as its preferred detector location but
now falls back to the available `/home/jason/Proyectos3/st7` sibling checkout
when the original location is absent. All 40 focused playback tests and all 171
repository tests passed. No camera, CAN interface, or physical arm was used.

## Highest-normality continuous 100 BPM playback — 2026-09-26

Changed only the recorded-path `camera_playback` workflow after its first ST7
sound detection. A first hit at depth `D` is now retained as the baseline, then
every safe value in the fixed order `D−2°`, `D−4°`, `D+2°`, `D+4°` is
tested as a complete anchor round trip. Missing-hit candidates advance after
the existing delayed-decision wait. The window remains fixed, and after all
candidates the scored depth with the highest normality is selected; there is no
early threshold acceptance or final retry.

The selected exact J7 target then repeats as anchor→target→anchor strokes with
`STRIKE_BPM = 100.0` and a 0.600-second outbound-command period. Strokes never
overlap: an encoder-confirmed return is required, and a slow stroke delays the
next beat rather than issuing an unsafe overlapping target. During the loop,
the panel button reads **STOP 100 BPM STRIKING: CENTER + RELAX**. A press while
moving latches Stop, completes the active fast return, restores the ordinary
0.4 rad/s J7 speed, centers, and relaxes; a press while waiting begins that
center sequence immediately. Emergency Relax and existing stall, zone, motor,
and program-close safeguards remain active.

Offline tests verified the requested `10° → 8°, 6°, 12°, 14°` example,
selection of 8° when it has the highest score, exact 0.600-second scheduling,
fast down/return speed ordering, full return before another stroke, and safe
Stop/center/relax behavior. All 47 focused playback tests and all 178 repository
tests passed; the offline Tk UI and shell/Python compilation checks passed.
No CAN interface, camera, or physical arm was used.

## First-hit one-degree 100 BPM striking — 2026-09-26

This supersedes the preceding highest-normality tuning behavior for the current
recorded-path `camera_playback` workflow. Strike discovery now begins at 5
degrees and increases J7 depth by exactly 1 degree per complete anchor round
trip: 5°, 6°, 7°, and so on through the consecutive safe targets. A no-hit
attempt advances by only 1 degree after the existing delayed-decision wait.

The first timestamp-valid ST7 `HIT` fixes the target. The active search stroke
still finishes its fast return to the preserved anchor for safety, then that
exact detected target begins repeating at 100 BPM. There is no fine-tuning
state, neighborhood search, score comparison, or movement above or below the
first detected depth. ST7 may continue to carry its normality field for protocol
compatibility, but `camera_playback/app.py` does not read it when accepting or
selecting a strike depth.

The existing non-overlapping 100 BPM loop and **STOP 100 BPM STRIKING: CENTER +
RELAX** behavior remain intact: Stop finishes an active fast return, restores
normal J7 speed, centers, and relaxes. Emergency Relax and the existing joint,
safe-zone, stall, and timeout checks remain active.

All 43 focused camera-playback tests and all 174 repository tests passed. The
offline Tk UI check passed, Python compilation and shell syntax checks passed,
and a static audit confirmed `STRIKE_START_DEGREES = 5`,
`STRIKE_INCREMENT_DEGREES = 1`, and no remaining fine-tuning symbols in the
runtime strike code. No CAN interface, camera, microphone, or physical arm was
used for this verification.

Evidence: `diagnostics/right_camera_playback_one_degree_focused_tests.txt`,
`diagnostics/right_camera_playback_one_degree_full_tests.txt`,
`diagnostics/right_camera_playback_one_degree_ui_check.txt`, and
`diagnostics/right_camera_playback_one_degree_static_audit.txt`.

## ESP32 hi-hat every-other-beat integration — 2026-09-27

Copied the proven classic-ESP32 `motor_beat` firmware from
`/home/jason/Proyectos3/esp-main` into `esp32_hihat/motor_beat/`; the source and
copy have the same SHA-256
`cc6df2ab7359b26f210ca01bdbf170c648086e71bbd8c58f21b597c8adcb1db8`.
The copied firmware retains motor 2's 110-degree close/hold target, zero-degree
return/release behavior, half-speed return, encoder control, and 400 ms serial
watchdog. An offline Arduino compile for `esp32:esp32:esp32` succeeded.

Added `camera_playback/hihat.py` and connected it directly to the existing arm
beat transition. The launcher defaults to the currently connected CP2102's
stable `/dev/serial/by-id/` path and uses the proven 115200 baud configuration.
It verifies the expected `motor_beat` firmware response before enabling Start,
sends `H` every 100 ms, and sends exactly one motor 2 command with each new arm
outbound beat: `C`, `O`, `C`, `O`, and so on. Thus right J7 strikes every beat
while motor 2 closes to 110 degrees every other beat and returns to zero on the
intervening beats. The integrated runtime never sends motor 1's `K` command.

The ordinary Stop control sends `O` immediately so motor 2 returns to zero while
the arm completes its active return, centers, and relaxes. Emergency Relax and
application shutdown send `S` to release the firmware outputs. A missing port,
wrong/unresponsive firmware, serial write failure, or firmware fault prevents
Start or stops continuous arm playback through the existing safe return and
center path.

The current device was confirmed query-only as the Silicon Labs CP2102 at
`/dev/ttyUSB0`, reachable by the configured stable by-id link, with user
read/write access. The real port was not opened and no serial motor command was
sent during verification. Pseudo-terminal tests covered 115200-baud setup,
firmware readiness, the exact `C/O/C/O` sequence, 110-degree constant, 100 ms
heartbeat, stop/open behavior, Emergency `S`, missing-port handling, and fault
latching. All 50 focused ESP/playback tests and all 181 repository tests passed;
the offline Tk UI, launcher help, Python compilation, shell syntax, firmware
compile, static command audit, and byte-for-byte firmware comparison passed.
No CAN interface, camera, microphone, physical arm, or ESP32 motor was operated.

Evidence: `diagnostics/right_camera_playback_esp_hihat_focused_tests.txt`,
`diagnostics/right_camera_playback_esp_hihat_full_tests.txt`,
`diagnostics/right_camera_playback_esp_hihat_ui_check.txt`,
`diagnostics/right_camera_playback_esp_hihat_launcher_help.txt`,
`diagnostics/right_camera_playback_esp_hihat_firmware_compile.txt`, and
`diagnostics/right_camera_playback_esp_hihat_static_audit.txt`.

## Pure-simulation 5-degree playback test mode — 2026-09-27

Added a mutually exclusive `--test` mode to
`start_beat.sh`. It cannot construct the real `Motors` class,
does not open SocketCAN, and cannot command the physical arm or gripper. It
shares the normal playback application and safety state machine through an
in-memory `SimulatedMotors` substitute, while the launcher omits the Y2 camera
process, ST7/TONOR audio process, and ESP32 hi-hat connection.

Test mode simulates recording preflight, right-arm centering, gripper loading,
recorded playback, joint-state feedback, zone enforcement, exact strike
control, and Stop/Center/Relax in RViz. After the simulated recording endpoint
settles, it assumes the endpoint is correctly positioned, validates the normal
first 5-degree strike path, skips camera alignment and all hit-search attempts,
and starts the same non-overlapping straight quarter-note 100 BPM loop at
exactly 5 degrees. It does not issue any hi-hat command or introduce swing.

The Tk integration check explicitly replaces the real CAN constructor with an
exception and confirms that `--test` instead creates `SimulatedMotors`, which
has no socket member. Launcher composition is also inspected without launching
its actions to confirm that test mode contains no camera or audio child process.
That integration then completes simulated center, gripper close, recorded
playback, two fixed 5-degree 100 BPM strikes, Stop, recenter, and relax. All 55
focused playback/hi-hat tests and all 186 repository tests passed, along with
launcher help/mutual exclusion, Python compilation, shell syntax, and static
launch audits. No CAN interface, camera, microphone, ESP32 serial port, or
physical arm was used during these checks.

Evidence: `diagnostics/right_camera_playback_test_mode_focused_tests.txt`,
`diagnostics/right_camera_playback_test_mode_full_tests.txt`,
`diagnostics/right_camera_playback_test_mode_ui_check.txt`,
`diagnostics/right_camera_playback_test_mode_launcher_help.txt`, and
`diagnostics/right_camera_playback_test_mode_static_audit.txt`.

## Triplet swing ride rhythm — 2026-09-27

Changed only the continuous ride rhythm after first-hit depth selection. The
selected J7 depth and the test-mode 5-degree depth now use the same
triplet-based 100 BPM swing scheduler. The first ride command is the extra
pickup before beat 1. The sequence then plays beat 1, beat 2, the third-triplet
extra after beat 2, beat 3, beat 4, and the third-triplet extra after beat 4.
Starting at the pickup, its intended command gaps are 0.200, 0.600, 0.400,
0.200, 0.600, 0.400, and 0.200 seconds before repeating.

The ESP32 hi-hat behavior remains on the four main quarter-note beats. The
pickup and ride extras do not advance or command the hi-hat; beats 1 through 4
retain the existing `C`, `O`, `C`, `O` close/open sequence. Strike discovery,
the chosen degree value, speeds, complete-return safety, Stop/Center/Relax, and
all camera/audio behavior are unchanged. In particular, the return is not
interrupted to force a short swing interval: when a full outbound-and-return
stroke exceeds its 0.200, 0.400, or 0.600-second gap, the next ride event waits
for the encoder-confirmed return.

All 57 focused camera-playback/hi-hat tests and all 188 repository tests passed.
The pure-simulation Tk integration completed center, recording playback, the
5-degree swing pickup and beat 1, Stop, recenter, and relax while explicitly
forbidding construction of the real CAN motor class. Static timing and hi-hat
audits, in-memory Python compilation, and shell syntax checks also passed. No
CAN interface, camera, microphone, ESP32 serial port, or physical arm was used.

Evidence: `diagnostics/right_camera_playback_swing_focused_tests.txt`,
`diagnostics/right_camera_playback_swing_full_tests.txt`,
`diagnostics/right_camera_playback_swing_test_mode_ui_check.txt`, and
`diagnostics/right_camera_playback_swing_static_audit.txt`.

## Equal 3.5 rad/s cymbal-strike return speed — 2026-09-27

Changed the recorded-path `camera_playback` J7 return limit from 2.0 rad/s to
3.5 rad/s, matching its existing 3.5 rad/s downstroke limit. This applies both
to the 5°, 6°, 7°… discovery round trips and to every continuous 100 BPM
swing stroke. Target positions, swing scheduling, full-return-before-next-hit
safety, hi-hat timing, and Stop/Center/Relax behavior are unchanged.

The separate standalone `right_joint7_test` remains at its original 2.0 rad/s;
its constant and direct-CAN allowlist were kept distinct so this playback-only
speed change does not alter the neighboring test program.

All 81 focused camera-playback, hi-hat, motor-command, and standalone-J7 tests
passed, as did all 188 repository tests. The pure-simulation Tk sequence and
static equal-speed/allowlist audits passed. No CAN interface, camera,
microphone, ESP32 serial port, or physical arm was used.

Evidence: `diagnostics/right_camera_playback_equal_strike_speed_focused_tests.txt`,
`diagnostics/right_camera_playback_equal_strike_speed_full_tests.txt`,
`diagnostics/right_camera_playback_equal_strike_speed_ui_check.txt`, and
`diagnostics/right_camera_playback_equal_strike_speed_static_audit.txt`.

## Two-rad/s interruptible swing returns — 2026-09-27

This supersedes the immediately preceding equal-speed change. Restored the
recorded-path cymbal return limit to 2.0 rad/s while retaining the 3.5 rad/s
downstroke. Discovery attempts remain complete anchor round trips. During the
continuous swing loop only, a return may now be reversed before reaching the
anchor when the following strike deadline is too close.

The opening pickup establishes the musical grid when its strike controller
reports the target reached. Every later ride event is stored as a target-arrival
deadline. On each 20 ms control update during a return or anchor wait, the
program reads live J7 feedback and estimates the outbound requirement as the
current distance from the fixed strike target divided by 3.5 rad/s, plus a
40 ms command/control margin. It switches from the 2.0 rad/s return to the
3.5 rad/s downstroke when that estimate reaches the remaining deadline time.
The scheduled grid is retained rather than being shifted by a late stroke.

An interrupted return requires at least 4° of measured rebound. This is twice
the strike controller's 2° reached tolerance, ensuring that the new outbound
controller cannot immediately declare success without moving. The partial path
is strictly within the same target-to-anchor segment already validated for the
full stroke. Longer 0.400 and 0.600-second swing gaps may still complete the
full return. Stop disables further reversals, completes the current target and
full 2.0 rad/s return, restores ordinary speed, centers, and relaxes.

Because J7 may start its downstroke before the beat deadline, ESP32 hi-hat
commands were separated from the outbound transition. They remain `C`, `O`,
`C`, `O` on main quarter-note deadlines only; the pickup and ride extras still
do not advance the hi-hat.

All 83 focused playback, hi-hat, motor-command, and standalone-J7 tests passed,
as did all 190 repository tests. The pure-simulation Tk sequence completed
center, recording, swing, Stop, recenter, and relax while the real CAN
constructor was explicitly forbidden. Static timing, speed, rebound, hi-hat,
Python-compilation, and shell checks passed. No CAN interface, camera,
microphone, ESP32 serial port, or physical arm was operated, so physical impact
timing and the 40 ms margin still require live observation.

Evidence: `diagnostics/right_camera_playback_interruptible_return_focused_tests.txt`,
`diagnostics/right_camera_playback_interruptible_return_full_tests.txt`,
`diagnostics/right_camera_playback_interruptible_return_ui_check.txt`, and
`diagnostics/right_camera_playback_interruptible_return_static_audit.txt`.

## Manual powered strike checkpoints and `start_beat.sh` rename — 2026-09-28

Renamed the recorded-path shell entry point from
`start_right_camera_playback.sh` to `start_beat.sh` and added a third mutually
exclusive mode, `--hardwaretest`. The new mode launches the same real CAN arm,
processed camera, TONOR/ST7 detector, and ESP32 paths as `--hardware`; it does
not reuse the in-memory `--test` motor substitute.

After normal centering, recording playback, and visible-pink acceptance,
`--hardwaretest` plans the same safe 5°, 6°, 7°… J7 targets but holds the
accepted anchor without issuing a strike command. A dedicated button displays
**Attempt 5° hit** and authorizes exactly one full anchor-to-target-to-anchor
attempt. ST7 detection and its timestamp validation remain automatic. A no-hit
decision leaves the arm holding the returned anchor and changes the button to
the next degree. A detected hit also finishes the return, then changes the
button to **Continue to swing beat**; only that click starts the existing exact
detected-depth swing/hi-hat loop. The ordinary `--hardware` automatic search
and `--test` pure-simulation flow retain their previous branches.

All 60 focused camera-playback tests and all 197 repository tests passed. The
regular offline playback UI, new hardware-test checkpoint UI, and complete
pure-simulation UI flow passed. Launcher help, mutual-exclusion checks, Python
compilation, shell syntax, renamed-entry-point checks, and static launch/mode
audits also passed. These were offline checks only: no CAN interface, physical
arm, camera, microphone, or ESP32 serial device was opened or operated.

Evidence: `diagnostics/right_camera_playback_hardwaretest_focused_tests.txt`,
`diagnostics/right_camera_playback_hardwaretest_full_tests.txt`,
`diagnostics/right_camera_playback_hardwaretest_ui_checks.txt`, and
`diagnostics/right_camera_playback_hardwaretest_static_audit.txt`.

## Editable one-shot `--hardwaretest` strikes with continuous ST7 log — 2026-09-28

This supersedes the preceding `--hardwaretest` checkpoint workflow. The normal
`--hardware` automatic 5°/6°/7° search and the pure-simulation `--test` flow
remain unchanged. `--hardwaretest` still performs powered centering, recorded
playback, and camera alignment, but no longer runs automatic depth discovery or
enters the continuous swing loop.

After visible-pink acceptance, the panel exposes an editable J7 strike-degree
field. Decimal amounts are accepted. Every press revalidates the exact entered
target against the J7 limit and the complete buffered safe-zone path, selects
the ordinary 0.4 rad/s J7 speed before the position command, and performs
exactly one anchor-to-target-to-anchor round trip. On return it enables the
field and button again and does not issue another strike unless the user presses
the button. The smaller endpoint tolerance scales below the entered movement so
small valid entries cannot be declared reached at the anchor without moving.

ST7/TONOR remains required and active throughout the manual strike section.
Every incoming `HIT` updates a persistent GUI sound-log line and prints a
terminal log entry containing the count, score, normality metadata, and current
manual-strike phase. These events are informational only and do not trigger or
repeat motion. Because this mode has no swing/hi-hat sequence, it does not open
or require the ESP32 serial controller.

All 63 focused camera-playback tests and all 200 repository tests passed. The
ordinary offline playback UI, editable hardware-test UI and sound-log check,
and complete pure-simulation UI flow passed. Launcher help, mutual exclusion,
Python compilation, shell syntax, one-shot-state static checks, normal-speed
command ordering, and stale-workflow audits passed. These were offline checks
only: no CAN interface, physical arm, camera, microphone, or ESP32 serial device
was opened or operated.

Evidence:
`diagnostics/right_camera_playback_hardwaretest_manual_focused_tests.txt`,
`diagnostics/right_camera_playback_hardwaretest_manual_full_tests.txt`,
`diagnostics/right_camera_playback_hardwaretest_manual_ui_checks.txt`, and
`diagnostics/right_camera_playback_hardwaretest_manual_static_audit.txt`.

## `--hardwaretest` ESP32-free preflight regression — 2026-09-28

Fixed the powered hardware-test preflight crash reported as
`AttributeError: 'NoneType' object has no attribute 'tick'`. Hardware-test mode
intentionally does not construct an ESP32 hi-hat controller, so its delayed
recording preflight now refreshes camera and ST7 state but calls `hihat.tick()`
only when a controller exists. Normal `--hardware` still performs the same
hi-hat refresh, and `--test` remains fully offline.

Regression tests exercise both branches directly: hardware-test preflight with
`hihat=None` proceeds to the shared arm-start transition, while normal hardware
preflight still ticks its controller. All 65 focused camera-playback tests and
all 202 repository tests passed, along with Python/shell syntax checks and the
offline hardware-test UI check. No CAN frame, motor command, camera, microphone,
or ESP32 serial operation was used during verification.

Evidence:
`diagnostics/right_camera_playback_hardwaretest_hihat_preflight_focused_tests.txt`,
`diagnostics/right_camera_playback_hardwaretest_hihat_preflight_full_tests.txt`,
and
`diagnostics/right_camera_playback_hardwaretest_hihat_preflight_ui_check.txt`.

## `--hardwaretest` strike speeds matched to `--hardware` — 2026-09-28

Changed each user-authorized one-shot hardware-test strike to use the same J7
leg speeds as normal hardware striking: 3.5 rad/s from the pink-zone anchor to
the entered strike target and 2.0 rad/s back to the anchor. The controller sets
the applicable speed before each exact position command, scales the stage
timeout using that leg's speed, and restores the ordinary 0.4 rad/s J7 setting
after the completed return before waiting for another button press. The manual
one-shot rule and motion-independent ST7 logging remain unchanged.

Regression tests verify both speed-command/target-command orderings against the
same shared constants used by `--hardware`, verify ordinary-speed restoration
after the return, and preserve the normal-hardware and pure-simulation paths.
All 65 focused camera-playback tests and all 202 repository tests passed, along
with compilation, shell syntax, and the offline hardware-test UI check. No CAN
frame, motor command, camera, microphone, or ESP32 serial operation was used.

Evidence:
`diagnostics/right_camera_playback_hardwaretest_matched_speeds_focused_tests.txt`,
`diagnostics/right_camera_playback_hardwaretest_matched_speeds_full_tests.txt`,
and
`diagnostics/right_camera_playback_hardwaretest_matched_speeds_ui_check.txt`.

## `start_beat.sh` single-frame pink-zone hill goal — 2026-09-29

The recorded-playback camera controller used by `start_beat.sh` no longer has
the smaller invisible inner rectangle as its hill-climber completion goal.
After the recording endpoint is outside the visible pink rectangle, robust
outside-zone samples still provide the smooth center-distance score used to
choose safe Cartesian moves. At each settled hill pose, however, any one fresh
directly observed stick-tip frame inside the complete visible pink rectangle
immediately establishes the strike anchor and continues the selected hardware
workflow. There is no timed hold, smaller target, or second confirmation.

This change is confined to the separate `camera_playback` controller launched
by `start_beat.sh`; the original `camera_search` program and its independent
target-hold behavior remain unchanged. Regression coverage checks visible-edge
acceptance during hill measurement, immediate transition into automatic strike
planning, and continued outside-frame measurement. All 65 focused playback
tests and all 202 repository tests passed. Python compilation, shell syntax,
launcher help, stale inner-target runtime-text checks, and original-controller
isolation checks also passed. These checks were offline only: no CAN interface,
physical arm, camera, microphone, or ESP32 serial device was opened or operated.

Evidence:
`diagnostics/right_camera_playback_pink_single_frame_focused_tests.txt`,
`diagnostics/right_camera_playback_pink_single_frame_full_tests.txt`, and
`diagnostics/right_camera_playback_pink_single_frame_static_audit.txt`.

## Faster playback-strike J7 reversal — 2026-09-29

The `camera_playback` strike path launched by `start_beat.sh` now uses 3.5 rad/s
for both the downward and return legs. Because the two legs share one speed,
the reversal retains the already-selected firmware limit instead of inserting
another speed-setting frame. The separate standalone `right_joint7_test`
retains its original 2.0 rad/s speed.

Active playback strike legs temporarily prioritize right-J7 state queries at
the 20 ms GUI control-loop limit, or 50 Hz. The other 15 motor/gripper channels
retain their existing 20 Hz request rate, and J7 returns to 20 Hz while waiting
between strikes. This changes the steady state-query count from 320 to 350
requests per second during an active leg, a 9.375% increase, rather than raising
all 16 channels to 50 Hz.

Every manual hardware-test, automatic discovery, and continuous-swing return
now sends one motor-7 anchor-position frame immediately. It no longer sends
seven J1–J7 position frames with J7 last. Regression tests also verify that an
outbound-to-return transition at the common 3.5 rad/s speed does not issue a
redundant speed frame, that only right J7 receives the priority query, and that
ordinary motor feedback rates remain unchanged.

All 82 focused centering/camera-playback tests and all 204 repository tests
passed. The pure-simulation complete UI sequence and the offline editable
hardware-test UI check passed. Python compilation, shell syntax, launcher help,
speed/isolation checks, query-load checks, and all three J7-only return paths
passed static validation. These were offline checks only: no CAN interface,
physical arm, camera, microphone, or ESP32 serial device was opened or
operated. The physical reduction in cymbal-contact dwell therefore remains to
be confirmed with a controlled one-shot hardware test.

Evidence:
`diagnostics/right_camera_playback_fast_return_focused_tests.txt`,
`diagnostics/right_camera_playback_fast_return_full_tests.txt`,
`diagnostics/right_camera_playback_fast_return_test_mode_ui_check.txt`,
`diagnostics/right_camera_playback_fast_return_hardwaretest_ui_check.txt`, and
`diagnostics/right_camera_playback_fast_return_static_audit.txt`.
## 2026-09-30: restored `--hardwaretest` playback/alignment with manual MIT strikes

- Restored the recording selector, center/open/load/Continue/close workflow,
  recorded playback, processed-camera alignment, and single-frame visible-pink
  acceptance in `--hardwaretest`.
- Kept the relative-degree manual J7 MIT freefall/rebound control at the accepted
  pink-zone pose. ST7 runs as an informational logger; the ESP32 remains omitted.
- Added hardware-test-only fault containment: a named right-drive fault disables
  only that drive and holds the others; generic non-stall failures lock motion
  with powered holds; `STALL: motor N` still immediately relaxes the whole right
  arm. The MIT return now applies the existing 0.5-second stall criteria to J7.
- Offline verification only; no CAN socket was opened and no physical motor was
  commanded:
  - 86 focused camera-playback tests passed.
  - 234 complete repository tests passed.
  - The offline Tk hardware-test UI check passed.
  - Python compilation, `start_beat.sh` shell syntax, launcher help, and static
    contract checks passed.
- Evidence:
  - `diagnostics/right_camera_playback_hardwaretest_restored_mit_focused_tests.txt`
  - `diagnostics/right_camera_playback_hardwaretest_restored_mit_full_tests.txt`
  - `diagnostics/right_camera_playback_hardwaretest_restored_mit_ui_check.txt`
  - `diagnostics/right_camera_playback_hardwaretest_restored_mit_launcher_help.txt`
  - `diagnostics/right_camera_playback_hardwaretest_restored_mit_static_audit.txt`
