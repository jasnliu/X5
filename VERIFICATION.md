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
