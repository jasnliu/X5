# Left- and right-gripper Cartesian goals

These are arm-specific input layers over the same measured-joint goal program.
The original left joint-angle program remains available through `start_goal.sh`.
The left Cartesian program uses `left_zones/zone1.json`; the right Cartesian
program uses `right_zones/zone1.json`.

Run exactly one Cartesian program at a time:

```bash
cd /home/jason/Proyectos3/X5
./start_left_cartesian.sh --hardware
# or:
./start_right_cartesian.sh --hardware
```

`start_cartesian_goal.sh` remains an alias for the left program.

Enter X, Y, and Z in **meters**. For each arm, `(0, 0, 0)` remains the modeled
gripper TCP position when all seven joints are zero. The fields are offsets in
the RViz/world axes; positive Z is upward.

The left program keeps its original **RUN CARTESIAN GOAL → HOLD 2 s → CENTER →
RELAX** workflow and never controls either gripper. The right program uses this
right-only workflow:

1. **Start** centers J1–J6 at 0° and J7 at +80.214° while opening the custom
   right gripper to its displayed encoder value of −3°.
2. When both the arm and gripper reach that loading pose, arm motion fully
   pauses and **Continue** becomes available. The right arm and gripper remain
   powered at their targets so the user can load the object.
3. **Continue** commands the right gripper to +7°. The close command is given
   0.75 seconds for this small 10° movement before arm motion resumes.
4. The arm moves to the Cartesian goal and holds there indefinitely with the
   gripper closed. The right-only **End: Return to Center → Relax** button becomes
   available after the measured goal has settled. At that point, editing X/Y/Z
   to a different valid point enables **Update: Move to Changed Goal**. Update is
   disabled while moving, before the first goal is reached, when the values are
   unchanged, and when the preview is outside zone1.
5. **Update** solves near the currently held posture while the arm continues to
   hold still. It checks the new endpoint, current-to-new transition, and center
   returns before commanding movement. When the updated goal settles, the arm
   holds there again and another changed coordinate can be submitted.
6. **End** returns the arm to its special right center while continuing to hold
   the +7° gripper target. Only after recentering are all eight right motors
   disabled. Once Continue has been pressed, no open command is sent before that
   relaxation.

The left center remains all seven joints at zero. The right center is J1–J6 at
zero and J7 at its positive URDF limit, `+1.4 rad` (`+80.214°`). The right arm's
motor direction is reversed, so this is the safe folded position corresponding
to the motor-side negative direction. That right center does not redefine
Cartesian zero: the right program centers at J7 `+80.214°`, moves to the joint
solution for the requested point relative to the all-zero TCP, holds until End
is pressed, returns to the same right center, and only then relaxes.

The standalone right program's initial and Update IK keep right J2 at its
centered `0°` value and use the other six joints for the requested TCP position.
Live testing showed that this physical J2 stayed near center for small negative
targets even while the other six joints settled, which left a persistent Y
error. Position-only IK has enough redundancy to remove that untrackable J2
target without changing the requested point. The left Cartesian solver and the
separate camera Cartesian program are not changed by this standalone-right
selection; joint control, correction, settling, and zone/path validation remain
the existing shared implementations.

The RViz goal-preview sphere updates immediately as any X/Y/Z field changes;
you do not need to click Run. An accepted zone location is magenta, a point
outside zone1 is red, and incomplete or nonnumeric input removes the preview
until all three fields are valid again. The text panel shows the entered offset
and `INSIDE zone1` or `OUTSIDE zone1`. Previewing performs no IK and sends no
motor command. Reachability and the complete path check still occur only after
Run is clicked.

Before any motor setup, the program:

1. Rejects missing, nonnumeric, NaN, or infinite values.
2. Adds the requested offset to that arm's all-zero TCP world position.
3. Rejects the point with `ERROR: Requested coordinate is outside zone1` if it
   is outside zone1's existing 5 mm classification tolerance.
4. Solves position-only inverse kinematics within the URDF joint limits.
5. Prefers a solution near that arm's configured center and rejects a solution
   if its nominal 0.4 rad/s center-to-goal-and-back path leaves zone1.
6. Waits for fresh encoders again after IK before enabling the selected arm.

The resulting seven joint angles are passed to exactly the same joint controller
used by the tested angle-goal program. All seven encoders must remain within 3
degrees of their computed targets for 0.6 seconds before the goal is declared
reached. This completion rule is identical for the standalone left and right
Cartesian programs. The **left** Cartesian program then holds for 2 continuous seconds and
automatically recenters. The **right** Cartesian program instead keeps correcting
and holding the goal until Update or End is pressed. Update performs its IK and
path checks in a background worker so live feedback, correction, and gripper
holding continue; it then moves to and holds the changed goal. The update solver
first uses the current held posture, checks the complete update trajectory, and
checks complete center-return trajectories at seven bounded points including
both ends. This replaces the old quadratic every-sample return check that could
take roughly 30 seconds. End recenters.
Each program disables its selected motors only after recentering. The left
program keeps the right arm and both grippers relaxed. The right program keeps
the left arm and left gripper relaxed while controlling right motors 1–8. Arm-joint stall,
zone-recovery, other fault, arm isolation, and Emergency Relax behavior remain
active. Intentional right-gripper contact is not treated as an arm-joint stall;
Emergency Relax and any arm fault disable the right gripper with the arm.

The encoder panel at the bottom shows joints 1–7 plus **Gripper (motor 8)** for
the selected arm. In the left program that gripper value is display-only. In the
right program it provides the degree convention used by the −3° and +7° custom
gripper targets.

The IK controls gripper **position only**; gripper orientation is free. The final
point is Cartesian, but the motion between center and the point is the firmware's
joint-space motion, not a guaranteed straight Cartesian line. RViz shows the
requested target as a magenta sphere and the live TCP as the existing green/red
sphere.

The current URDF still defines the standard `openarmx_right_hand_tcp`. The
physical tip extension of the custom right gripper is not modeled because no
tool-offset dimensions were provided. The right zone recorder and Cartesian
program therefore consistently track the same modeled TCP, not a newly measured
point at the end of the longer custom tool.

## Right-center zone verification

Offline analysis of `right_zones/zone1.json` confirms that the corrected
J7 `+80.214°` right center is inside the original visible hull. The example
`(0,0,+0.050 m)` target is also inside, and inverse kinematics found a valid
center-to-goal path that remains inside the buffered zone.

The all-zero TCP used to define Cartesian `(0,0,0)` is still 44.083 mm outside
the original hull. This is intentional: it remains only the coordinate-system
origin and is not used as the right arm's physical center/return pose. Entering
`(0,0,0)` as a goal is therefore rejected by the existing zone check.

## Left-arm physical upward test

The requested test was only `(0, 0, +0.050 m)`, five centimeters straight above
the centered TCP. The requested point was 28.622 mm inside the original visible
zone hull. IK produced approximately:

```text
J1 +23.703°, J2 +0.007°, J3 +0.004°, J4 +23.262°,
J5 −0.003°, J6 +0.034°, J7 −33.686°
```

The physical goal-transition encoders produced a relative TCP of
`(−0.001884, −0.000091, +0.049855) m`, which is 1.891 mm from the requested
point. It then recentered and relaxed. All 845 samples stayed within the 5 mm
buffer; maximum signed distance beyond the visible hull was 3.905 mm. Because
this program converts one endpoint to joint angles, the joint-space trajectory
had up to 23.424 mm of lateral deviation even though the endpoint requested only
positive Z.

An X=2.000 m input was also tested while the arm was disabled. The interface
displayed the outside-zone error and did not start motor setup. A post-run
query-only audit confirmed all 16 motors disabled.

Evidence: `diagnostics/cartesian_up_live_encoder.csv`,
`diagnostics/cartesian_up_analysis.txt`, `diagnostics/cartesian_up_complete.png`,
`diagnostics/cartesian_outside_rejected.png`,
`diagnostics/cartesian_goal_offline_tests.txt`,
`diagnostics/cartesian_up_plan.txt`, and
`diagnostics/cartesian_up_post_relax_query.json`.

## Two-second hold test

The shared goal controller was updated to enter `HOLDING GOAL` before
recentering. A new physical run used only the same `(0, 0, +0.050 m)` target.
The recorded transition from `HOLDING GOAL` to `RECENTERING` took 2.038 seconds
(the small excess is recorder/tick timing). During all 125 hold samples, maximum
joint error was 0.090 degrees and maximum Cartesian error was 1.294 mm. At the
return transition, the measured TCP was 0.119 mm from the requested endpoint.

The complete 1,010-sample trace had no buffered-zone violations; its maximum
signed distance beyond the original visible hull was 2.667 mm, below the 5 mm
background tolerance. The right-arm maximum encoder span was 0.066 degrees. The
arm recentered, relaxed, and a separate 60-batch query-only audit confirmed all
16 motors disabled.

Evidence: `diagnostics/cartesian_hold_live_encoder.csv`,
`diagnostics/cartesian_hold_live_analysis.txt`,
`diagnostics/cartesian_hold_preflight_query.json`,
`diagnostics/cartesian_hold_post_relax_query.json`, and
`diagnostics/cartesian_hold_offline_tests.txt`.

## Live preview verification

The offline/no-CAN viewer was used to edit the fields without clicking Run.
Changing the offset to `(0.020, -0.010, 0.030)` moved RViz marker 5 immediately
to the centered TCP plus that offset and kept it magenta. Changing X to 2.000 m
moved the marker and changed it to red/`OUTSIDE zone1`. Replacing X with invalid
text published a marker deletion, and restoring `(0, 0, 0.050)` restored the
magenta preview. The application remained `Offline preview — no CAN` throughout.

Evidence: `diagnostics/cartesian_live_preview_offline_test.txt`.
