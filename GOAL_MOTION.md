# Left-arm center → J1 +45° / J4 +80° / J7 −80° → hold → center → relax

Run:

```bash
cd /home/jason/X5
./start_goal.sh --hardware
```

Wait for `Ready` and `Zone: INSIDE`, then click
**RUN: CENTER → GOAL → HOLD 2 s → CENTER → RELAX**.
The program:

1. Centers left motors 1–7 at 0°.
2. Moves left motor 1 to +45°, motor 4 to +80°, and motor 7 to −80°;
   the other four remain at 0°.
3. Uses the motor firmware's 0.4 rad/s speed setting (double the previous 0.2 rad/s).
4. Uses measured encoder error to add up to 12° of temporary target correction.
5. Declares success after all seven left joints remain within 3° for 0.6 seconds.
6. Holds the goal for 2 continuous seconds while correction and safety checks
   remain active; leaving the 3° tolerance restarts that timer.
7. Automatically returns every left joint to 0° and verifies that centered
   position with the same tolerance and settling time.
8. Disables left motor torque only after the normal return-to-center completes.

`left_zones/zone1.json` is loaded automatically. Its original surface is drawn in RViz;
the existing invisible 5 mm membership tolerance is used. If the powered left
TCP leaves that volume, the goal is cancelled and the arm attempts to return to
center before disabling left IDs 1–8. During this special recovery, a stall,
timeout, encoder/CAN fault, motor fault, or temperature fault disables immediately.
A red/`OUTSIDE` indication after Relax is not another fault—the arm is already
disabled and can hang under gravity outside the zone.

Stall detection is deliberately lenient: if a joint is still more than 8° from
the active target and changes by less than 1° over about 2.5 seconds, all left
motors are disabled immediately without recentering. Each phase also has a
30-second timeout. Encoder/CAN faults, stale readings, an active right
motor/gripper, motor temperature >=65°C, and all non-zone faults also disable
immediately. A fault does not auto-restart.

**Emergency Relax, stall faults, and non-zone faults disable motor torque
immediately rather than commanding more movement. A zone breach uses the
center-then-relax recovery described above. Software disable cannot disconnect
electrical power.**
Keep physical cutoff available and support the arm when relaxing. The right bus
receives state queries only. The left gripper never receives enable or position
commands. The safety zone checks the left TCP, not all links or collision meshes.

## Physical verification

On 2026-09-14, the current J1 +45° / J4 +80° / J7 −80° goal completed
its full physical goal-and-return cycle at 0.4 rad/s. At the goal transition the
measured angles were:

```text
J1 45.025°, J2 0.011°, J3 0.011°, J4 80.016°,
J5 0.011°, J6 0.011°, J7 −79.994°
```

It then returned all left joints to within 0.253° of zero before automatically
reporting `Left motors disabled`. Button-to-relaxed time was about 10.8 seconds.
The independently recorded 1,445-sample ROS trace went through READY, SETTING UP,
CENTERING, MOVING TO GOAL, RECENTERING, RELAXING, and RELAXED. Every sampled left
TCP position was inside the 5 mm background tolerance; the maximum signed
distance beyond the original hull was 2.373 mm. The final target itself is inside
the original visible hull by 101.674 mm. The right arm remained disabled and its
maximum encoder span was 0.066°. After the viewer closed, a separate 60-batch
query-only audit confirmed all 16 motors disabled and captured only the expected
960 state requests.

Zone-breach recovery and stall-immediate-relax were exercised with offline fault
injection. The arm was not intentionally driven outside zone1.

Evidence is in `diagnostics/goal_j1_45_live_encoder.csv`,
`diagnostics/goal_j1_45_ready.png`, `diagnostics/goal_j1_45_complete.png`,
`diagnostics/goal_j1_45_analysis.txt`, `diagnostics/goal_j1_45_path_check.txt`,
`diagnostics/goal_j1_45_final_tests.txt`, and
`diagnostics/goal_j1_45_post_relax_query.json`.
