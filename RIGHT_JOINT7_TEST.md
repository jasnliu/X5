# Right-J7 ten-degree test

This is a separate right-arm-only test program. It does not use the camera or a
recording, and it does not change the existing Cartesian or camera-playback
programs.

## Run

Offline UI and safety preview (no CAN and no motion):

```bash
cd /home/jason/Proyectos3/X5
./start_right_joint7_test.sh
```

Physical right-arm test:

```bash
cd /home/jason/Proyectos3/X5
./start_right_joint7_test.sh --hardware
```

The hardware command causes intentional fast physical motion. Keep people and
objects out of the arm's path, stay at the emergency stop, and support/load the
gripper only while the panel says `WAITING FOR LOAD`.

## Sequence

1. **Start** enables and centers only right motors 1-7 at the customized right
   center: J1-J6 at 0 degrees and J7 at its +1.4 radian (+80.214 degree) model
   limit. Right motor 8 opens to -3 degrees.
2. The program confirms the open-gripper encoder, then stops at `WAITING FOR
   LOAD` and enables **Continue**.
3. **Continue** commands the right gripper to +7 degrees. J7 does not move until
   the gripper encoder remains within 0.75 degree of +7 degrees for 0.20 second.
   Failure to close within 3 seconds aborts and disables the right motors.
4. Right J7 changes from the custom center by exactly -10 degrees, then
   immediately returns to the same custom center. It uses the camera-playback
   strike speed and endpoint behavior: 2.0 rad/s, exact endpoint commands, and
   no correction overshoot.
5. The program restores J7's ordinary 0.4 rad/s setting and disables all eight
   right motors. The gripper remains commanded closed until that disable.

The left arm and left gripper are never enabled or commanded. They are observed
only so the shared transport can enforce that all non-selected motors remain
relaxed.

## Safety checks

Before Start can enable anything, the program samples both the custom-center to
J7-minus-10-degree path and its return at no more than one-degree intervals. It
requires every sample to remain within joint limits and the buffered
`right_zones/zone1.json` TCP envelope. The current model/zone passes this check:
the center TCP is approximately `(0.17691, -0.17010, 0.20658)` m and the lowered
TCP is approximately `(0.16889, -0.17010, 0.17622)` m.

Encoder loss, motor faults, a J7 stall/timeout, a right-zone breach, or a
gripper-close timeout aborts the test. If the fast speed was selected, the
program restores the normal J7 speed before ordinary recovery or disable. The
red **Emergency Relax** button and Escape key immediately disable the right
motors.
