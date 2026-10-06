# Right-arm camera Cartesian hill climber

This is the modified camera program; the original `start_right_cartesian.sh`
remains unchanged. It launches three GUI windows: RViz, one right-arm control
panel, and the processed Y2 detection window. It does not open Y2's raw-camera
comparison window.

```bash
cd /home/jason/Proyectos3/X5
./start_right_camera_cartesian.sh --hardware
```

Close other arm-control and camera programs first. `--hardware` is required for
physical movement. Without it, the control panel is an explicitly labeled
no-CAN preview. The physical hill-climber has **not** been run as part of the
implementation verification; only offline/unit geometry and state-machine tests
were run.

## Camera target and start gate

The strongest class-0 cymbal box is divided into equal thirds in each direction.
The center ninth is highlighted in magenta as `CYMBAL TARGET`. The camera sends
that cymbal box and target rectangle, plus the strongest class-1 drumstick box
and its directly observed YOLO-pose tip, to the control process.

The Start button is disabled and `start()` refuses to initiate motor setup unless
a fresh processed frame contains a valid cymbal box. Cymbal presence is checked
again before Continue and after the gripper finishes closing.

## Sequence

1. With the motors disabled, preflight the fixed initial search.
2. Center the right arm at J1-J6 = 0 and J7 = +1.4 rad, using the original
   0.4 rad/s speed, while opening the right gripper to -3 degrees.
3. Wait indefinitely for the user to load the drumstick and press Continue.
4. Close the gripper to +7 degrees and keep resending that closed target.
5. Move to `(X,Y,Z) = (0.250, 0.000, 0.350)` meters. The search IK holds joint 2
   at its centered value and uses the other six joints.
6. If no fresh class-1 drumstick plus its directly observed YOLO tip appears,
   keep X and Z fixed and increase Y by 0.010 m through the preflighted points.
   With the current zone, Y=0.000 through Y=0.190 are accepted; Y=0.200 is the
   first rejected point. Exhaustion returns to center and then relaxes.
7. A qualifying detection is latched even if the tip appears only briefly. The
   current arm stage finishes settling before alignment begins; detection no
   longer causes an immediate return to center.

## Deterministic hill climber

At each settled pose, the program holds still and collects fresh, simultaneous
cymbal-plus-observed-tip samples for 0.5 seconds. It takes component-wise medians
of the tip, cymbal box, and target box. The score is the image-plane distance
from the median tip to the center target rectangle, normalized by the full
cymbal-box width and height. A tip inside the rectangle has score zero.

Candidate directions always use this order:

1. `+X`
2. `-X`
3. `+Y`
4. `-Y`
5. `+Z`
6. `-Z`

The first step size is 0.010 m. An improving candidate becomes the new anchor,
and the program continues in the same direction. A non-improving candidate
returns to the previous anchor before the next direction is tried. After a full
six-direction pass with no improvement, the step becomes 0.005 m and then
0.0025 m. A full non-improving pass at 0.0025 m reports failure, centers, and
relaxes.

Every candidate is solved while holding joint 2 centered. Before movement, the
program checks that the candidate lies inside the buffered right zone, is
reachable, and that the candidate transition, reverse anchor return, and
emergency center returns remain inside the zone. Unsafe or unreachable
candidates are skipped without moving. SciPy candidate planning runs on a
separate worker with independent model/zone objects so the GUI can continue
polling CAN feedback and camera status while the arm holds its anchor.

The customized center intentionally places J7 at its positive URDF limit. Live
encoder quantization can report that joint a few counts beyond the exact limit;
passing that raw value to SciPy previously caused an immediate `x0 is
infeasible` rejection before any candidate move was calculated. The planner
now accepts at most three encoder counts of such discrepancy and clips only
SciPy's initial numerical seed slightly inside the URDF bounds. The original,
unmodified measured joints are still used for transition and return-path safety
checks. Larger limit violations remain rejected rather than being hidden.

## Intermittent camera detections and success

Missing-tip frames are ignored: no box-center or tracked-tip fallback is
invented. During measurement the arm simply keeps holding until a direct tip
returns. Continuous loss of a simultaneous cymbal and observed tip for 30
seconds aborts the alignment, centers, and relaxes.

When the median tip enters the center cell, the arm holds that pose. Missing-tip
frames do not cancel the hold or cause movement. A fresh valid outside-tip frame
resets the hold and remeasures the current pose. After at least two wall-clock
seconds, a new fresh valid inside-tip frame is required to confirm success. The
arm then returns to the customized center and relaxes. The gripper stays closed
through alignment, the two-second hold, and recentering.

## Fault behavior

All program failures and non-stall faults command the customized center first
and disable the eight right motors only after the center controller reports that
the center was reached. If a center command temporarily fails, the motors remain
enabled with center recovery pending rather than being dropped at an arbitrary
pose. A detected motor stall is the sole automatic immediate-relax exception.
The red **Emergency Relax** button remains an intentional manual immediate
disable.

The launcher also reports camera, RViz, or robot-state-publisher exits to the
control process instead of immediately tearing down the controller. If the arm
is powered, that report uses the same center-before-relax recovery. Closing the
control process itself shuts down the remaining windows.

Camera options retain Y2 defaults and can be overridden, for example:

```bash
./start_right_camera_cartesian.sh --hardware --camera 1 --conf 0.50
```

The camera runs inside `/home/jason/Proyectos3/Y2/.venv`; ROS/RViz and arm
control use the ROS system Python. The environments exchange compact local
Unix-datagram messages only. Offline verification is recorded in
`diagnostics/camera_hill_climber_offline_tests.txt` and
`diagnostics/camera_hill_candidate_safety.txt`.
