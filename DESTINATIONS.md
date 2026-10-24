# Capture arm destinations

```bash
cd /home/jason/Proyectos3/X5
./start_destination.sh --right --hardware
# Or:
./start_destination.sh --left --hardware
```

Two windows open: the same arm-specific RViz view as the motion recorder, and a
small capture panel. Motors must already be relaxed/disabled. This program sends
only encoder queries on the selected arm's bus; it cannot enable, move, hold,
open the gripper, or relax motors. The other arm is not queried or displayed.
No camera is needed. Close other programs owning the selected CAN bus. Run one
destination-capture session at a time; its ROS display uses isolated domain 94.

1. Manually support and position the selected arm at the desired destination.
2. When the panel says **READY**, click **Capture destination**. The latest fresh
   complete encoder batch is checked against the selected arm's planner joint
   limits, then immediately saved if in range. There is no two-second hold,
   recording period, editor, confirmation, or save dialog.
3. Move to another destination and click again. The panel shows the saved path,
   session count, and next filename. Closing either window closes both; no motor
   state is changed on startup, errors, or exit.

Files are automatically named:

- `destinations/right/destinationR1.json`, `destinationR2.json`, …
- `destinations/left/destinationL1.json`, `destinationL2.json`, …

Each arm has independent numbering. Restarting continues after the highest existing
number; gaps are not filled. Existing files are never overwritten, even during
concurrent saves. Deleting all files resets that arm's numbering to 1.

Feedback failures leave the button available and reconnect automatically. A click
without fresh feedback reports **Not saved**; it is not queued for later. Retry
after READY. An out-of-range joint also reports **Not saved** in red, naming the
arm/joint, measured angle and allowed range. No destination or temporary file is
written, the next filename and saved count stay unchanged, and the same button
can be retried after repositioning. For example, right J7 at 87.05° is rejected
because its maximum is 80.21°.

The seven selected-arm angles use the **default planner's limits in saved encoder
coordinates**: original URDF bounds minus the validated right-arm calibration
offsets; left-arm bounds are unchanged. Exact boundaries are accepted; no extra
encoder tolerance is added and no angles are clipped. This does not check the
opposite arm or change the custom gripper behavior. Existing files, including
previously saved out-of-range destinations, are not modified or deleted.
Capturing still does not require a zone/collision test: an in-range saved pose
is not a promise that a collision-free trajectory to it exists.

## File format and planner compatibility

Files reuse `openarmx-{right,left}-motion-recording-v1` with exactly **one sample**,
`time_s: 0`, `duration_s: 0`, and `capture_kind: "destination"`. They contain joint
order, all seven measured joint angles in the existing `encoder_to_joint` radians
convention, model hash, gripper opening and raw motor-8 feedback, UTC timestamp,
feedback age/batch duration, and model-derived original gripper TCP coordinates
(not the drumstick tip). Joint angles are the authoritative destination; TCP is
derived metadata. This is not a recording of the path taken to get there.

The existing `drum_planner.common.endpoint(path, side=...)` reads these files.
`./plan_drums.sh experiment --right` / `--left` now lists them in its Destination
dropdown. Choose a destination to start calculating; use **Refresh destinations**
to pick up new captures. This does not change Single arc/hardware mode goals.

## Offline preview / testing

Omit `--hardware` for RViz plus the panel without opening CAN. The zero-pose preview
is explicitly labeled offline and Capture cannot save it as a real destination.
`--output-root PATH` overrides the parent of the right/left folders for testing;
`--right-can` / `--left-can` match the existing recorder's interface overrides.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv-planner-experiment/bin/python \
  -m pytest tests/test_destination_capture.py

# Native capture-panel tests with fake encoders; requires a desktop display.
ROS_DOMAIN_ID=94 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=".:$PYTHONPATH" \
  /usr/bin/python3 tests/destination_capture_ui_check.py
```
