# Query-only right-arm motion recorder

This program records a manually guided right-arm motion while the arm remains
limp. It opens two windows: an RViz encoder visualization and a separate
recording panel. It never centers the arm and the first and last pose may be
anywhere.

## Start

Close every arm controller and other CAN program first. Confirm that the right
arm and right gripper are already disabled, physically support the arm, then run:

```bash
cd /home/jason/Proyectos3/X5
./start_right_recording.sh --hardware
```

The recorder opens only `can0`, queries right IDs 1–8, and refuses feedback from
an enabled or faulted motor. It never sends enable, disable, configuration,
position, velocity, torque, zeroing, or gravity-compensation packets. Because it
does not disable anything itself, an already powered arm causes an error rather
than an automatic state change.

An unpowered arm can fall under gravity. Keep it supported throughout setup,
recording, and shutdown. The software provides no collision checking, safe-zone
restriction, or physical holding.

## Record and save

1. Wait for **READY — right motors 1–8 report disabled**.
2. Put the limp right arm at any desired starting pose.
3. Click **Start new recording**.
4. Manually guide and support the arm through the motion.
5. Click **Stop recording** at any desired ending pose.
6. Click **Save recording as JSON…**.

Every incoming J1–J7 encoder sample is checked against the current URDF joint
limits. If a joint moves beyond its limit by more than the three-encoder-count
measurement tolerance (approximately 0.066 degrees), the program immediately:

1. shows a **Joint limit exceeded** warning identifying the joint, measured
   angle, limit, and excess;
2. stops the active recording;
3. discards every sample from that recording; and
4. disables Save because there is no partial recording left to save.

Starting a recording is also blocked while a joint is already outside those
limits. This monitoring remains query-only and does not command or reposition
the limp arm.

Files are forced into:

```text
/home/jason/Proyectos3/X5/recordings/
```

Selecting another directory in the file dialog changes only the requested
basename; the program still writes inside `recordings/`. Saving uses a temporary
file, flushes it, and atomically replaces the destination. Closing with an
unsaved recording asks for confirmation.

JSON is appropriate for these recordings because it preserves ordered timing,
joint-name order, units, model identity, gripper state, TCP positions, and
format-version metadata in one portable file. CSV would be smaller but would
need a separate metadata convention. If recordings eventually become many
hours long, a streaming JSONL or binary format would be preferable; the current
recorder is capped at 100,000 samples, approximately 83 minutes at 20 Hz.

Each JSON file uses schema `openarmx-right-motion-recording-v1` and contains:

- seven right-arm joint positions in radians;
- right gripper opening in meters;
- encoder-derived right TCP position in meters;
- monotonically increasing time from the first sample in seconds;
- UTC start time, sample count, duration, nominal rate, and URDF SHA-256.

This program records only. To replay, scrub, and crop the JSON in an RViz
simulation without opening CAN, use the separate editor documented in
[RIGHT_RECORDING_EDITOR.md](RIGHT_RECORDING_EDITOR.md). Recorded files can also
be used by the separate powered program documented in
[RIGHT_CAMERA_PLAYBACK.md](RIGHT_CAMERA_PLAYBACK.md).

## Offline preview

```bash
./start_right_recording.sh
```

Offline mode opens the panel and RViz without opening CAN. It shows a zero-pose
illustration and disables recording. With hardware enabled, RViz follows only
the queried right arm. The recorder's RViz layout hides the left-arm links; the
left bus is not opened, queried, or recorded.

## Safety boundary and files

The only outgoing hardware frame is the installed-SDK-equivalent state request:

- extended arbitration ID `0x0200fd00 | motor_id`, right IDs 1–8;
- DLC 8, payload `01 00 00 00 00 00 00 00`.

The cooperative `can0` lock prevents this recorder from running alongside this
workspace's motion programs. An unrelated external program could ignore that
lock, so close other control software explicitly.

- `start_right_recording.sh`: user launcher, isolated ROS domain 91.
- `launch_right_recording.py`: starts only robot-state publisher, RViz, and the
  recorder panel.
- `motion_recording/app.py`: UI, read-only sampling, and live joint publication.
- `motion_recording/recording.py`: schema validation and atomic JSON saving.
- `safe_zone/encoder.py`: audited query packet, disabled-state decoder, and the
  single-right-bus observer.
- `config/right_recording.rviz`: RViz layout.
- `recordings/`: enforced destination for JSON recordings.
