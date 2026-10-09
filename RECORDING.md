# Query-only left/right motion recorder

This program records one manually guided OpenArmX arm while that arm remains
limp. It never centers, enables, disables, holds, or commands a motor.

## Start

Close other CAN programs, confirm the selected arm and gripper are disabled,
and physically support the arm throughout setup, recording, and shutdown.

```bash
cd /home/jason/Proyectos3/X5
./start_recording.sh --right --hardware
./start_recording.sh --left --hardware
```

Exactly one mode is required. `--right` queries right IDs 1–8 on `can0` and
`--left` queries left IDs 1–8 on `can1`. Override the bus only when needed with
`--right-can` or `--left-can`. The other CAN bus is never opened.

Omit `--hardware` for a no-CAN offline preview:

```bash
./start_recording.sh --right
./start_recording.sh --left
```

## Record and save

### Raw gripper feedback

Both arm panels show motor 8's **exact 16-bit encoder count**, its protocol-decoded
angle in radians, and the same angle in degrees. No width estimate, sign reversal,
zero offset, or clipping is applied to this readout. Missing raw feedback says
**unavailable**, never a fabricated zero.

New live recordings retain `motor8_encoder_count` and `motor8_raw_rad` for every
sample. The editor displays the nearest actual sample's raw reading (not an
interpolated estimate); old files without raw feedback say unavailable. Existing
files are not rewritten or recalibrated. The legacy `gripper_opening_m` field and
prismatic RViz finger coordinates remain for model/file compatibility only;
they are not presented as a measured physical opening. Raw motor angles must not
be written into a metre-valued URDF joint.

1. Wait until the selected motors 1–8 report disabled.
2. Put the supported limp arm at the desired starting pose.
3. Click **Start new recording**.
4. Guide the arm manually.
5. Click **Stop recording**.
6. Click **Save recording as JSON…**.

The destination is forced by arm:

- right: `/home/jason/Proyectos3/X5/recordings/`
- left: `/home/jason/Proyectos3/X5/left_recordings/`

Right files use `openarmx-right-motion-recording-v1`; left files use
`openarmx-left-motion-recording-v1`. Each file identifies its arm, joint order,
gripper, URDF hash, units, timestamps, samples, and encoder-derived TCP positions.
Saving uses an atomic replacement. Existing right recordings remain unchanged
and compatible with right-arm playback.

Every J1–J7 sample is checked against that arm's URDF limits. A violation beyond
the encoder tolerance warns, aborts, and discards the whole active take.

## Safety boundary

The hardware worker is query-only and accepts feedback only when all eight
selected-arm motors report disabled. It sends only state-request frames. An
unpowered arm can fall under gravity, so software is not a substitute for
physical support.

Key files:

- `start_recording.sh`: user launcher, isolated ROS domain 91.
- `launch_recording.py`: selects the matching left/right RViz layout.
- `motion_recording/app.py`: one-arm query/UI process.
- `motion_recording/recording.py`: arm-specific schemas and atomic saving.
- `recordings/`, `left_recordings/`: enforced right/left destinations.
