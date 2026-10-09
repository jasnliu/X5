# Left/right recording video editor

This offline program previews and crops JSON recordings made by
`start_recording.sh`. It never opens CAN or commands either physical arm.

## Start and select a file

```bash
cd /home/jason/Proyectos3/X5
./start_recording_editor.sh
```

The chooser starts in the X5 folder so both `recordings/` (right) and
`left_recordings/` (left) are available. You can also open either arm directly:

```bash
./start_recording_editor.sh --recording recordings/record1.json
./start_recording_editor.sh --recording left_recordings/left_motion_example.json
```

The editor validates the arm-specific schema, joint order, gripper name, model
hash, units, sample count, timing, and numeric values. RViz previews the arm
specified in the file.

## Preview, crop, and save

Play or scrub the timeline, or drag the sample-aligned START and END handles.
RViz immediately previews the retained boundary pose. **Reset crop** restores
the currently loaded full file.

**SAVE CROP — OVERWRITE SELECTED JSON** asks for confirmation and atomically
replaces that exact file. There is no Save As operation or automatic backup.
Times are rebased, metadata is updated, and all arm-specific metadata is
preserved.

Key files:

- `start_recording_editor.sh`: user launcher, isolated ROS domain 92.
- `launch_recording_editor.py`: offline RViz/editor launch.
- `motion_recording/editor_app.py`: left/right preview UI.
- `motion_recording/editor.py`: validation, cropping, and atomic replacement.
- `config/recording_editor.rviz`: displays either arm's preview.
