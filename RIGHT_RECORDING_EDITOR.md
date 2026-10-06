# Right-arm recording video editor

This separate, offline program edits JSON files created by
`start_right_recording.sh`. It opens two windows:

- RViz, which previews the recorded right-arm posture; and
- a video-editor-style control window with playback, scrubbing, and start/end
  crop handles.

It never opens CAN and has no hardware mode. RViz is only a simulation. The
program does not enable, disable, hold, center, or command either physical arm.

## Start and select a file

```bash
cd /home/jason/Proyectos3/X5
./start_right_recording_editor.sh
```

A file chooser opens at startup in `recordings/`. Select the recording to edit.
If the chooser is cancelled, use **SELECT RECORDING JSON…** in the control
window. A file can also be opened directly:

```bash
./start_right_recording_editor.sh --recording recordings/record1.json
```

The editor accepts only compatible `openarmx-right-motion-recording-v1` JSON
files made for the current robot model. Invalid units, joint order, sample
counts, timing, numeric values, or model identity are rejected before editing.

## Preview and crop

- **Play/Pause** replays the retained green portion at its recorded timing.
- Click the green timeline to scrub like a video.
- Drag the **blue START** handle right to delete time from the beginning.
- Drag the **orange END** handle left to delete time from the end.
- During either drag, playback pauses and RViz immediately shows the exact
  sample-aligned pose at that retained crop boundary.
- **Crop start**, **Crop end**, Home, and End jump to the boundary poses.
- **Reset crop** restores the complete currently loaded file.

The timeline reports current/original time, seconds deleted at both ends,
retained duration, and retained sample count. Handles snap to real recorded
samples, so the saved trajectory always starts and ends on recorded states.

Example: on a 10-second file, placing START at 1.000 s and END at 8.000 s
deletes the first second and final two seconds, retaining the inclusive samples
from those two boundary poses.

## Save in place

Click **SAVE CROP — OVERWRITE SELECTED JSON** and confirm the warning. There is
no Save As dialog: the editor atomically replaces the exact JSON file selected
at startup. It does not create a new recording or backup file.

Saved sample times are rebased so the first retained sample is `0.0`,
`duration_s` and `sample_count` are updated, and `started_at_utc` is advanced by
the deleted start time. All other recording metadata remains compatible with
the existing playback program. After saving, the shortened file reloads as the
new full timeline.

Closing or selecting a different file while crop handles contain unsaved
changes asks for confirmation. Merely playing or scrubbing never changes the
JSON file.

## Files

- `start_right_recording_editor.sh`: user launcher on isolated ROS domain 92.
- `launch_right_recording_editor.py`: starts only robot-state publisher, RViz,
  and the editor panel.
- `motion_recording/editor_app.py`: Tk video-style editor and RViz publication.
- `motion_recording/editor.py`: strict loading, interpolation, cropping, and
  atomic in-place replacement.
- `config/right_recording.rviz`: the same right-only RViz presentation used by
  the recorder.
