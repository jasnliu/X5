# start_beat camera-input selection

Implemented and verified 2026-10-04 in `/home/jason/Proyectos3/X5`.

## Available controls

- `--list-cameras`: prints connected capture-device names, current numeric
  indices, serials and preferred stable paths; excludes paired UVC metadata
  nodes. Exits before perception dependencies or any controller launch.
- `--select-camera`: interactive terminal selection before ROS actions/controllers
  are constructed. Enter a listed index/path, Enter for Orbbec, or `q` to cancel.
  Missing/invalid choices re-prompt. Noninteractive use errors with instructions
  to use `--camera` instead; no implicit fallback or launch on cancellation.
- `--camera SOURCE`: accepts the existing `orbbec` and numeric selectors, plus
  `/dev/videoN`, `/dev/v4l/by-id/...`, and `/dev/v4l/by-path/...` local paths.
  Missing or non-video targets error rather than silently selecting another input.
- Picker and explicit `--camera` are mutually exclusive. All choices apply in
  `--hardware`, `--hardwaretest`, and camera-using no-hardware preview. `--test`
  remains camera-free and rejects the interactive picker.

Selection is per launch; no camera is switched while an arm controller is active.
The configured Orbbec remains the shared default and retains its full-FOV checks,
including when selected through a device path. The picker prefers a stable path
for ordinary USB webcams and `orbbec` for the Gemini RGB interface. Other devices
keep their previous resolution behavior and existing `--width`/`--height` options.
Network/video-file inputs are deliberately not accepted as live control cameras.

## Current attached camera and live checks

The attached device was a Logitech Webcam C925e, serial A26952DF, capture index 0.
The paired `/dev/video1` metadata node was omitted from the list. Preferred path:

```
/dev/v4l/by-id/usb-046d_Logitech_Webcam_C925e_A26952DF-video-index0
```

- Ran actual `./start_beat.sh --list-cameras`; output is in `list.log`.
- Ran the actual `--hardwaretest --select-camera` startup prompt and entered `q`:
  it exited with `Camera selection cancelled; no controllers were launched`.
- Ran the picker alone in a real terminal and selected `0`; it returned the
  Logitech stable path and exited successfully (`picker_positive.json`).
- Ran the actual `--select-camera` launcher with no interactive stdin; it
  refused launch and explained how to select explicitly (`no_terminal.log`).
- Ran the actual playback-camera pipeline with `--camera 0`: 301 displayed
  frames and 158 inference-result messages over 10.13 seconds, clean exit.
- Repeated using the stable path: 301 displayed frames and 142 inference-result
  messages over 10.09 seconds, clean exit.

Both live captures were 640x480 at approximately 30 display FPS, using the
Logitech's existing default-resolution behavior. Previews were visually checked;
the actual Y2 model, preprocessing, overlays, local messages and saved videos all
ran. Only the quit key was automated by `live_check.py`. This verifies camera
input selection, not robot alignment accuracy or maximum FOV of this different
camera model. Existing nonfatal OpenCV Qt font warnings were retained.

No arm movement was performed. Live pipeline tests prohibited CAN sockets and
did not launch the robot-control app. Both hardware-mode command-construction
branches were exercised offline, not run through physical arm motion.

## Evidence and preservation

`unit.log` covers thirteen new tests: local-path parsing and failure handling,
device-list metadata filtering, RGB labeling, startup picker selection/re-prompt/
cancel, all camera-using launcher branches, and camera-free simulation isolation.
`regressions.log` confirms **all 415 repository regression tests passed**.

Original edited sources are in `before/`. `protected_before.json` records hashes
for the original recordings, motor/controller code, shared Orbbec configuration,
standalone playback video implementation, and `start_beat.sh`. Those files remain
unchanged. Only startup selection, shared local-device-path support, tests, and
documentation were added/updated. Historical recordings/evidence were preserved.
