# Orbbec default-camera verification

Completed 2026-10-04 in `/home/jason/Proyectos3/X5`.

## Changed

All active camera entry points now use the shared selector in
`camera_search/device.py` and identity in `config/camera.json`: Orbbec Gemini 2,
serial AY6R463004K, USB 2bc5:0670, RGB interface 04, capture index 0 (not metadata).
The verified Linux device is currently `/dev/video4`; runtime selection does not
hardcode that number. Depth, IR, other cameras, and metadata nodes are excluded.
No fallback is allowed if the configured camera is missing. Numeric camera
overrides on the existing camera launchers are preserved.

This covers the shared camera process, its playback overlay, both camera ROS
launchers, and the standalone smooth-playback video recorder. Programs without a
camera, simulation modes, saved motion recordings, and arm-control logic remain
unchanged. No SDK installation, driver replacement, firmware update, or motor
test was performed. OrbbecViewer was initially running; it had closed before
capture testing began. It was not restarted, so the camera remains available.

## Physical camera evidence (no arm controller)

| Path tested | Result |
| --- | --- |
| Shared opener, system Python/OpenCV | 147 RGB frames in 5.03 s; 640x480 BGR, native MJPEG, reported 30 FPS |
| Actual `camera_search.camera.main()` in Y2 venv | 302 displayed frames, 185 processed detection messages, 10.04 s; successful saved video and clean exit |
| Actual `camera_playback.camera.main()` in Y2 venv | 301 displayed frames, 193 processed detection messages, 10.06 s; successful saved video and clean exit |
| Actual `smooth_playback.hardware.Video` only | 91 video frames spanning 3.00 s; no video-thread error |

The two processed tests ran the actual camera, existing lighting preprocessing,
pose model, preview overlays, and recording path. Only the exit key was automated
after ten seconds by `live_check.py`. CAN socket creation was explicitly forbidden
in these integration tests and the standalone video test; no arm was moved.
Snapshots were visually checked as RGB images. Detection accuracy and robot
alignment with the new viewpoint were **not** certified.

There are nonfatal existing OpenCV/Qt font-directory warnings. The Gemini does
not accept the optional V4L2 low-light FPS-priority control (EINVAL); the existing
warning path handles this, and observed streaming stayed about 30 FPS. No
exposure, projector, or laser setting was forced to address it.

## Offline verification and preservation

- **396 regression tests passed**, including eleven new selector/open/cleanup
  tests; physical CAN prohibited by the regression runner.
- Both shell-launcher `--help` outputs and direct playback-camera help advertise
  the new default and numeric overrides without starting hardware.
- Parser tests execute only each launcher's `parse_args`, not its ROS launch.
- `verification.json` records final file hashes, live results, and preserved
  recordings/motor-control hashes. Original sources are in `before/`, with
  `changes.diff` for the existing production files.
- Camera logs/videos/images and `standalone_video/` remain here. Historical
  playback/experiment evidence and source snapshots were not changed.

See `../../CAMERA_DEFAULT.md` for usage and device-selection details. Close any
other camera application before launching an X5 camera program.
