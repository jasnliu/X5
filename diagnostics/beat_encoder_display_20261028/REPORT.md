# Both-arm beat encoder display — 2026-10-28

`start_beat.sh` already receives both arms' encoder snapshots. Its inherited
display showed right J1–J7 and both grippers, but omitted left J1–J7.

Minimal change: `camera_playback.app.App.encoder_text()` now appends left
J1–J7 in degrees using the existing `self.q` snapshot and the same one-decimal
format as the right block. Both motor-8 raw telemetry blocks remain intact.
Normal generated hardware and the retained playback modes inherit this display.
No extra queries, device owners, sign changes, planner changes, motion commands,
audio-model changes, or controller changes were introduced.

## Fresh verification

- 122 software tests passed: `test_beat_encoder_display.py`,
  `test_raw_gripper_feedback.py`, `test_camera_playback.py`,
  `test_generated_beat.py` (ROS/workspace sourced).
- Native Tk `tests.beat_encoder_display_ui_check` passed using an existing saved
  mission and explicit fake motors. CAN construction and ESP32 connections were
  prohibited. No camera, microphone, serial, or physical motor tests occurred.
- Both seven-joint rows and both raw gripper values appeared in the real label;
  a fake left-feedback update changed only the left block. No simulated movement
  command was issued while viewing/testing the display (Start was never pressed).
- Full label fits in the 788×968 panel on the 2560×1440 test display; Emergency
  Hold and Emergency Relax remain accessible. `panel.png` visually inspected.
- `ui_report.json` contains the exact rendered values and verification results.

This verifies display routing and formatting, not current physical encoder
readings. Existing right-arm formatting and hardware acquisition remain unchanged.
