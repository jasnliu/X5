"""Headless offline Tk check for the left-arm recorder path; CAN is forbidden."""
import argparse
import json
from pathlib import Path
import tempfile
import time
from unittest.mock import patch

import rclpy

from safe_zone.gripper_feedback import EncoderState, motor8_feedback
from motion_recording.app import App
from motion_recording.recording import SCHEMAS, gripper_name, joint_names


rclpy.init(args=[])
try:
    with patch("motion_recording.app.SingleArmObserver",
               side_effect=AssertionError("CAN forbidden in offline UI check")):
        app = App(argparse.Namespace(
            hardware=False, arm="left", can="can1", right_can="can0", left_can="can1"
        ))
        app.root.withdraw()
        assert app.side == "left"
        assert app.recordings_dir.name == "left_recordings"
        assert app.joint_names == joint_names("left")

        # Synthetic feedback proves the arm-specific state reaches left JSON
        # without opening either CAN bus.
        app.args.hardware = True
        state = {name: .01 * (index + 1)
                 for index, name in enumerate(joint_names("left"))}
        state[gripper_name("left")] = .021
        state = EncoderState(state, motor8_feedback={'left': motor8_feedback(35774)})
        app.latest = state.copy()
        app.latest_time = time.monotonic()
        app.start_recording()
        app.offer(("sample", time.monotonic() + .01, state.copy()))
        app.tick()
        app.stop_recording()
        app.update_panel()
        assert 'raw count 35774 / 65535' in app.angles.get()
        assert 'mm' not in app.angles.get()

        with tempfile.TemporaryDirectory() as directory:
            app.recordings_dir = Path(directory) / "left_recordings"
            app.recordings_dir.mkdir()
            with patch("motion_recording.app.filedialog.asksaveasfilename",
                       return_value=str(Path(directory) / "outside" / "left_take")), \
                    patch("motion_recording.app.messagebox.showinfo"):
                app.save_recording()
            payload = json.loads((app.recordings_dir / "left_take.json").read_text())
            assert payload["samples"][0]["motor8_encoder_count"] == 35774
            assert payload["samples"][0]["motor8_raw_rad"] == motor8_feedback(35774)["raw_rad"]
            assert payload["schema"] == SCHEMAS["left"]
            assert payload["arm"] == "left"
            assert payload["joint_order"] == list(joint_names("left"))
            assert payload["gripper_name"] == gripper_name("left")

        app.root.destroy()
        app.node.destroy_node()
finally:
    rclpy.shutdown()

print("PASS: offline left recorder UI writes left-only JSON to the forced left folder; no CAN")
