"""Headless Tk integration check for video-style trim preview and in-place save."""
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
from unittest.mock import patch

import rclpy

from motion_recording.editor_app import App
from motion_recording.recording import (
    GRIPPER_NAME, JOINT_NAMES, MotionRecording, gripper_name, joint_names,
)
from safe_zone.geometry import Model


ROOT = Path(__file__).resolve().parents[1]


def make_recording(path: Path, side: str = "right") -> None:
    model = Model(ROOT / "model/openarmx.urdf")
    names = joint_names(side)
    gripper = gripper_name(side)
    recording = MotionRecording(model.digest, side)
    recording.start("2026-09-20T12:00:00.000Z")
    for index in range(4):
        state = {name: .01 * index * (joint + 1)
                 for joint, name in enumerate(names)}
        state[gripper] = .020 + index * .001
        recording.add(10.0 + index, state, [.1 * index, .05 * index, .02 * index])
    recording.stop()
    recording.save(path)


rclpy.init(args=[])
try:
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "selected.json"
        make_recording(source)
        app = App(source, show_initial_dialog=False)
        app.root.withdraw()
        app.timeline.configure(width=800)
        app.root.update_idletasks()

        # Simulate dragging the front handle to one second. The preview must be
        # the retained start boundary, not the original start state.
        app.dragging_handle = "start"
        app.timeline_drag(SimpleNamespace(x=app._time_to_x(1.0)))
        assert app.trim_start_index == 1
        assert app.playhead_s == 1.0
        assert abs(app.preview.positions_rad[0] - .01) < 1e-9

        # Simulate dragging the back handle to two seconds (deleting one second
        # from this 3-second take), and confirm that boundary is previewed too.
        app.dragging_handle = "end"
        app.timeline_drag(SimpleNamespace(x=app._time_to_x(2.0)))
        assert app.trim_end_index == 2
        assert app.playhead_s == 2.0
        assert abs(app.preview.positions_rad[0] - .02) < 1e-9
        assert str(app.save_button.cget("state")) == "normal"

        before = {item.resolve() for item in Path(directory).iterdir()}
        with patch("motion_recording.editor_app.messagebox.askyesno", return_value=True), \
                patch("motion_recording.editor_app.messagebox.showinfo"):
            app.save_crop()
        after = {item.resolve() for item in Path(directory).iterdir()}
        assert before == after == {source.resolve()}
        payload = json.loads(source.read_text())
        assert payload["sample_count"] == 2
        assert payload["duration_s"] == 1.0
        assert [sample["time_s"] for sample in payload["samples"]] == [0.0, 1.0]
        assert not app.dirty
        assert "overwritten" in app.status_text.get()

        left_source = Path(directory) / "left_selected.json"
        make_recording(left_source, "left")
        assert app.load_recording(left_source, show_error=False)
        assert app.clip.arm == "left"
        assert app.clip.joint_names == joint_names("left")
        assert "LEFT recording editor" in app.root.title()

        app.closed = True
        app.root.destroy()
        app.node.destroy_node()
finally:
    rclpy.shutdown()

print("PASS: trim handles preserve right JSON and the same editor loads left recordings; no CAN")
