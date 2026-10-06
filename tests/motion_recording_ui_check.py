"""Offline Tk integration for the right-arm recorder; CAN is forbidden."""
import argparse
import json
from pathlib import Path
import tempfile
import time
from unittest.mock import patch

import rclpy

from motion_recording.app import App
from motion_recording.recording import GRIPPER_NAME, JOINT_NAMES, SCHEMA


rclpy.init(args=[])
try:
    with patch("motion_recording.app.SingleArmObserver",
               side_effect=AssertionError("CAN forbidden in offline UI check")):
        app = App(argparse.Namespace(hardware=False, right_can="can0"))
        app.root.withdraw()
        with patch("motion_recording.app.messagebox.showwarning") as warning:
            app.start_recording()
            assert warning.called and not app.recording.active

        # Inject synthetic right-only feedback after construction; no worker or
        # CAN socket exists in this check.
        app.args.hardware = True
        state = {name: .01 * (index + 1) for index, name in enumerate(JOINT_NAMES)}
        state[GRIPPER_NAME] = .021
        app.latest = state.copy()
        app.latest_time = time.monotonic()
        app.start_recording()
        assert app.recording.active
        app.offer(("sample", time.monotonic() + .01, state.copy()))
        app.tick()
        app.stop_recording()
        assert len(app.recording.samples) == 1

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            app.recordings_dir = root / "recordings"
            app.recordings_dir.mkdir()
            selected = root / "outside" / "manual_take"
            with patch("motion_recording.app.filedialog.asksaveasfilename",
                       return_value=str(selected)) as dialog, \
                    patch("motion_recording.app.messagebox.showinfo"):
                app.save_recording()
            saved = app.recordings_dir / "manual_take.json"
            assert Path(dialog.call_args.kwargs["initialdir"]) == app.recordings_dir
            assert saved.exists() and not selected.exists()
            payload = json.loads(saved.read_text())
            assert payload["schema"] == SCHEMA
            assert payload["arm"] == "right"
            assert payload["joint_order"] == list(JOINT_NAMES)
            assert payload["sample_count"] == 1
            assert not any("left" in name for name in payload["joint_order"])

        # A limit violation during a new recording must warn, abort, and erase
        # every sample from that take rather than leave a partially saveable file.
        with patch("motion_recording.app.messagebox.askyesno", return_value=True):
            app.start_recording()
        assert app.recording.active
        safe = state.copy()
        unsafe = state.copy()
        unsafe[JOINT_NAMES[6]] = app.upper[6] + .02
        app.offer(("sample", time.monotonic() + .02, safe))
        app.offer(("sample", time.monotonic() + .03, unsafe))
        with patch("motion_recording.app.messagebox.showwarning") as warning:
            app.tick()
        assert warning.called
        assert not app.recording.active
        assert app.recording.samples == []
        assert not app.recording.dirty
        assert app.saved_path is None
        assert "aborted and discarded" in app.status.get()
        assert str(app.save_button.cget("state")) == "disabled"

        # A real warning dialog can stay open long enough for the 50 Hz worker
        # to fill the inbox. Since the failed take has already been discarded,
        # that inactive backlog must collapse to the newest sample instead of
        # permanently stopping feedback and greying the Start button.
        recovered = state.copy()
        recovered[JOINT_NAMES[6]] = min(0.0, app.upper[6])
        for index in range(app.inbox.maxsize + 5):
            app.offer(("sample", time.monotonic() + .1 + index * .001, recovered.copy()))
        assert not app.stop_event.is_set()
        app.tick()
        assert app.error is None
        assert app.fresh()
        assert str(app.start_button.cget("state")) == "normal"
        app.start_recording()
        assert app.recording.active
        app.stop_recording()

        app.root.destroy()
        app.node.destroy_node()
finally:
    rclpy.shutdown()

print("PASS: offline UI, right-only JSON, limit abort/discard warning, forced folder; no CAN")
