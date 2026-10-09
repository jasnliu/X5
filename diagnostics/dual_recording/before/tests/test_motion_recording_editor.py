"""Offline tests for loading, previewing, and overwriting cropped recordings."""
import json
import math
from pathlib import Path
import tempfile
import unittest

from motion_recording.editor import EditableRecording
from motion_recording.recording import (
    GRIPPER_NAME, JOINT_NAMES, MotionRecording, SCHEMAS, gripper_name, joint_names,
)


ROOT = Path(__file__).resolve().parents[1]


def state(value: float) -> dict:
    result = {name: value * (index + 1) for index, name in enumerate(JOINT_NAMES)}
    result[GRIPPER_NAME] = .020 + value / 100.0
    return result


def make_recording(path: Path, digest: str = "a" * 64) -> Path:
    recording = MotionRecording(digest)
    recording.start("2026-09-20T12:00:00.000Z")
    for index in range(4):
        recording.add(
            100.0 + index,
            state(index / 10.0),
            [index / 10.0, index / 20.0, index / 40.0],
        )
    recording.stop()
    recording.save(path)
    return path


class EditableRecordingTests(unittest.TestCase):
    def test_load_nearest_sample_and_smooth_preview(self):
        with tempfile.TemporaryDirectory() as directory:
            path = make_recording(Path(directory) / "take.json")
            clip = EditableRecording.load(path, "a" * 64)

        self.assertEqual(clip.sample_count, 4)
        self.assertEqual(clip.duration_s, 3.0)
        self.assertEqual(clip.nearest_index(.49), 0)
        self.assertEqual(clip.nearest_index(.51), 1)
        self.assertEqual(clip.nearest_index(2.9, maximum=2), 2)
        preview = clip.state_at(1.5)
        self.assertEqual(preview.time_s, 1.5)
        self.assertAlmostEqual(preview.positions_rad[0], .15)
        self.assertAlmostEqual(preview.positions_rad[6], 1.05)
        self.assertAlmostEqual(preview.tcp_position_m[0], .15)
        self.assertAlmostEqual(preview.gripper_opening_m, .0215)

    def test_crop_rebases_times_and_overwrites_only_selected_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = make_recording(root / "original.json")
            original_mode = path.stat().st_mode & 0o777
            clip = EditableRecording.load(path, "a" * 64)
            returned = clip.overwrite_crop(1, 2)

            self.assertEqual(returned, path.resolve())
            self.assertEqual(sorted(item.name for item in root.iterdir()), ["original.json"])
            self.assertEqual(path.stat().st_mode & 0o777, original_mode)
            payload = json.loads(path.read_text())

        self.assertEqual(payload["sample_count"], 2)
        self.assertEqual(payload["duration_s"], 1.0)
        self.assertEqual([sample["time_s"] for sample in payload["samples"]], [0.0, 1.0])
        self.assertAlmostEqual(payload["samples"][0]["positions_rad"][0], .1)
        self.assertAlmostEqual(payload["samples"][1]["positions_rad"][0], .2)
        self.assertEqual(payload["started_at_utc"], "2026-09-20T12:00:01.000Z")

    def test_single_sample_crop_is_a_valid_zero_duration_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            path = make_recording(Path(directory) / "one.json")
            clip = EditableRecording.load(path, "a" * 64)
            payload = clip.cropped_payload(2, 2)
        self.assertEqual(payload["sample_count"], 1)
        self.assertEqual(payload["duration_s"], 0.0)
        self.assertEqual(payload["samples"][0]["time_s"], 0.0)
        self.assertEqual(payload["started_at_utc"], "2026-09-20T12:00:02.000Z")

    def test_loads_and_crops_left_arm_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "left.json"
            names = joint_names("left")
            gripper = gripper_name("left")
            recording = MotionRecording("a" * 64, "left")
            recording.start("2026-10-06T12:00:00.000Z")
            for index in range(3):
                values = {name: .01 * index for name in names}
                values[gripper] = .02
                recording.add(20.0 + index, values, [.1, .2, .3])
            recording.stop()
            recording.save(path)
            clip = EditableRecording.load(path, "a" * 64)
            clip.overwrite_crop(1, 2)
            payload = json.loads(path.read_text())
        self.assertEqual(clip.arm, "left")
        self.assertEqual(clip.joint_names, names)
        self.assertEqual(payload["schema"], SCHEMAS["left"])
        self.assertEqual(payload["arm"], "left")

    def test_rejects_wrong_model_invalid_timing_and_mismatched_duration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = make_recording(Path(directory) / "bad.json")
            with self.assertRaisesRegex(ValueError, "model hash"):
                EditableRecording.load(path, "b" * 64)

            payload = json.loads(path.read_text())
            payload["samples"][2]["time_s"] = payload["samples"][1]["time_s"]
            path.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "strictly increase"):
                EditableRecording.load(path, "a" * 64)

            make_recording(path)
            payload = json.loads(path.read_text())
            payload["duration_s"] = math.pi
            path.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "duration"):
                EditableRecording.load(path, "a" * 64)

    def test_launcher_is_offline_and_uses_separate_ros_domain(self):
        script = (ROOT / "start_recording_editor.sh").read_text()
        launcher = (ROOT / "launch_recording_editor.py").read_text()
        app = (ROOT / "motion_recording/editor_app.py").read_text()
        self.assertIn("motion_recording.editor_app", launcher)
        self.assertIn("ROS_DOMAIN_ID=92", script)
        self.assertNotIn("--hardware", script)
        self.assertNotIn("SingleArmObserver", app)
        self.assertNotIn("can0", app.lower())
        self.assertNotIn("Motors(", app)
        self.assertFalse((ROOT / "start_right_recording_editor.sh").exists())


if __name__ == "__main__":
    unittest.main()
