"""Offline tests for the query-only right-arm trajectory recorder."""
import json
import math
from pathlib import Path
import tempfile
import unittest

from motion_recording.recording import (
    GRIPPER_NAME,
    JOINT_LIMIT_TOLERANCE_RAD,
    JOINT_NAMES,
    NOMINAL_SAMPLE_RATE_HZ,
    SCHEMA,
    MotionRecording,
    SCHEMAS,
    confined_json_path,
    default_filename,
    gripper_name,
    joint_names,
    joint_limit_violation,
)


ROOT = Path(__file__).resolve().parents[1]


def right_state(scale=0.0):
    state = {name: scale * (index + 1) for index, name in enumerate(JOINT_NAMES)}
    state[GRIPPER_NAME] = .022
    return state


class MotionRecordingTests(unittest.TestCase):
    def test_joint_limit_monitor_matches_playback_tolerance(self):
        lower = [-1.0] * 7
        upper = [1.0] * 7
        state = right_state()
        state[JOINT_NAMES[6]] = 1.0 + JOINT_LIMIT_TOLERANCE_RAD
        self.assertIsNone(joint_limit_violation(state, lower, upper))

        state[JOINT_NAMES[6]] += math.radians(.5)
        violation = joint_limit_violation(state, lower, upper)
        self.assertEqual(violation.joint_number, 7)
        self.assertEqual(violation.direction, "above")
        self.assertGreater(violation.excess_rad, math.radians(.5))

    def test_json_contains_timed_right_arm_trajectory_and_metadata(self):
        recording = MotionRecording("a" * 64)
        recording.start("2026-09-18T12:00:00.000Z")
        recording.add(10.0, right_state(.01), [.1, .2, .3])
        recording.add(10.05, right_state(.02), [.11, .21, .31])
        recording.stop()

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motion.json"
            self.assertEqual(recording.save(path), path.resolve())
            payload = json.loads(path.read_text())

        self.assertEqual(payload["schema"], SCHEMA)
        self.assertEqual(payload["arm"], "right")
        self.assertEqual(payload["joint_order"], list(JOINT_NAMES))
        self.assertEqual(payload["gripper_name"], GRIPPER_NAME)
        self.assertEqual(payload["sample_count"], 2)
        self.assertEqual(payload["nominal_sample_rate_hz"], 50.0)
        self.assertEqual(NOMINAL_SAMPLE_RATE_HZ, 50.0)
        self.assertAlmostEqual(payload["duration_s"], .05)
        self.assertEqual([sample["time_s"] for sample in payload["samples"]], [0.0, .05])
        self.assertEqual(len(payload["samples"][0]["positions_rad"]), 7)
        self.assertEqual(payload["samples"][1]["tcp_position_m"], [.11, .21, .31])
        self.assertFalse(recording.dirty)

    def test_recording_can_start_and_stop_at_arbitrary_joint_positions(self):
        recording = MotionRecording("b" * 64)
        arbitrary = right_state(.31)
        arbitrary[JOINT_NAMES[0]] = -1.7
        arbitrary[JOINT_NAMES[-1]] = 1.39
        recording.start()
        recording.add(100.0, arbitrary, [-.25, .48, .91])
        recording.stop()
        sample = recording.as_dict()["samples"][0]
        self.assertAlmostEqual(sample["positions_rad"][0], -1.7)
        self.assertAlmostEqual(sample["positions_rad"][-1], 1.39)

    def test_left_recording_uses_left_schema_joints_and_filename(self):
        names = joint_names("left")
        gripper = gripper_name("left")
        state = {name: .01 * (index + 1) for index, name in enumerate(names)}
        state[gripper] = .019
        recording = MotionRecording("d" * 64, "left")
        recording.start("2026-10-06T12:00:00.000Z")
        recording.add(1.0, state, [.1, .2, .3])
        recording.stop()
        payload = recording.as_dict()
        self.assertEqual(payload["schema"], SCHEMAS["left"])
        self.assertEqual(payload["arm"], "left")
        self.assertEqual(payload["joint_order"], list(names))
        self.assertEqual(payload["gripper_name"], gripper)
        self.assertRegex(default_filename(side="left"), r"^left_motion_\d{8}_\d{6}\.json$")

    def test_invalid_or_out_of_order_samples_and_active_save_are_rejected(self):
        recording = MotionRecording("c" * 64)
        recording.start()
        recording.add(1.0, right_state(), [0, 0, 0])
        with self.assertRaisesRegex(ValueError, "times must increase"):
            recording.add(1.0, right_state(), [0, 0, 0])
        invalid = right_state()
        invalid[JOINT_NAMES[2]] = math.nan
        with self.assertRaisesRegex(ValueError, "invalid values"):
            recording.add(2.0, invalid, [0, 0, 0])
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, "Stop recording"):
                recording.save(Path(directory) / "active.json")

    def test_save_selection_is_confined_to_recordings_folder(self):
        with tempfile.TemporaryDirectory() as directory:
            recordings = Path(directory) / "recordings"
            outside = Path(directory) / "somewhere" / "take_one"
            self.assertEqual(
                confined_json_path(recordings, outside),
                recordings.resolve() / "take_one.json",
            )
        self.assertRegex(default_filename(), r"^right_motion_\d{8}_\d{6}\.json$")

    def test_launcher_requires_one_arm_and_remains_query_only(self):
        launcher = (ROOT / "launch_recording.py").read_text()
        app = (ROOT / "motion_recording/app.py").read_text()
        script = (ROOT / "start_recording.sh").read_text()
        self.assertIn("motion_recording.app", launcher)
        self.assertIn("SingleArmObserver(self.side, self.args.can)", app)
        self.assertIn('mode.add_argument("--right"', app)
        self.assertIn('mode.add_argument("--left"', app)
        self.assertNotIn("Motors(", app)
        self.assertNotIn("Observer(", app.replace("SingleArmObserver(", ""))
        self.assertIn("ROS_DOMAIN_ID=91", script)
        self.assertFalse((ROOT / "start_right_recording.sh").exists())


if __name__ == "__main__":
    unittest.main()
