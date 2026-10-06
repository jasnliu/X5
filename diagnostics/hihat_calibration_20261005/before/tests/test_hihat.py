"""Offline pseudo-terminal tests for the integrated ESP32 hi-hat controller."""
import fcntl
import os
from pathlib import Path
import pty
import re
import tempfile
import termios
import unittest

from camera_playback.hihat import (
    DEFAULT_ESP_PORT,
    ESP_BAUD,
    ESP_HEARTBEAT_SECONDS,
    HiHatController,
    MOTOR2_TARGET_DEGREES,
    find_esp_port,
)


class HiHatControllerTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        flags = fcntl.fcntl(self.master, fcntl.F_GETFL)
        fcntl.fcntl(self.master, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self.port = os.ttyname(self.slave)
        self.controller = HiHatController(self.port)

    def tearDown(self):
        self.controller.close()
        os.close(self.master)
        os.close(self.slave)

    def drain(self) -> bytes:
        result = bytearray()
        while True:
            try:
                result.extend(os.read(self.master, 256))
            except BlockingIOError:
                break
        return bytes(result)

    def make_ready(self, start: float = 10.0) -> bytes:
        self.assertTrue(self.controller.connect(now=start))
        self.controller.tick(start + 0.31)
        handshake = self.drain()
        os.write(self.master, b"STOP -- all motors released.\r\n")
        self.controller.tick(start + 0.32)
        self.assertTrue(self.controller.ready())
        return handshake

    def test_proven_port_baud_and_current_motor2_target(self):
        self.assertEqual(ESP_BAUD, 115200)
        self.assertEqual(MOTOR2_TARGET_DEGREES, 100)
        self.assertEqual(ESP_HEARTBEAT_SECONDS, 0.1)
        self.assertIn("Silicon_Labs_CP2102", DEFAULT_ESP_PORT)
        self.assertEqual(find_esp_port(self.port), self.port)

        self.assertTrue(self.controller.connect(now=10.0))
        attrs = termios.tcgetattr(self.controller.fd)
        self.assertEqual(attrs[4], termios.B115200)
        self.assertEqual(attrs[5], termios.B115200)

    def test_firmware_and_host_close_targets_match_at_100_degrees(self):
        firmware = (Path(__file__).resolve().parents[1] /
                    'esp32_hihat/motor_beat/motor_beat.ino').read_text()
        for name in ('BEAT_TARGET_DEGREES', 'KEY_TARGET_DEGREES'):
            value = float(re.search(rf'{name}\s*=\s*([0-9.]+)f;', firmware).group(1))
            self.assertEqual(value, MOTOR2_TARGET_DEGREES)
        self.assertEqual(round(28 * 19.2 * MOTOR2_TARGET_DEGREES / 360), 149)
        self.assertIn("m2Close(BEAT_TARGET_COUNTS)", firmware)
        self.assertIn("m2Open()  { setpoint2 = 0;", firmware)

    def test_arm_beats_replace_motor1_and_alternate_motor2_close_open(self):
        handshake = self.make_ready()
        self.controller.start_sequence()
        commands = [self.controller.send_beat() for _ in range(4)]
        self.assertEqual(commands, [b"C", b"O", b"C", b"O"])
        self.assertEqual(self.controller.beat_count, 4)
        self.controller.stop_sequence()
        self.controller.close()

        wire = handshake + self.drain()
        self.assertEqual(wire, b"HSCOCOOS")
        self.assertNotIn(b"K", wire)

    def test_explicit_sync_edges_open13_close24_without_target_changes(self):
        self.make_ready()
        self.drain()
        self.controller.start_sequence()
        commands = [self.controller.send_state(closed) for closed in (False, True, False, True)]
        self.assertEqual(commands, [b"O", b"C", b"O", b"C"])
        self.assertEqual(self.drain(), b"OCOC")
        self.assertTrue(self.controller.motor2_closed)
        self.assertEqual(self.controller.beat_count, 4)
        self.controller.stop_sequence()
        self.assertEqual(self.drain(), b"O")
        with self.assertRaises(RuntimeError):
            self.controller.send_state(True)

    def test_firmware_ready_line_can_finish_warmup_early(self):
        self.assertTrue(self.controller.connect(now=20.0))
        os.write(self.master, b"READY motor_beat (2 motors)\r\n")
        self.controller.tick(20.01)
        self.assertTrue(self.controller.ready())
        self.assertIn("100", self.controller.detail)
        self.assertEqual(self.drain(), b"HS")

    def test_firmware_fault_is_latched(self):
        self.assertTrue(self.controller.connect(now=30.0))
        self.controller.tick(30.31)
        self.drain()
        os.write(self.master, b"FAULT: Motor 2 runaway\r\n")
        self.controller.tick(30.32)
        self.assertEqual(self.controller.state, "error")
        self.assertIn("runaway", self.controller.detail)
        self.assertFalse(self.controller.sequence_active)

    def test_emergency_release_uses_stop_not_a_return_motion(self):
        self.make_ready(40.0)
        self.controller.start_sequence()
        self.controller.send_beat()
        self.drain()
        self.controller.release_all()
        self.assertEqual(self.drain(), b"S")
        self.assertFalse(self.controller.sequence_active)
        self.assertFalse(self.controller.motor2_closed)

    def test_missing_explicit_port_fails_without_using_another_device(self):
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "missing-esp32"
            with self.assertRaisesRegex(FileNotFoundError, "does not exist"):
                find_esp_port(missing)


if __name__ == "__main__":
    unittest.main()
