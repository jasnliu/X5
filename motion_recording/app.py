"""Tk/ROS interface for recording a manually guided, disabled right arm."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import queue
import signal
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from safe_zone.encoder import SingleArmObserver
from safe_zone.geometry import Model, RIGHT_TCP
from .recording import (
    GRIPPER_NAME,
    JOINT_NAMES,
    NOMINAL_SAMPLE_RATE_HZ,
    JOINT_LIMIT_TOLERANCE_RAD,
    MotionRecording,
    confined_json_path,
    default_filename,
    joint_limit_violation,
)


ROOT = Path(__file__).resolve().parents[1]


class App:
    def __init__(self, args):
        self.args = args
        self.model = Model(ROOT / "model/openarmx.urdf")
        model_joints = {joint.get("name"): joint for joint in self.model.joints}
        limits = [
            model_joints[name].find("limit") for name in JOINT_NAMES
        ]
        self.lower = tuple(float(limit.get("lower")) for limit in limits)
        self.upper = tuple(float(limit.get("upper")) for limit in limits)
        self.recordings_dir = ROOT / "recordings"
        self.recordings_dir.mkdir(parents=True, exist_ok=True)
        self.recording = MotionRecording(self.model.digest)
        self.saved_path = None
        self.latest = None
        self.latest_time = 0.0
        self.error = None
        self.recording_notice = None
        self.stop_event = threading.Event()
        self.inbox = queue.Queue(maxsize=128)
        self.worker = None

        self.node = Node("right_arm_motion_recorder")
        self.publisher = self.node.create_publisher(JointState, "/joint_states", 10)
        self.root = tk.Tk()
        suffix = "" if args.hardware else " — OFFLINE PREVIEW"
        self.root.title("OpenArmX RIGHT arm motion recorder" + suffix)
        self.status = tk.StringVar()
        self.angles = tk.StringVar()

        tk.Label(
            self.root,
            text="RIGHT ARM MANUAL MOTION RECORDER",
            font=("Sans", 16, "bold"),
        ).pack(padx=18, pady=(14, 6))
        tk.Label(
            self.root,
            text=(
                "Query-only: right motors 1–8 must already be disabled.\n"
                "Support the limp arm at all times; this program provides no holding or gravity compensation."
            ),
            fg="darkred",
            wraplength=620,
            justify="center",
        ).pack(padx=18, pady=6)
        tk.Label(
            self.root,
            text=(
                "The recording may begin and end at any position. No centering, zone, or path requirement is applied.\n"
                "A J1–J7 limit violation warns, aborts, and discards the entire active recording.\n"
                "Only can0/right-arm feedback is queried; the left arm is not queried or recorded."
            ),
            wraplength=620,
            justify="center",
        ).pack(padx=18, pady=6)

        controls = tk.Frame(self.root)
        controls.pack(fill="x", padx=18, pady=8)
        self.start_button = tk.Button(
            controls, text="Start new recording", command=self.start_recording,
            font=("Sans", 13, "bold"),
        )
        self.start_button.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.stop_button = tk.Button(
            controls, text="Stop recording", command=self.stop_recording,
            state="disabled",
        )
        self.stop_button.pack(side="left", fill="x", expand=True, padx=(4, 0))
        self.save_button = tk.Button(
            self.root, text="Save recording as JSON…", command=self.save_recording,
            state="disabled",
        )
        self.save_button.pack(fill="x", padx=18, pady=4)
        self.clear_button = tk.Button(
            self.root, text="Discard / new recording…", command=self.clear_recording,
            state="disabled",
        )
        self.clear_button.pack(fill="x", padx=18, pady=4)

        tk.Label(
            self.root, textvariable=self.status, wraplength=620,
            justify="left", font=("Sans", 11),
        ).pack(fill="x", padx=18, pady=(12, 6))
        tk.Label(
            self.root, textvariable=self.angles, justify="left",
            font=("Monospace", 10),
        ).pack(fill="x", padx=18, pady=6)
        tk.Label(
            self.root,
            text="RViz: left-drag rotates · middle-drag pans · wheel zooms (view only)",
        ).pack(padx=18, pady=(4, 14))

        if args.hardware:
            self.worker = threading.Thread(target=self.observe, daemon=True)
            self.worker.start()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        signal.signal(signal.SIGINT, lambda *_: self.close())
        signal.signal(signal.SIGTERM, lambda *_: self.close())
        self.root.after(50, self.tick)
        self.update_panel()

    def observe(self) -> None:
        observer = None
        try:
            observer = SingleArmObserver("right", self.args.right_can)
            period = 1.0 / NOMINAL_SAMPLE_RATE_HZ
            while not self.stop_event.is_set():
                started = time.monotonic()
                values = observer.sample()
                self.offer(("sample", time.monotonic(), values))
                self.stop_event.wait(max(0.0, period - (time.monotonic() - started)))
        except Exception as exc:
            self.offer(("error", time.monotonic(), str(exc)))
        finally:
            if observer is not None:
                observer.close()

    def offer(self, item) -> None:
        try:
            self.inbox.put_nowait(item)
        except queue.Full:
            # Modal dialogs (especially the joint-limit warning) pause Tk's
            # consumer while the query-only feedback worker keeps running.  If
            # no take is active, old display-only samples may safely collapse
            # to the newest fresh state.  Losing samples during an active take
            # remains a fatal recording error, as before.
            if item[0] == "sample" and not self.recording.active:
                retained = []
                try:
                    while True:
                        queued = self.inbox.get_nowait()
                        if queued[0] != "sample":
                            retained.append(queued)
                except queue.Empty:
                    pass
                for queued in retained:
                    self.inbox.put_nowait(queued)
                self.inbox.put_nowait(item)
                return
            try:
                while True:
                    self.inbox.get_nowait()
            except queue.Empty:
                pass
            self.inbox.put_nowait((
                "error", time.monotonic(),
                "Recording stopped because the display could not keep up with encoder samples",
            ))
            self.stop_event.set()

    def fresh(self) -> bool:
        return (
            self.args.hardware and self.error is None and self.latest is not None
            and time.monotonic() - self.latest_time < .3
        )

    def start_recording(self) -> None:
        if not self.fresh():
            messagebox.showwarning(
                "Not ready",
                "Recording requires fresh feedback confirming right motors 1–8 are disabled.",
            )
            return
        violation = joint_limit_violation(self.latest, self.lower, self.upper)
        if violation is not None:
            detail = self._joint_limit_detail(violation)
            self.recording_notice = (
                "WARNING: recording not started — " + detail +
                ". Move the joint back inside its limit."
            )
            messagebox.showwarning("Joint limit exceeded", self.recording_notice)
            self.update_panel()
            return
        if self.recording.samples:
            detail = "Discard the current unsaved recording?" if self.recording.dirty else "Start a new recording?"
            if not messagebox.askyesno("Start new recording?", detail):
                return
        self.recording.start()
        self.saved_path = None
        self.recording_notice = None
        self.update_panel()

    @staticmethod
    def _joint_limit_detail(violation) -> str:
        return (
            f"J{violation.joint_number} measured "
            f"{math.degrees(violation.measured_rad):+.3f}°, "
            f"{violation.direction} its {math.degrees(violation.limit_rad):+.3f}° limit "
            f"by {math.degrees(violation.excess_rad):.3f}°"
        )

    def abort_for_joint_limit(self, violation) -> None:
        detail = self._joint_limit_detail(violation)
        self.recording.reset()
        self.saved_path = None
        self.recording_notice = (
            "WARNING: recording aborted and discarded — " + detail
        )
        messagebox.showwarning(
            "Recording discarded: joint limit exceeded",
            self.recording_notice +
            f". The {math.degrees(JOINT_LIMIT_TOLERANCE_RAD):.3f}° encoder tolerance "
            "was already included.",
        )

    def stop_recording(self, reason: str | None = None) -> None:
        if self.recording.active:
            self.recording.stop()
        if reason:
            self.status.set(reason)
        self.update_panel(reason)

    def save_recording(self) -> None:
        if self.recording.active:
            self.stop_recording()
        if not self.recording.samples:
            messagebox.showwarning("Nothing to save", "Record at least one right-arm sample first.")
            return
        selected = filedialog.asksaveasfilename(
            initialdir=self.recordings_dir,
            initialfile=default_filename(),
            defaultextension=".json",
            filetypes=[("OpenArmX motion JSON", "*.json")],
        )
        if not selected:
            return
        try:
            path = confined_json_path(self.recordings_dir, selected)
            self.saved_path = self.recording.save(path)
            messagebox.showinfo("Recording saved", str(self.saved_path))
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))
        self.update_panel()

    def clear_recording(self) -> None:
        if self.recording.samples and not messagebox.askyesno(
                "Discard recording?", "Discard the current recording and start with an empty buffer?"):
            return
        self.recording.reset()
        self.saved_path = None
        self.recording_notice = None
        self.update_panel()

    def display_state(self):
        state = {name: 0.0 for name in self.model.names}
        if self.latest is not None:
            state.update(self.latest)
        return state

    def publish(self) -> None:
        if self.args.hardware and not self.fresh():
            return
        state = self.display_state()
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(state)
        message.position = list(state.values())
        self.publisher.publish(message)

    def update_panel(self, override: str | None = None) -> None:
        sample_count = len(self.recording.samples)
        duration = self.recording.duration_s
        if override:
            state_text = override
        elif not self.args.hardware:
            state_text = "OFFLINE PREVIEW — no CAN socket; recording is disabled"
        elif self.error:
            state_text = "STOPPED: " + self.error
        elif self.recording_notice:
            state_text = self.recording_notice
        elif not self.fresh():
            state_text = "Waiting for fresh disabled right-arm feedback"
        elif self.recording.active:
            state_text = (
                f"RECORDING AT UP TO {NOMINAL_SAMPLE_RATE_HZ:.0f} Hz — "
                "manually guide and support the limp right arm"
            )
        else:
            state_text = "READY — right motors 1–8 report disabled"
        save_text = str(self.saved_path) if self.saved_path else "not saved"
        self.status.set(
            f"{state_text}\nSamples: {sample_count}   Duration: {duration:.2f} s   File: {save_text}"
        )
        state = self.display_state()
        tcp = self.model.transforms(state)[RIGHT_TCP][:3, 3]
        degrees = [math.degrees(state.get(name, 0.0)) for name in JOINT_NAMES]
        self.angles.set(
            "Right joint degrees:\n" +
            "  ".join(f"J{i + 1}: {value:+7.2f}" for i, value in enumerate(degrees)) +
            f"\nTCP meters: X={tcp[0]:+.4f}  Y={tcp[1]:+.4f}  Z={tcp[2]:+.4f}" +
            f"\nGripper opening: {state.get(GRIPPER_NAME, 0.0) * 1000:.1f} mm"
        )
        can_start = self.fresh() and not self.recording.active
        self.start_button.config(state="normal" if can_start else "disabled")
        self.stop_button.config(state="normal" if self.recording.active else "disabled")
        self.save_button.config(state="normal" if sample_count else "disabled")
        self.clear_button.config(state="normal" if sample_count else "disabled")

    def tick(self) -> None:
        while True:
            try:
                kind, stamp, value = self.inbox.get_nowait()
            except queue.Empty:
                break
            if kind == "error":
                self.error = value
                self.stop_recording("STOPPED: " + value)
                continue
            if self.error is not None:
                continue
            self.latest = value
            self.latest_time = stamp
            if self.recording.active:
                try:
                    violation = joint_limit_violation(value, self.lower, self.upper)
                    if violation is not None:
                        self.abort_for_joint_limit(violation)
                        continue
                    tcp = self.model.transforms(value)[RIGHT_TCP][:3, 3]
                    self.recording.add(stamp, value, tcp)
                except Exception as exc:
                    self.error = str(exc)
                    self.stop_recording("STOPPED: " + str(exc))
        if self.recording.active and not self.fresh():
            self.stop_recording("Recording stopped because right-arm feedback became stale")
        rclpy.spin_once(self.node, timeout_sec=0)
        self.publish()
        self.update_panel()
        self.root.after(50, self.tick)

    def close(self) -> None:
        if self.recording.active:
            self.recording.stop()
        if self.recording.dirty and self.recording.samples:
            if not messagebox.askyesno(
                    "Unsaved recording", "Exit without saving the current recording?"):
                self.update_panel()
                return
        self.stop_event.set()
        self.root.quit()

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            self.stop_event.set()
            if self.worker is not None:
                self.worker.join(timeout=1.0)
            self.root.destroy()
            self.node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description="Query-only OpenArmX right-arm motion recorder")
    parser.add_argument("--hardware", action="store_true")
    parser.add_argument("--right-can", default="can0")
    args = parser.parse_args()
    rclpy.init(args=[])
    try:
        App(args).run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
