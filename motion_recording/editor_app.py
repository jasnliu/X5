"""Video-editor-style Tk/ROS preview and crop UI for right-arm recordings."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import signal
import time
import tkinter as tk
from tkinter import filedialog, messagebox

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from safe_zone.geometry import Model, RIGHT_TCP
from .editor import EditableRecording
from .recording import GRIPPER_NAME, JOINT_NAMES


ROOT = Path(__file__).resolve().parents[1]
RECORDINGS_DIR = ROOT / "recordings"
TIMELINE_MARGIN = 44
TIMELINE_Y = 50
TIMELINE_HEIGHT = 22


def format_time(seconds: float) -> str:
    seconds = max(0.0, float(seconds))
    minutes = int(seconds // 60)
    remainder = seconds - minutes * 60
    return f"{minutes:02d}:{remainder:06.3f}"


class App:
    """Offline recording editor. It publishes simulation joint states only."""

    def __init__(self, recording_path=None, show_initial_dialog: bool = True):
        self.model = Model(ROOT / "model/openarmx.urdf")
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
        self.node = Node("right_arm_recording_editor")
        self.publisher = self.node.create_publisher(JointState, "/joint_states", 10)
        self.clip: EditableRecording | None = None
        self.trim_start_index = 0
        self.trim_end_index = 0
        self.playhead_s = 0.0
        self.preview = None
        self.playing = False
        self.play_wall_started = 0.0
        self.play_source_started = 0.0
        self.dragging_handle: str | None = None
        self.dirty = False
        self.closed = False

        self.root = tk.Tk()
        self.root.title("OpenArmX RIGHT recording video editor — SIMULATION ONLY")
        self.root.minsize(760, 600)
        self.file_text = tk.StringVar(value="No recording selected")
        self.time_text = tk.StringVar(value="--:--.--- / --:--.---")
        self.crop_text = tk.StringVar(value="Select a recording to begin")
        self.status_text = tk.StringVar(
            value="Choose a JSON file. This editor never opens CAN or commands the physical arm."
        )
        self.pose_text = tk.StringVar(value="No pose loaded")

        tk.Label(
            self.root,
            text="RIGHT ARM RECORDING EDITOR",
            font=("Sans", 17, "bold"),
        ).pack(padx=20, pady=(15, 4))
        tk.Label(
            self.root,
            text=(
                "RViz is a simulation preview only — no CAN socket, motor driver, enable, hold, or motion command is used.\n"
                "Drag either crop handle like a video editor; RViz immediately previews that retained boundary pose."
            ),
            fg="#8b0000",
            justify="center",
            wraplength=760,
        ).pack(padx=20, pady=(0, 10))

        selector = tk.Frame(self.root)
        selector.pack(fill="x", padx=20, pady=4)
        self.select_button = tk.Button(
            selector, text="SELECT RECORDING JSON…", command=self.choose_recording,
            font=("Sans", 11, "bold"),
        )
        self.select_button.pack(side="left", padx=(0, 10))
        tk.Label(
            selector, textvariable=self.file_text, anchor="w", justify="left",
            wraplength=570,
        ).pack(side="left", fill="x", expand=True)

        player = tk.Frame(self.root)
        player.pack(fill="x", padx=20, pady=(10, 2))
        self.to_start_button = tk.Button(
            player, text="⏮ Crop start", command=self.seek_crop_start,
        )
        self.to_start_button.pack(side="left", padx=(0, 5))
        self.play_button = tk.Button(
            player, text="▶ Play", command=self.toggle_playback,
            font=("Sans", 11, "bold"), width=10,
        )
        self.play_button.pack(side="left", padx=5)
        self.stop_button = tk.Button(
            player, text="■ Stop", command=self.stop_playback,
        )
        self.stop_button.pack(side="left", padx=5)
        self.to_end_button = tk.Button(
            player, text="Crop end ⏭", command=self.seek_crop_end,
        )
        self.to_end_button.pack(side="left", padx=5)
        tk.Label(
            player, textvariable=self.time_text, font=("Monospace", 13, "bold"),
            anchor="e",
        ).pack(side="right", fill="x", expand=True)

        self.timeline = tk.Canvas(
            self.root, height=120, bg="#25272b", highlightthickness=1,
            highlightbackground="#50545b", cursor="hand2",
        )
        self.timeline.pack(fill="x", padx=20, pady=(8, 2))
        self.timeline.bind("<Configure>", lambda _event: self.draw_timeline())
        self.timeline.bind("<Button-1>", self.timeline_press)
        self.timeline.bind("<B1-Motion>", self.timeline_drag)
        self.timeline.bind("<ButtonRelease-1>", self.timeline_release)

        tk.Label(
            self.root,
            text=(
                "Blue handle = delete from start  •  Orange handle = delete from end  •  "
                "click the green section to scrub"
            ),
            fg="#333333",
        ).pack(padx=20, pady=(2, 3))
        tk.Label(
            self.root, textvariable=self.crop_text, font=("Sans", 11, "bold"),
            justify="center",
        ).pack(fill="x", padx=20, pady=3)

        crop_actions = tk.Frame(self.root)
        crop_actions.pack(fill="x", padx=20, pady=6)
        self.reset_button = tk.Button(
            crop_actions, text="Reset crop", command=self.reset_crop,
        )
        self.reset_button.pack(side="left", fill="x", expand=True, padx=(0, 5))
        self.save_button = tk.Button(
            crop_actions,
            text="SAVE CROP — OVERWRITE SELECTED JSON",
            command=self.save_crop,
            bg="#ffd8a8",
            activebackground="#ffc078",
            font=("Sans", 11, "bold"),
        )
        self.save_button.pack(side="left", fill="x", expand=True, padx=(5, 0))

        tk.Label(
            self.root, textvariable=self.status_text, wraplength=760,
            justify="left", anchor="w", font=("Sans", 10),
        ).pack(fill="x", padx=20, pady=(8, 4))
        tk.Label(
            self.root, textvariable=self.pose_text, justify="left", anchor="w",
            font=("Monospace", 9),
        ).pack(fill="x", padx=20, pady=(2, 8))
        tk.Label(
            self.root,
            text="RViz: left-drag rotates · middle-drag pans · wheel zooms (view only)",
        ).pack(padx=20, pady=(2, 12))

        for widget in (
            self.to_start_button, self.play_button, self.stop_button,
            self.to_end_button, self.reset_button, self.save_button,
        ):
            widget.config(state="disabled")

        self.root.bind("<space>", lambda _event: self.toggle_playback())
        self.root.bind("<Home>", lambda _event: self.seek_crop_start())
        self.root.bind("<End>", lambda _event: self.seek_crop_end())
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        signal.signal(signal.SIGINT, lambda *_args: self.close())
        signal.signal(signal.SIGTERM, lambda *_args: self.close())
        self.root.after(20, self.tick)
        if recording_path is not None:
            self.load_recording(recording_path, show_error=True)
        elif show_initial_dialog:
            self.root.after(180, self.choose_recording)

    @property
    def trim_start_s(self) -> float:
        return 0.0 if self.clip is None else self.clip.times[self.trim_start_index]

    @property
    def trim_end_s(self) -> float:
        return 0.0 if self.clip is None else self.clip.times[self.trim_end_index]

    def choose_recording(self) -> None:
        if self.dirty and not messagebox.askyesno(
                "Discard unsaved crop?",
                "Selecting another recording will discard the current unsaved crop handles. Continue?",
                parent=self.root):
            return
        initial = self.clip.source.parent if self.clip is not None else RECORDINGS_DIR
        selected = filedialog.askopenfilename(
            parent=self.root,
            title="Select a right-arm recording to edit",
            initialdir=initial,
            filetypes=[("OpenArmX right motion JSON", "*.json")],
        )
        if selected:
            self.load_recording(selected, show_error=True)

    def load_recording(self, path, show_error: bool = True) -> bool:
        self.pause_playback()
        try:
            clip = EditableRecording.load(path, self.model.digest)
        except (OSError, ValueError) as exc:
            self.status_text.set("Recording rejected: " + str(exc))
            if show_error:
                messagebox.showerror("Recording rejected", str(exc), parent=self.root)
            return False
        self.clip = clip
        self.trim_start_index = 0
        self.trim_end_index = clip.sample_count - 1
        self.playhead_s = 0.0
        self.preview = clip.state_at(0.0)
        self.dragging_handle = None
        self.dirty = False
        self.file_text.set(str(clip.source))
        self.root.title(
            f"OpenArmX RIGHT recording editor — {clip.source.name} — SIMULATION ONLY"
        )
        self.status_text.set(
            f"Loaded {clip.sample_count} samples. Play, scrub, or drag the crop handles; "
            "saving will atomically replace this exact JSON file."
        )
        self.update_interface()
        return True

    def pause_playback(self) -> None:
        self.playing = False
        if hasattr(self, "play_button"):
            self.play_button.config(text="▶ Play")

    def toggle_playback(self) -> None:
        if self.clip is None:
            return
        if self.playing:
            self.pause_playback()
            self.status_text.set("Playback paused. Click the timeline to scrub.")
            self.update_interface()
            return
        if self.playhead_s < self.trim_start_s or self.playhead_s >= self.trim_end_s:
            self.playhead_s = self.trim_start_s
        if self.trim_end_s <= self.trim_start_s:
            self.show_time(self.trim_start_s, "The retained clip contains one sample.")
            return
        self.playing = True
        self.play_wall_started = time.monotonic()
        self.play_source_started = self.playhead_s
        self.play_button.config(text="⏸ Pause")
        self.status_text.set("Playing the retained section in RViz at recorded timing.")

    def stop_playback(self) -> None:
        if self.clip is None:
            return
        self.pause_playback()
        self.show_time(self.trim_start_s, "Playback stopped at the retained start pose.")

    def seek_crop_start(self) -> None:
        if self.clip is None:
            return
        self.pause_playback()
        self.show_time(self.trim_start_s, "Previewing the retained start boundary pose.")

    def seek_crop_end(self) -> None:
        if self.clip is None:
            return
        self.pause_playback()
        self.show_time(self.trim_end_s, "Previewing the retained end boundary pose.")

    def reset_crop(self) -> None:
        if self.clip is None:
            return
        self.pause_playback()
        self.trim_start_index = 0
        self.trim_end_index = self.clip.sample_count - 1
        self.dirty = False
        self.show_time(0.0, "Crop reset; the full recording is retained.")

    def _timeline_bounds(self) -> tuple[float, float]:
        width = max(300, self.timeline.winfo_width())
        return float(TIMELINE_MARGIN), float(width - TIMELINE_MARGIN)

    def _time_to_x(self, time_s: float) -> float:
        left, right = self._timeline_bounds()
        if self.clip is None or self.clip.duration_s <= 0.0:
            return left
        return left + (right - left) * float(time_s) / self.clip.duration_s

    def _x_to_time(self, x: float) -> float:
        left, right = self._timeline_bounds()
        if self.clip is None or right <= left:
            return 0.0
        fraction = min(1.0, max(0.0, (float(x) - left) / (right - left)))
        return fraction * self.clip.duration_s

    def timeline_press(self, event) -> None:
        if self.clip is None:
            return
        start_x = self._time_to_x(self.trim_start_s)
        end_x = self._time_to_x(self.trim_end_s)
        start_distance = abs(event.x - start_x)
        end_distance = abs(event.x - end_x)
        if min(start_distance, end_distance) <= 15:
            self.dragging_handle = "start" if start_distance <= end_distance else "end"
            self.timeline_drag(event)
            return
        self.dragging_handle = None
        self.pause_playback()
        target = min(max(self._x_to_time(event.x), self.trim_start_s), self.trim_end_s)
        self.show_time(target, "Scrubbing the retained recording.")

    def timeline_drag(self, event) -> None:
        if self.clip is None or self.dragging_handle is None:
            return
        self.pause_playback()
        target = self._x_to_time(event.x)
        if self.dragging_handle == "start":
            index = self.clip.nearest_index(
                target, minimum=0, maximum=self.trim_end_index
            )
            self.trim_start_index = index
            boundary = self.trim_start_s
            detail = "START crop boundary"
        else:
            index = self.clip.nearest_index(
                target, minimum=self.trim_start_index,
                maximum=self.clip.sample_count - 1,
            )
            self.trim_end_index = index
            boundary = self.trim_end_s
            detail = "END crop boundary"
        self.dirty = not (
            self.trim_start_index == 0
            and self.trim_end_index == self.clip.sample_count - 1
        )
        self.show_time(
            boundary,
            f"Dragging {detail}: RViz is previewing this exact retained boundary pose.",
        )

    def timeline_release(self, _event) -> None:
        self.dragging_handle = None

    def show_time(self, time_s: float, status: str | None = None) -> None:
        if self.clip is None:
            return
        self.playhead_s = min(max(float(time_s), self.trim_start_s), self.trim_end_s)
        self.preview = self.clip.state_at(self.playhead_s)
        if status:
            self.status_text.set(status)
        self.update_interface()

    def draw_timeline(self) -> None:
        canvas = self.timeline
        canvas.delete("all")
        left, right = self._timeline_bounds()
        y0, y1 = TIMELINE_Y, TIMELINE_Y + TIMELINE_HEIGHT
        canvas.create_rectangle(left, y0, right, y1, fill="#555b65", outline="#d0d3d8")
        if self.clip is None:
            canvas.create_text(
                (left + right) / 2, (y0 + y1) / 2,
                text="Select a JSON recording", fill="white",
            )
            return
        start_x = self._time_to_x(self.trim_start_s)
        end_x = self._time_to_x(self.trim_end_s)
        canvas.create_rectangle(left, y0, start_x, y1, fill="#8a3540", outline="")
        canvas.create_rectangle(start_x, y0, end_x, y1, fill="#319568", outline="")
        canvas.create_rectangle(end_x, y0, right, y1, fill="#8a3540", outline="")
        for index in range(5):
            fraction = index / 4
            x = left + (right - left) * fraction
            timestamp = self.clip.duration_s * fraction
            canvas.create_line(x, y1 + 2, x, y1 + 8, fill="#d0d3d8")
            canvas.create_text(
                x, y1 + 22, text=format_time(timestamp), fill="#e6e8eb",
                font=("Monospace", 8),
            )
        canvas.create_line(start_x, y0 - 15, start_x, y1 + 12, fill="#48a9ff", width=4)
        canvas.create_polygon(
            start_x - 8, y0 - 15, start_x + 8, y0 - 15, start_x, y0 - 5,
            fill="#48a9ff", outline="white",
        )
        canvas.create_text(start_x, y0 - 27, text="START", fill="#8dccff", font=("Sans", 8, "bold"))
        canvas.create_line(end_x, y0 - 15, end_x, y1 + 12, fill="#ff9f43", width=4)
        canvas.create_polygon(
            end_x - 8, y0 - 15, end_x + 8, y0 - 15, end_x, y0 - 5,
            fill="#ff9f43", outline="white",
        )
        canvas.create_text(end_x, y0 - 27, text="END", fill="#ffc078", font=("Sans", 8, "bold"))
        play_x = self._time_to_x(self.playhead_s)
        canvas.create_line(play_x, y0 - 7, play_x, y1 + 7, fill="white", width=2)
        canvas.create_oval(play_x - 4, y0 - 11, play_x + 4, y0 - 3, fill="white", outline="")

    def update_interface(self) -> None:
        if self.clip is None:
            self.draw_timeline()
            return
        removed_start = self.trim_start_s
        removed_end = self.clip.duration_s - self.trim_end_s
        kept_duration = self.trim_end_s - self.trim_start_s
        kept_samples = self.trim_end_index - self.trim_start_index + 1
        self.time_text.set(
            f"{format_time(self.playhead_s)} / {format_time(self.clip.duration_s)}"
        )
        self.crop_text.set(
            f"Delete start: {removed_start:.3f} s  |  Keep: {kept_duration:.3f} s "
            f"({kept_samples}/{self.clip.sample_count} samples)  |  Delete end: {removed_end:.3f} s"
        )
        state = self.preview or self.clip.state_at(self.playhead_s)
        values = {name: value for name, value in zip(JOINT_NAMES, state.positions_rad)}
        values[GRIPPER_NAME] = state.gripper_opening_m
        all_values = {name: 0.0 for name in self.model.names}
        all_values.update(values)
        tcp = self.model.transforms(all_values)[RIGHT_TCP][:3, 3]
        degrees = [math.degrees(value) for value in state.positions_rad]
        self.pose_text.set(
            "Preview right joint degrees:\n" +
            "  ".join(f"J{i + 1}: {value:+7.2f}" for i, value in enumerate(degrees)) +
            f"\nTCP meters: X={tcp[0]:+.4f}  Y={tcp[1]:+.4f}  Z={tcp[2]:+.4f}" +
            f"   Gripper: {state.gripper_opening_m * 1000:.1f} mm"
        )
        for widget in (
            self.to_start_button, self.play_button, self.stop_button,
            self.to_end_button, self.reset_button,
        ):
            widget.config(state="normal")
        self.save_button.config(state="normal" if self.dirty else "disabled")
        self.draw_timeline()

    def publish_preview(self) -> None:
        if self.clip is None or self.preview is None:
            return
        state = {name: 0.0 for name in self.model.names}
        state.update(dict(zip(JOINT_NAMES, self.preview.positions_rad)))
        state[GRIPPER_NAME] = self.preview.gripper_opening_m
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(state)
        message.position = list(state.values())
        self.publisher.publish(message)

    def save_crop(self) -> None:
        if self.clip is None or not self.dirty:
            return
        kept = self.trim_end_index - self.trim_start_index + 1
        removed = self.clip.sample_count - kept
        path = self.clip.source
        question = (
            f"Overwrite this exact file?\n\n{path}\n\n"
            f"This permanently removes {removed} sample(s):\n"
            f"• first {self.trim_start_s:.3f} seconds\n"
            f"• last {self.clip.duration_s - self.trim_end_s:.3f} seconds\n\n"
            "No new recording file or backup will be created."
        )
        if not messagebox.askyesno("Overwrite selected recording?", question, parent=self.root):
            return
        self.pause_playback()
        try:
            saved = self.clip.overwrite_crop(self.trim_start_index, self.trim_end_index)
            # Reload what was actually written. The saved crop becomes the new full timeline.
            self.load_recording(saved, show_error=False)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Save failed", str(exc), parent=self.root)
            self.status_text.set("Save failed; the selected JSON was not replaced: " + str(exc))
            return
        self.status_text.set(
            f"Saved in place: {saved}. The selected JSON was overwritten; no new recording was made."
        )
        messagebox.showinfo(
            "Crop saved",
            f"The selected recording was updated in place:\n\n{saved}",
            parent=self.root,
        )
        self.update_interface()

    def tick(self) -> None:
        if self.closed:
            return
        if self.clip is not None and self.playing:
            elapsed = time.monotonic() - self.play_wall_started
            target = self.play_source_started + elapsed
            if target >= self.trim_end_s:
                target = self.trim_end_s
                self.pause_playback()
                self.status_text.set("Playback reached the retained end boundary.")
            self.playhead_s = target
            self.preview = self.clip.state_at(target)
            self.update_interface()
        rclpy.spin_once(self.node, timeout_sec=0)
        self.publish_preview()
        self.root.after(20, self.tick)

    def close(self) -> None:
        if self.closed:
            return
        if self.dirty and not messagebox.askyesno(
                "Unsaved crop",
                "Exit without saving the crop changes? The JSON file has not been modified.",
                parent=self.root):
            return
        self.closed = True
        self.pause_playback()
        self.root.quit()

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            self.closed = True
            self.root.destroy()
            self.node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Offline RViz video-style crop editor for OpenArmX right-arm recordings"
    )
    parser.add_argument(
        "--recording",
        help="optional right-arm JSON to open immediately; otherwise a chooser opens at startup",
    )
    args = parser.parse_args()
    rclpy.init(args=[])
    try:
        App(args.recording).run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
