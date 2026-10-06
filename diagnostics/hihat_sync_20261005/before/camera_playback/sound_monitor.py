"""Hi-hat audio display only: no arm, motor, search or beat-control references."""
from __future__ import annotations

import time
import tkinter as tk

from .audio import AudioReceiver


class HiHatSoundMonitor:
    def __init__(self, root, socket_path, *, before, disabled=False):
        self.root = root
        self.disabled = disabled
        self.receiver = None
        self.startup_error = None
        if socket_path and not disabled:
            try:
                self.receiver = AudioReceiver(socket_path, instrument="hihat")
            except Exception as exc:
                self.startup_error = str(exc)
        self.count = 0
        self.last_hit = None
        self.last_received_at = None
        self.closed = False
        self.after_id = None
        self.text = tk.StringVar(master=root)
        self.label = tk.Label(root, textvariable=self.text, font=("Sans", 11, "bold"),
                              wraplength=650, padx=8, pady=6)
        self.normal_background = self.label.cget('background')
        self.label.pack(fill="x", padx=20, pady=3, before=before)
        self.tick()

    def poll_display(self, now=None):
        """Return display state only. Even a detector failure is informational."""
        if self.disabled:
            return "Hi-hat sound v1: ignored in test mode — no microphone", "#666666", False
        if getattr(self, 'startup_error', None):
            return ("Hi-hat sound v1: UNAVAILABLE — visual only; beat unaffected\n"
                    + self.startup_error[:220]), '#b00020', False
        if self.receiver is None:
            return "Hi-hat sound v1: not connected — visual only", "#8a5a00", False
        messages = self.receiver.poll()
        # The receiver timestamps status at receipt. Sample display time after
        # polling so a fresh heartbeat cannot look microscopically future-dated.
        now = time.monotonic() if now is None else now
        for message in messages:
            if message.get('kind') == 'hit' and message.get('instrument') == 'hihat':
                self.count += 1
                self.last_hit = message
                self.last_received_at = now
                print(f"HI-HAT SOUND (visual only) #{self.count}: "
                      f"score={message['score']:.3f}; normality={message['normality_score']:.1f}",
                      flush=True)
        summary = f"Closures: {self.count}"
        if self.last_hit:
            summary += (f" | last score {self.last_hit['score']:.3f}"
                        f" | normality {self.last_hit['normality_score']:.1f}")
        if self.receiver.state in {'error', 'stopped'}:
            return ("Hi-hat sound v1: UNAVAILABLE — visual only; beat unaffected\n"
                    + self.receiver.detail[:220] + " | " + summary), '#b00020', False
        if self.receiver.ready(now):
            flash = self.last_received_at is not None and now-self.last_received_at < .6
            status = 'CLOSURE DETECTED' if flash else 'READY'
            return f"Hi-hat sound v1: {status} — visual only\n{summary}", '#087f23', flash
        if self.receiver.state == 'ready':
            return f"Hi-hat sound v1: STALE — visual only; beat unaffected\n{summary}", '#b00020', False
        return f"Hi-hat sound v1: warming up on TONOR — visual only\n{summary}", '#8a5a00', False

    def tick(self):
        if self.closed:
            return
        try:
            text, color, flash = self.poll_display()
        except Exception as exc:
            # Never propagate an optional visual monitor failure into the
            # hardware application's callback-failure/center recovery handler.
            text = 'Hi-hat sound v1: display error — beat unaffected\n' + str(exc)[:220]
            color, flash = '#b00020', False
        self.text.set(text)
        self.label.config(fg=color, bg='#c8f5d2' if flash else self.normal_background)
        self.after_id = self.root.after(50, self.tick)

    def close(self):
        self.closed = True
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
            except tk.TclError:
                pass
        if self.receiver is not None:
            self.receiver.close()
