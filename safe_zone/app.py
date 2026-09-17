import argparse
import queue
import signal
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from .geometry import Model, Zone, FRAME, LEFT_TCP, RIGHT_TCP, MEMBERSHIP_BUFFER_M
from .encoder import Observer

ROOT = Path(__file__).resolve().parents[1]

class App:
    def __init__(self, args):
        self.args = args
        self.side = args.arm
        self.other_side = 'right' if self.side == 'left' else 'left'
        self.tcp = LEFT_TCP if self.side == 'left' else RIGHT_TCP
        self.zone_dir = ROOT / f'{self.side}_zones'
        self.zone_dir.mkdir(parents=True, exist_ok=True)
        self.model = Model(ROOT / 'model/openarmx.urdf')
        self.zone = Zone(self.model.digest, self.tcp)
        self.zone_committed = bool(args.load)
        if args.load: self.zone = Zone.load(args.load, self.model.digest, self.tcp)
        self.node = Node(f'{self.side}_safe_zone_recorder')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.joints = self.node.create_publisher(JointState, '/joint_states', 10)
        self.markers = self.node.create_publisher(MarkerArray, '/safe_zone/markers', qos)
        self.root = tk.Tk()
        self.root.title(f'OpenArmX {self.side.upper()} safe-zone teaching — NO MOTOR CONTROL')
        self.recording = False
        self.latest = None
        self.latest_time = 0.
        self.error = None
        self.stop = threading.Event()
        self.inbox = queue.Queue(maxsize=2)
        self.status = tk.StringVar()
        tk.Label(self.root, text=f'{self.side.upper()} TCP convex envelope — ESTIMATE, NOT A SAFETY GUARD', fg='darkred').pack(padx=15, pady=8)
        tk.Label(self.root, text=f'Both arms must already be disabled. Support the {self.side} arm by hand.\nNo holding / gravity compensation. {self.other_side.capitalize()} arm is display-only.\nTrace multiple heights and outer boundaries; concavities are filled.').pack()
        self.zone_indicator = tk.Label(self.root, text='ZONE STATUS UNKNOWN', font=('Sans', 15, 'bold'), wraplength=580)
        self.zone_indicator.pack(padx=15, pady=10)
        tk.Label(self.root, text='RViz camera: left-drag rotates · middle-drag pans · wheel zooms.\nUse Move Camera in the RViz toolbar (view only; does not move the arm).').pack(padx=15, pady=5)
        self.record_button = tk.Button(self.root, text='Start recording', command=self.toggle)
        self.record_button.pack(fill='x', padx=15, pady=5)
        tk.Button(self.root, text='Capture one point', command=self.capture).pack(fill='x', padx=15)
        tk.Button(self.root, text='Save zone…', command=self.save).pack(fill='x', padx=15, pady=5)
        tk.Button(self.root, text='Import zone… (pauses recording)', command=self.load).pack(fill='x', padx=15)
        tk.Button(self.root, text='Clear / new zone…', command=self.clear).pack(fill='x', padx=15, pady=5)
        tk.Label(self.root, textvariable=self.status, wraplength=580, justify='left').pack(padx=15, pady=12)
        self.worker = None
        if args.hardware:
            self.worker = threading.Thread(target=self.observe, daemon=True)
            self.worker.start()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        signal.signal(signal.SIGINT, lambda *_: self.root.quit())
        signal.signal(signal.SIGTERM, lambda *_: self.root.quit())
        self.root.after(50, self.tick)

    def observe(self):
        observer = None
        try:
            observer = Observer(self.args.left_can, self.args.right_can)
            while not self.stop.is_set():
                start = time.monotonic()
                values = observer.sample()
                self.offer(('sample', time.monotonic(), values))
                self.stop.wait(max(0, .05 - (time.monotonic() - start)))
        except Exception as e:
            self.offer(('error', time.monotonic(), str(e)))
        finally:
            if observer: observer.close()

    def offer(self, item):
        try: self.inbox.put_nowait(item)
        except queue.Full:
            try: self.inbox.get_nowait()
            except queue.Empty: pass
            self.inbox.put_nowait(item)

    def fresh(self):
        return self.args.hardware and not self.error and self.latest is not None and time.monotonic() - self.latest_time < .3

    def toggle(self):
        if not self.recording and not self.fresh():
            messagebox.showwarning('Not ready', 'Recording requires fresh readings from all 16 disabled motors. Offline mode is view-only.')
            return
        self.recording = not self.recording
        self.record_button.config(text='Pause recording' if self.recording else 'Start recording')

    def capture(self):
        if not self.fresh():
            messagebox.showwarning('Not ready', 'No fresh, verified disabled-motor feedback.')
            return
        self.add_point()

    def add_point(self):
        p = self.model.transforms(self.latest)[self.tcp][:3, 3]
        try:
            if self.zone.add(p): self.zone_committed = False
        except ValueError as e:
            self.recording = False
            messagebox.showerror('Recording stopped', str(e))

    def save(self):
        self.recording = False
        selected = filedialog.asksaveasfilename(initialdir=self.zone_dir, defaultextension='.json', filetypes=[('Zone JSON', '*.json')])
        if selected:
            # The arm-specific recorder never writes outside its own zone folder.
            path = self.zone_dir / Path(selected).name
            if path.suffix.lower() != '.json': path = path.with_name(path.name + '.json')
            try:
                self.zone.save(path)
                self.zone_committed = True
                messagebox.showinfo('Saved', path)
            except Exception as e: messagebox.showerror('Save failed', str(e))

    def load(self):
        self.recording = False
        path = filedialog.askopenfilename(initialdir=self.zone_dir, filetypes=[('Zone JSON', '*.json')])
        if path:
            try:
                self.zone = Zone.load(path, self.model.digest, self.tcp)
                self.zone_committed = True
            except Exception as e: messagebox.showerror('Import failed', str(e))

    def clear(self):
        if messagebox.askyesno('Clear zone?', 'Discard unsaved points and start a new zone?'):
            self.recording = False
            self.zone = Zone(self.model.digest, self.tcp)
            self.zone_committed = False

    def membership(self):
        """Never classify offline, stale, faulted, incomplete or changing zones."""
        neutral = (1., .65, 0., 1.)
        if self.recording:
            return "RECORDING — zone check paused", neutral
        if self.error:
            return "ZONE STATUS UNKNOWN — encoder fault", neutral
        if not self.zone_committed:
            return "ZONE STATUS UNKNOWN — save or import a zone", neutral
        self.zone.rebuild()
        if self.zone.hull is None:
            return "ZONE STATUS UNKNOWN — no 3-D volume", neutral
        if not self.fresh():
            return "ZONE STATUS UNKNOWN — no fresh live encoders", neutral
        p = self.model.transforms(self.latest)[self.tcp][:3, 3]
        if self.zone.contains(p, buffer_m=MEMBERSHIP_BUFFER_M):
            return f"INSIDE SAFE ZONE ({self.side} TCP estimate)", (0., .8, .2, 1.)
        return f"OUTSIDE SAFE ZONE ({self.side} TCP estimate)", (1., .2, .2, 1.)

    def marker(self, idx, kind, color, scale=.01):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = f'{self.side}_workspace_estimate'
        m.id = idx
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.
        m.scale.x = m.scale.y = m.scale.z = float(scale)
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def publish(self):
        # Offline robot posture is explicitly an illustration, never fake feedback for recording.
        q = self.latest or {name: 0. for name in self.model.names}
        if not self.args.hardware or self.fresh():
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name, msg.position = list(q), list(q.values())
            self.joints.publish(msg)
        self.zone.rebuild()
        dots = self.marker(0, Marker.POINTS, (1., .65, 0., 1.))
        dots.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.zone.points]
        mesh = self.marker(1, Marker.TRIANGLE_LIST, (0., .7, 1., .20), 1.)
        edges = self.marker(2, Marker.LINE_LIST, (0., .8, 1., 1.), .002)
        h = self.zone.hull
        if h is not None:
            for tri in h.simplices:
                ps = [Point(x=float(h.points[i,0]), y=float(h.points[i,1]), z=float(h.points[i,2])) for i in tri]
                # Both windings make the translucent shell visible inside and outside.
                mesh.points.extend(ps + ps[::-1])
                edges.points.extend([ps[0], ps[1], ps[1], ps[2], ps[2], ps[0]])
        else:
            mesh.action = edges.action = Marker.DELETE
        label = self.marker(3, Marker.TEXT_VIEW_FACING, (1., .5, 0., 1.), .04)
        label.pose.position.z = 1.5
        membership, color = self.membership()
        label.text = membership
        label.color.r, label.color.g, label.color.b, label.color.a = color
        self.zone_indicator.config(text=membership, fg=('#16852d' if color[0] == 0. else '#c52020' if color[1] == .2 else '#946000'))
        tip = self.marker(4, Marker.SPHERE, color, .025)
        if self.fresh():
            p = self.model.transforms(q)[self.tcp][:3, 3]
            tip.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        else: tip.action = Marker.DELETE
        self.markers.publish(MarkerArray(markers=[dots, mesh, edges, label, tip]))
        state = ('STOPPED: ' + self.error) if self.error else ('Recording' if self.recording else 'Paused')
        if not self.args.hardware: state = 'OFFLINE VIEW ONLY — robot shown at zero posture, not live'
        elif not self.fresh() and not self.error: state = 'Waiting for fresh readings from both disabled arms'
        detail = f'Volume estimate: {h.volume:.6f} m³; {len(h.vertices)} extreme vertices' if h is not None else 'No 3-D envelope yet: collect at least 4 non-coplanar points at multiple heights.'
        self.status.set(f'{state}\nSamples: {len(self.zone.points)} (5 mm spacing)\n{detail}\nBlue = inferred interior, outside = not labeled. Neither is a collision-safety guarantee.')
        self.record_button.config(text='Pause recording' if self.recording else 'Start recording')

    def tick(self):
        while not self.inbox.empty():
            kind, stamp, value = self.inbox.get_nowait()
            if kind == 'error':
                self.error = value
                self.recording = False
            elif not self.error:
                self.latest, self.latest_time = value, stamp
                if self.recording and self.fresh(): self.add_point()
        if self.recording and not self.fresh(): self.recording = False
        rclpy.spin_once(self.node, timeout_sec=0)
        self.publish()
        self.root.after(100, self.tick)

    def close(self):
        self.recording = False
        if self.zone.points and not messagebox.askyesno('Exit?', 'Exit recorder? Save first if you want to keep your current zone.'):
            return
        self.stop.set()
        if self.worker: self.worker.join(timeout=1)
        self.root.destroy()

    def run(self):
        try: self.root.mainloop()
        finally:
            self.stop.set()
            if self.worker: self.worker.join(timeout=1)
            self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--left-can', default='can1')
    parser.add_argument('--right-can', default='can0')
    parser.add_argument('--arm', choices=('left','right'), default='left')
    parser.add_argument('--load')
    args = parser.parse_args()
    rclpy.init(args=[])
    try: App(args).run()
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
