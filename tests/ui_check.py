"""Offline panel integration. Fake encoder values only; CAN sockets forbidden."""
import argparse
import tempfile
import time
from pathlib import Path
from unittest.mock import patch
import rclpy
from safe_zone.app import App
rclpy.init(args=[])
with patch('safe_zone.app.Observer', side_effect=AssertionError('CAN forbidden')):
    app=App(argparse.Namespace(hardware=False,load=None,left_can='can1',right_can='can0',arm='left'))
    app.root.withdraw()
    with patch('safe_zone.app.messagebox.showwarning') as warning:
        app.toggle()
        assert not app.recording and warning.called
    # Feed known fake joint values directly; never start a worker.
    app.args.hardware=True
    app.latest={n:0. for n in app.model.names}
    app.latest_time=time.monotonic()
    app.toggle();assert app.recording
    app.add_point(); assert len(app.zone.points)==1
    for i in range(1,8):
        app.latest[f'openarmx_left_joint{i}']=.3
        app.add_point()
    app.publish()
    assert len(app.zone.points)>3
    with tempfile.TemporaryDirectory() as tmp:
        app.zone_dir=Path(tmp)
        path=str(Path(tmp)/'zone.json')
        with patch('safe_zone.app.filedialog.asksaveasfilename',return_value=path), patch('safe_zone.app.messagebox.showinfo'):
            app.save()
        assert not app.recording and Path(path).exists()
        points=app.zone.points.copy()
        with patch('safe_zone.app.messagebox.askyesno',return_value=True):app.clear()
        assert not app.zone.points
        with patch('safe_zone.app.filedialog.askopenfilename',return_value=path):app.load()
        assert app.zone.points==points and not app.recording
    app.recording=True
    app.latest_time=time.monotonic()-1
    app.tick()
    assert not app.recording
    app.offer(('error',time.monotonic(),'TEST FAULT'))
    app.tick()
    assert app.error=='TEST FAULT' and not app.fresh()
    app.root.destroy();app.node.destroy_node()
rclpy.shutdown()
print('PASS: offline block, mock recording, save/import, clear, stale and fault handling; no CAN')
