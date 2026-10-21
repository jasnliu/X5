"""Mocked live status and UI checks. No CAN transport is permitted."""
import argparse
import itertools
import tempfile
import time
from pathlib import Path
from unittest.mock import patch, Mock
import rclpy
from safe_zone.app import App
from safe_zone.geometry import Zone, TCP
rclpy.init(args=[])
with patch('safe_zone.app.Observer', side_effect=AssertionError('CAN forbidden')):
    app=App(argparse.Namespace(hardware=False,load=None,left_can='can1',right_can='can0',arm='left'))
    app.root.withdraw()
    app.latest={n:0. for n in app.model.names}
    p=app.model.transforms(app.latest)[TCP][:3,3]
    for offsets in itertools.product((-.1,.1),repeat=3):app.zone.add(p+offsets)
    app.zone.rebuild()
    app.args.hardware=True
    app.latest_time=time.monotonic()
    assert 'save or import' in app.membership()[0]
    with tempfile.TemporaryDirectory() as tmp:
        app.zone_dir=Path(tmp)
        path=str(Path(tmp)/'zone.json')
        with patch('safe_zone.app.filedialog.asksaveasfilename',return_value=path), patch('safe_zone.app.messagebox.showinfo'):
            app.save()
        assert app.zone_committed
        assert app.membership()[0].startswith('INSIDE')
        app.publish()
        assert app.zone_indicator.cget('text').startswith('INSIDE')
        # Resting TCP 4 mm below the original bottom face: only status changes.
        frames = app.model.transforms(app.latest)
        frames[TCP] = frames[TCP].copy()
        frames[TCP][2,3] = p[2] - .104
        publisher = Mock()
        with patch.object(app.model, 'transforms', return_value=frames), patch.object(app, 'markers', publisher):
            with patch('safe_zone.app.MEMBERSHIP_BUFFER_M', 0.):
                app.publish()
                strict = publisher.publish.call_args.args[0]
                assert app.zone_indicator.cget('text').startswith('OUTSIDE')
            app.publish()
            buffered = publisher.publish.call_args.args[0]
            assert app.zone_indicator.cget('text').startswith('INSIDE')
            for before, after in zip(strict.markers[:3], buffered.markers[:3]):
                assert before.points == after.points
                assert before.pose == after.pose and before.scale == after.scale
                assert before.color == after.color and before.action == after.action
            frames[TCP][2,3] = p[2] - .106
            assert app.membership()[0].startswith('OUTSIDE')
        app.latest['openarmx_left_joint2']=1.
        assert app.membership()[0].startswith('OUTSIDE')
        app.publish()
        assert app.zone_indicator.cget('text').startswith('OUTSIDE')
        app.latest['openarmx_left_joint2']=0.
        app.recording=True
        assert app.membership()[0].startswith('RECORDING')
        app.recording=False
        app.latest_time=time.monotonic()-1
        assert 'no fresh' in app.membership()[0]
        app.latest_time=time.monotonic()
        app.error='test fault'
        assert 'encoder fault' in app.membership()[0]
        app.error=None
        app.args.hardware=False
        assert 'no fresh' in app.membership()[0]
        app.args.hardware=True
        with patch('safe_zone.app.messagebox.askyesno',return_value=True):app.clear()
        assert not app.zone_committed
        with patch('safe_zone.app.filedialog.askopenfilename',return_value=path):app.load()
        assert app.zone_committed and app.membership()[0].startswith('INSIDE')
        app.add_point()
        assert not app.zone_committed
        with patch('safe_zone.app.filedialog.askopenfilename',return_value=path):app.load()
        with patch('safe_zone.app.filedialog.asksaveasfilename',return_value=''):
            app.save()
        assert app.zone_committed
        app.zone=Zone(app.model.digest)
        assert 'no 3-D volume' in app.membership()[0]
    app.root.destroy();app.node.destroy_node()
rclpy.shutdown()
print('PASS: inside/outside UI, save/import gating, recording, stale/fault/offline, clear, dirty and degenerate zones; no CAN')
