"""Offline left/right recorder integration; CAN transport is forbidden."""
import argparse
import tempfile
import time
from pathlib import Path
from unittest.mock import patch
import numpy as np
import rclpy
from safe_zone.app import App
from safe_zone.geometry import LEFT_TCP,RIGHT_TCP

rclpy.init(args=[])
try:
    with patch('safe_zone.app.Observer',side_effect=AssertionError('CAN forbidden')):
        for side,tcp in (('left',LEFT_TCP),('right',RIGHT_TCP)):
            app=App(argparse.Namespace(hardware=False,load=None,left_can='can1',right_can='can0',arm=side))
            app.root.withdraw()
            assert app.side==side and app.tcp==tcp and app.zone.tcp==tcp
            assert side.upper() in app.root.title()
            app.args.hardware=True
            app.latest={name:0. for name in app.model.names}
            app.latest_time=time.monotonic()
            expected=app.model.transforms(app.latest)[tcp][:3,3]
            app.add_point();np.testing.assert_allclose(app.zone.points[0],expected)
            with tempfile.TemporaryDirectory() as tmp:
                app.zone_dir=Path(tmp)/f'{side}_zones';app.zone_dir.mkdir()
                selected=Path(tmp)/'somewhere_else'/f'{side}_test'
                with patch('safe_zone.app.filedialog.asksaveasfilename',return_value=str(selected)) as save_dialog,patch('safe_zone.app.messagebox.showinfo'):
                    app.save()
                actual=app.zone_dir/f'{side}_test.json'
                assert Path(save_dialog.call_args.kwargs['initialdir'])==app.zone_dir
                assert actual.exists() and not selected.exists()
                text=actual.read_text();assert f'openarmx-{side}-envelope-v1' in text and tcp in text
                with patch('safe_zone.app.filedialog.askopenfilename',return_value=str(actual)) as load_dialog:
                    app.load()
                assert Path(load_dialog.call_args.kwargs['initialdir'])==app.zone_dir and app.zone.tcp==tcp
            app.root.destroy();app.node.destroy_node()
finally:
    rclpy.shutdown()
print('PASS: separate left/right TCP recording and forced arm-specific save directories; no CAN')
