"""Offline left/right Cartesian UI selection; motor transport is forbidden."""
import json
import numpy as np
from unittest.mock import patch
import rclpy
from cartesian_goal.app import App
from safe_zone.geometry import LEFT_TCP,RIGHT_TCP,MEMBERSHIP_BUFFER_M

rclpy.init(args=[])
results=[]
try:
    with patch('goal_motion.app.Motors',side_effect=AssertionError('Motor transport forbidden')):
        for side,tcp in (('left',LEFT_TCP),('right',RIGHT_TCP)):
            app=App(False,side);app.root.withdraw()
            assert app.bus is None and app.side==side and app.tcp_frame==tcp and app.zone.tcp==tcp
            assert side.upper() in app.root.title() and side.upper() in app.header.cget('text')
            assert 'Gripper (motor 8):' in app.encoder_text()
            np.testing.assert_allclose(app.ik.origin,np.zeros(7))
            np.testing.assert_allclose(app.ik.origin_tcp,app.model.transforms({})[tcp][:3,3])
            np.testing.assert_allclose(app.target_point,app.ik.origin_tcp+[0,0,.05])
            if side=='left':np.testing.assert_allclose(app.center_goal,np.zeros(7))
            else:
                np.testing.assert_allclose(app.center_goal[:6],np.zeros(6))
                assert app.center_goal[6]==app.upper[6]==1.4
                assert app.zone.contains(app.planned_tcp(app.center_goal),MEMBERSHIP_BUFFER_M)
            assert app.control_gripper==(side=='right')
            assert (app.continue_button is not None)==(side=='right')
            assert (app.update_button is not None)==(side=='right')
            assert (app.finish_button is not None)==(side=='right')
            if side=='right':
                assert 'UPDATE:' in app.update_button.cget('text')
                assert 'END:' in app.finish_button.cget('text')
            results.append({'side':side,'title':app.root.title(),'tcp':tcp,'zone_points':len(app.zone.points),
                            'manual_finish':app.manual_finish,
                            'center_degrees':np.degrees(app.center_goal).tolist(),
                            'origin_tcp_m':app.ik.origin_tcp.tolist(),
                            'default_preview_m':app.target_point.tolist(),
                            'center_inside_buffered_zone':app.zone.contains(app.planned_tcp(app.center_goal),MEMBERSHIP_BUFFER_M)})
            app.root.destroy();app.node.destroy_node()
finally:
    rclpy.shutdown()
print(json.dumps(results,indent=2))
print('PASS: distinct left/right Cartesian UI, TCP, zone and center configuration; no CAN or movement')
