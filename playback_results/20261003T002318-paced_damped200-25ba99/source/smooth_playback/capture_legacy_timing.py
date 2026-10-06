"""Capture the unchanged legacy GUI's command timing in pure simulation.

No physical CAN socket can be created. The captured command sequence can later
be replayed on the arm by the isolated center-before-relax runner.
"""
import json
from pathlib import Path
import socket
import sys
import tempfile
import time
from unittest.mock import patch
import numpy as np
import rclpy
from camera_playback.app import App
from camera_playback.simulation import SimulatedMotors
from centering.motors import RIGHT_GRIPPER_CLOSED
from .trajectory import ROOT,SOURCE


def audit(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.PF_CAN:
        raise RuntimeError('Legacy timing capture forbids physical CAN')
sys.addaudithook(audit)


def main():
    rclpy.init(args=[]);app=None;rows=[]
    with tempfile.TemporaryDirectory() as tmp:
      try:
        with patch('goal_motion.app.Motors',side_effect=AssertionError('No physical bus')):
            app=App(False,tmp+'/detections.sock',tmp+'/audio.sock',SOURCE,test_mode=True)
        deadline=time.monotonic()+30
        while app.playback_trajectory is None:
            app.root.update();time.sleep(.001)
            if time.monotonic()>deadline:raise RuntimeError('Legacy trajectory not ready')
        app.bus.close()
        app.bus=SimulatedMotors(app.playback_trajectory.first_joints,RIGHT_GRIPPER_CLOSED)
        app.bus.active=True;app.bus.poll();app.q=app.bus.positions()
        app.gripper_closed_latched=True
        original=app.bus.set_positions
        def record(q):
            if app.phase=='RECORDING PLAYBACK':
                rows.append(dict(time_s=time.monotonic()-app.playback_started_at,positions_rad=np.asarray(q).tolist()))
            original(q)
        app.bus.set_positions=record
        app._begin_playback(time.monotonic())
        while app.playback_active:
            app.root.update();time.sleep(.001)
            if time.monotonic()>deadline+20:raise RuntimeError('Legacy playback stalled')
        dt=np.diff([r['time_s'] for r in rows])
        result=dict(source='Unmodified camera_playback App Tk tick in test_mode, no CAN, no camera/audio/ESP32; all normal model/marker/UI publishing retained',
                    command_count=len(rows),duration_s=app.playback_trajectory.duration_s,
                    median_command_interval_s=float(np.median(dt)),max_command_interval_s=float(max(dt)),
                    min_command_interval_s=float(min(dt)),commands=rows)
        (ROOT/'playback_results/legacy_gui_timing.json').write_text(json.dumps(result,indent=2)+'\n')
        print({k:v for k,v in result.items() if k!='commands'})
      finally:
        if app:
            app._cancel_planning();app.planner_executor.shutdown(wait=False,cancel_futures=True)
            app.receiver.close();app.audio_receiver.close();app.bus.close();app.root.destroy();app.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
