#!/usr/bin/python3
"""Launch RViz plus the offline left/right recording editor panel."""
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


root = Path(__file__).resolve().parent
app = ExecuteProcess(
    cmd=["/usr/bin/python3", "-m", "motion_recording.editor_app", *sys.argv[1:]],
    cwd=str(root),
    output="screen",
)
rsp = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description": (root / "model/openarmx.urdf").read_text()}],
)
rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", str(root / "config/recording_editor.rviz")],
)
description = LaunchDescription([rsp, rviz, app])
for process in (app, rsp, rviz):
    description.add_action(RegisterEventHandler(OnProcessExit(
        target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason="Recording editor component exited"))],
    )))
service = LaunchService()
service.include_launch_description(description)
sys.exit(service.run())
