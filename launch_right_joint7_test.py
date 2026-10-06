#!/usr/bin/python3
"""Launch RViz and the isolated right-J7 ten-degree test panel."""
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parent
app = ExecuteProcess(
    cmd=["/usr/bin/python3", "-m", "right_joint7_test.app", *sys.argv[1:]],
    cwd=str(ROOT),
    output="screen",
)
rsp = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description": (ROOT / "model/openarmx.urdf").read_text()}],
)
rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", str(ROOT / "config/safe_zone.rviz")],
)
description = LaunchDescription([rsp, rviz, app])
for process in (app, rsp, rviz):
    description.add_action(RegisterEventHandler(OnProcessExit(
        target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason="Right-J7 test component exited"))],
    )))
service = LaunchService()
service.include_launch_description(description)
sys.exit(service.run())
