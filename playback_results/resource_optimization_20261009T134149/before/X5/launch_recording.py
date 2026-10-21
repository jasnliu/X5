#!/usr/bin/python3
"""Launch RViz plus the query-only left/right motion recorder panel."""
import argparse
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


root = Path(__file__).resolve().parent
selector = argparse.ArgumentParser(add_help=False)
mode = selector.add_mutually_exclusive_group(required=not any(
    flag in sys.argv[1:] for flag in ("-h", "--help")
))
mode.add_argument("--right", dest="arm", action="store_const", const="right")
mode.add_argument("--left", dest="arm", action="store_const", const="left")
selected, _ = selector.parse_known_args()
arm = selected.arm or "right"
app = ExecuteProcess(
    cmd=["/usr/bin/python3", "-m", "motion_recording.app", *sys.argv[1:]],
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
    arguments=["-d", str(root / f"config/{arm}_recording.rviz")],
)
description = LaunchDescription([rsp, rviz, app])
for process in (app, rsp, rviz):
    description.add_action(RegisterEventHandler(OnProcessExit(
        target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason=f"{arm.title()} motion recorder component exited"))],
    )))
service = LaunchService()
service.include_launch_description(description)
sys.exit(service.run())
