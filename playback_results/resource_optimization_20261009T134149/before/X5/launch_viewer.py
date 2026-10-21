#!/usr/bin/python3
"""Launch only robot_state_publisher, RViz, and the query-only recorder."""
import sys
from pathlib import Path
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

root = Path(__file__).resolve().parent
args = sys.argv[1:]
app = ExecuteProcess(cmd=['/usr/bin/python3', '-m', 'safe_zone.app', *args], cwd=str(root), output='screen')
rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': (root/'model/openarmx.urdf').read_text()}])
rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', str(root/'config/safe_zone.rviz')])
ld = LaunchDescription([rsp, rviz, app])
for process in (app, rsp, rviz):
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=process, on_exit=[EmitEvent(event=Shutdown(reason='Viewer component exited'))])))
service = LaunchService()
service.include_launch_description(ld)
sys.exit(service.run())
