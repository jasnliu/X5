#!/usr/bin/python3
"""Launch exactly one RViz view and one destination capture panel."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

from destination_capture.app import parse_args

ROOT = Path(__file__).resolve().parent


def display_description(side):
    """Hide unmeasured opposite-arm visuals, without editing the source URDF."""
    robot = ET.parse(ROOT / 'model/openarmx.urdf').getroot()
    other = 'left' if side == 'right' else 'right'
    for link in robot.findall('link'):
        if link.get('name', '').startswith(f'openarmx_{other}_'):
            for item in list(link):
                if item.tag in ('visual', 'collision'):
                    link.remove(item)
    return ET.tostring(robot, encoding='unicode')


def description(args, argv):
    app = ExecuteProcess(cmd=['/usr/bin/python3', '-m', 'destination_capture.app', *argv],
                         cwd=str(ROOT), output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': display_description(args.arm)}])
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', str(ROOT / f'config/{args.arm}_recording.rviz')])
    result = LaunchDescription()
    for process in (app, rsp, rviz):
        # Register before starting so even an immediate startup failure is handled.
        result.add_action(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Destination capture component exited'))])))
    for process in (rsp, rviz, app):
        result.add_action(process)
    return result


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    args = parse_args(argv)  # help/errors do not launch windows
    service = LaunchService()
    service.include_launch_description(description(args, argv))
    return service.run()


if __name__ == '__main__':
    sys.exit(main())
