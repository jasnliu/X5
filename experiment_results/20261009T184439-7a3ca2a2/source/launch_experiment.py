#!/usr/bin/python3
"""Independent strike-lab launcher: robot model, RViz and buttons; no perception."""
import sys
from strike_lab.cli import parse
from strike_lab.config import ROOT


def main():
    args,_=parse()
    if args.headless or args.no_rviz or args.list_methods or args.rescore:
        from strike_lab.cli import main as app_main
        return app_main()
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch_ros.actions import Node
    app=ExecuteProcess(cmd=['/usr/bin/python3','-m','strike_lab.cli',*sys.argv[1:]],cwd=str(ROOT),output='screen')
    rsp=Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'robot_description':(ROOT/'model/openarmx.urdf').read_text()}])
    rviz=Node(package='rviz2',executable='rviz2',arguments=['-d',str(ROOT/'config/experiment.rviz')])
    description=LaunchDescription([rsp,rviz,app])
    for process in (rsp,rviz,app):
        description.add_action(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[
            EmitEvent(event=Shutdown(reason='Experiment component closed; app will stop/relax controller'))])))
    service=LaunchService();service.include_launch_description(description)
    return service.run()


if __name__=='__main__':sys.exit(main())
