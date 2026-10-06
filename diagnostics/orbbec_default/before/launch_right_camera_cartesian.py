#!/usr/bin/python3
"""Launch RViz, the right-arm control panel, and one processed Y2 camera window."""
import argparse
import os
from pathlib import Path
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parent
Y2 = ROOT.parent / "Y2"


def parse_args():
    parser = argparse.ArgumentParser(description="Right-arm camera-guided Cartesian search")
    parser.add_argument("--hardware", action="store_true")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--imgsz", type=int, default=None)
    parser.add_argument("--conf", type=float, default=None)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--preprocessing", choices=("local", "reference", "none"), default=None)
    parser.add_argument("--lighting-profile", type=Path, default=None)
    parser.add_argument("--no-normalization", action="store_true")
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--allow-low-light-fps", action="store_true")
    parser.add_argument("--process-every", type=int, default=1)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def optional(command, flag, value):
    if value is not None:
        command.extend([flag, str(value)])


args = parse_args()
venv_python = Y2 / ".venv/bin/python"
detector_source = Y2 / "realtime_detector.py"
if not venv_python.is_file() or not detector_source.is_file():
    raise SystemExit(f"Y2 detector environment not found under {Y2}")
socket_path = Path(f"/tmp/x5-right-camera-{os.getpid()}.sock")

app_command = ["/usr/bin/python3", "-m", "camera_search.app",
               "--detection-socket", str(socket_path)]
if args.hardware:
    app_command.append("--hardware")
camera_command = [str(venv_python), str(ROOT / "camera_search/camera.py"),
                  "--socket", str(socket_path), "--root", str(Y2),
                  "--camera", str(args.camera), "--device", args.device,
                  "--process-every", str(args.process_every)]
optional(camera_command, "--model", args.model)
optional(camera_command, "--imgsz", args.imgsz)
optional(camera_command, "--conf", args.conf)
optional(camera_command, "--preprocessing", args.preprocessing)
optional(camera_command, "--lighting-profile", args.lighting_profile)
optional(camera_command, "--width", args.width)
optional(camera_command, "--height", args.height)
optional(camera_command, "--output", args.output)
if args.no_normalization:
    camera_command.append("--no-normalization")
if args.allow_low_light_fps:
    camera_command.append("--allow-low-light-fps")

camera_env = os.environ.copy()
camera_env.pop("PYTHONPATH", None)
camera_env.pop("LD_LIBRARY_PATH", None)

app = ExecuteProcess(cmd=app_command, cwd=str(ROOT), output="screen")
camera = ExecuteProcess(cmd=camera_command, cwd=str(Y2), env=camera_env, output="screen")
rsp = Node(
    package="robot_state_publisher", executable="robot_state_publisher",
    parameters=[{"robot_description": (ROOT / "model/openarmx.urdf").read_text()}],
)
rviz = Node(package="rviz2", executable="rviz2",
            arguments=["-d", str(ROOT / "config/safe_zone.rviz")])
description = LaunchDescription([rsp, rviz, app, camera])

# Never tear power away from an arm at an arbitrary pose merely because a
# camera/RViz support process exited.  Notify the control panel so it can use
# its normal center-before-relax recovery; the operator can close the panel
# after the recovery.  Exiting the control panel itself still shuts down the
# remaining visualization processes.
for process, label in (
    (camera, "Processed camera process exited"),
    (rsp, "Robot-state publisher exited"),
    (rviz, "RViz exited"),
):
    notifier = ExecuteProcess(
        cmd=["/usr/bin/python3", "-m", "camera_search.notify",
             "--socket", str(socket_path), "--detail", label],
        cwd=str(ROOT), output="screen",
    )
    description.add_action(RegisterEventHandler(OnProcessExit(
        target_action=process, on_exit=[notifier],
    )))
description.add_action(RegisterEventHandler(OnProcessExit(
    target_action=app,
    on_exit=[EmitEvent(event=Shutdown(reason="Right camera Cartesian control exited"))],
)))
service = LaunchService()
service.include_launch_description(description)
sys.exit(service.run())
