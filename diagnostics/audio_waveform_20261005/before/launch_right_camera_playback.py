#!/usr/bin/python3
"""Launch the separate recorded-path camera-alignment program."""
import argparse
import os
from pathlib import Path
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

from camera_playback.hihat import DEFAULT_ESP_PORT
from camera_playback.audio_bridge import ST7_MODEL_VERSION, HIHAT_MODEL_VERSION, detector_command
from camera_search.device import add_camera_argument
from camera_playback.camera_selection import choose_camera, list_cameras, print_cameras


ROOT = Path(__file__).resolve().parent
Y2 = ROOT.parent / "Y2"
# Use the sibling versioned model, never a legacy checkout or implicit v1 default.
ST7 = ROOT.parent / "st7"
DEFAULT_TONOR_SOURCE = (
    "alsa_input.usb-TONOR_TONOR_TD510_Dynamic_Mic_0000KT5a300000135-00.analog-stereo"
)


def parse_args():
    parser = argparse.ArgumentParser(description="Right recorded-path camera alignment")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--hardware", action="store_true",
        help="automatic hi-hat-v1 calibration 90..115 degrees; one RUN: center + close, recording, wait for hi-hat, ride-v2 search, 100 BPM swing (camera/TONOR/ESP32)",
    )
    mode.add_argument(
        "--test", action="store_true",
        help=("pure simulation with no CAN, camera, microphone, or ESP32; "
              "animate playback and fixed 10-degree strikes in RViz"),
    )
    mode.add_argument(
        "--hardwaretest", action="store_true",
        help=("run center/open/load, recorded playback, and single-frame pink-zone "
              "alignment on the physical right arm; then run one editable "
              "damped J7 freefall, predictive catch and smooth MIT return per press (no ESP32)"),
    )
    parser.add_argument(
        "--recording", type=Path, default=ROOT / "recordings/record1.json",
        help="right-arm recording (default: recordings/record1.json)",
    )
    camera_choice = parser.add_mutually_exclusive_group()
    add_camera_argument(camera_choice)
    camera_choice.add_argument('--select-camera', action='store_true',
                               help='choose a connected camera interactively before any controllers launch')
    parser.add_argument('--list-cameras', action='store_true',
                        help='list connected camera names, indices and stable paths, then exit without launching hardware')
    parser.add_argument("--recording-only", action="store_true",
                        help="physical no-strike test: recording, verified center, then relax; never align or strike")
    parser.add_argument("--auto-run", action="store_true",
                        help="requires --recording-only: Run automatically, close gripper after 5 s loading pause, exit after safe relax")
    parser.add_argument('--verify-swing', action='store_true',
                        help='AUTHORIZED MOTION: auto RUN, verify 5 s physical swing, center/relax, save evidence')
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--imgsz", type=int, default=None)
    parser.add_argument("--conf", type=float, default=None)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--preprocessing", choices=("local", "reference", "none"), default=None)
    parser.add_argument("--lighting-profile", type=Path, default=None)
    parser.add_argument("--no-normalization", action="store_true")
    parser.add_argument("--width", type=int, default=None,
                        help="RGB width (Orbbec full-FOV default: 1280; omitted partner dimension inferred as 16:9)")
    parser.add_argument("--height", type=int, default=None,
                        help="RGB height (Orbbec full-FOV default: 720; cropped modes are rejected)")
    parser.add_argument("--allow-low-light-fps", action="store_true")
    parser.add_argument("--process-every", type=int, default=1)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--audio-source", default=DEFAULT_TONOR_SOURCE,
        help="stable PipeWire/PulseAudio TONOR TD510 capture-source name",
    )
    parser.add_argument(
        "--audio-device", default="pipewire",
        help="sounddevice input used by ST7 after selecting the TONOR source",
    )
    parser.add_argument(
        "--esp-port", default=DEFAULT_ESP_PORT,
        help="ESP32 motor_beat serial port (default: connected CP2102 by-id path)",
    )
    parser.add_argument("--mit-fall-kd", type=float, default=None,
                        help="hardwaretest only: gravity-descent damping (default 0.04)")
    parser.add_argument("--mit-inertia", type=float, default=None,
                        help="hardwaretest only: loaded J7 inertia, kg m^2 (default 0.02)")
    parser.add_argument("--mit-brake-accel", type=float, default=None,
                        help="hardwaretest only: calibrated catch acceleration, rad/s^2 (default 40)")
    parser.add_argument("--mit-latency-ms", type=float, default=None,
                        help="hardwaretest only: command/actuation delay estimate, ms (default 4)")
    args = parser.parse_args()
    if args.test and args.select_camera:
        parser.error('--select-camera is unavailable in --test, which deliberately uses no camera')
    if args.recording_only and not (args.hardware or args.hardwaretest):
        parser.error("--recording-only requires --hardware or --hardwaretest")
    if args.auto_run and not args.recording_only:
        parser.error("--auto-run requires --recording-only")
    if args.verify_swing and (not args.hardware or args.recording_only):
        parser.error('--verify-swing requires normal --hardware')
    if not args.hardwaretest and any(getattr(args, name) is not None for name in (
            "mit_fall_kd", "mit_inertia", "mit_brake_accel", "mit_latency_ms")):
        parser.error("--mit-* tuning flags require --hardwaretest")
    return args


def optional(command, flag, value):
    if value is not None:
        command.extend([flag, str(value)])


args = parse_args()
if args.list_cameras:
    print_cameras(list_cameras())
    raise SystemExit(0)
if args.select_camera:
    try:
        args.camera = choose_camera()
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None
perception_enabled = not args.test
venv_python = Y2 / ".venv/bin/python"
detector_source = Y2 / "realtime_detector.py"
if perception_enabled and (not venv_python.is_file() or not detector_source.is_file()):
    raise SystemExit(f"Y2 detector environment not found under {Y2}")
if perception_enabled:
    try:
        # Share the bridge's exact preflight; this does not start the listener.
        detector_command(ST7, args.audio_device)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None
    print(f"ST7 audio model {ST7_MODEL_VERSION}: {ST7 / 'models' / ST7_MODEL_VERSION}", flush=True)
    # This second detector is optional feedback, never a motion-readiness gate.
    try:
        detector_command(ST7, args.audio_device, 'hihat')
    except RuntimeError as exc:
        print(f"Hi-hat sound monitor unavailable (required for normal hardware calibration): {exc}", flush=True)
    hihat_purpose = ('startup calibration and hi-hat-only timing'
                     if args.hardware and not args.recording_only else 'visual only')
    print(f"ST7 hi-hat audio model {HIHAT_MODEL_VERSION}: {hihat_purpose}; "
          "ride v2 remains the only strike-search input", flush=True)
socket_path = Path(f"/tmp/x5-right-camera-playback-{os.getpid()}.sock")
audio_socket_path = Path(f"/tmp/x5-right-camera-audio-{os.getpid()}.sock")
hihat_audio_socket_path = Path(f"/tmp/x5-right-hihat-audio-{os.getpid()}.sock")

app_command = [
    "/usr/bin/python3", "-m", "camera_playback.app",
    "--detection-socket", str(socket_path),
    "--audio-socket", str(audio_socket_path),
    "--hihat-audio-socket", str(hihat_audio_socket_path),
    "--esp-port", args.esp_port,
]
if args.hardware:
    app_command.append("--hardware")
if args.test:
    app_command.append("--test")
if args.hardwaretest:
    app_command.append("--hardwaretest")
    for flag in ("mit-fall-kd", "mit-inertia", "mit-brake-accel", "mit-latency-ms"):
        optional(app_command, "--" + flag, getattr(args, flag.replace("-", "_")))
optional(app_command, "--recording", args.recording)
if args.recording_only:
    app_command.append("--recording-only")
if args.auto_run:
    app_command.append("--auto-run")
if args.verify_swing:
    app_command.append('--verify-swing')
camera_command = [
    str(venv_python), str(ROOT / "camera_playback/camera.py"),
    "--socket", str(socket_path), "--root", str(Y2),
    "--camera", str(args.camera), "--device", args.device,
    "--process-every", str(args.process_every),
]
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
audio_command = [
    "/usr/bin/python3", "-m", "camera_playback.audio_bridge",
    "--socket", str(audio_socket_path),
    "--root", str(ST7),
    "--source", args.audio_source,
    "--device", args.audio_device,
]
hihat_audio_command = [
    "/usr/bin/python3", "-m", "camera_playback.audio_bridge",
    "--socket", str(hihat_audio_socket_path),
    "--root", str(ST7), "--instrument", "hihat",
    "--source", args.audio_source, "--device", args.audio_device,
]

camera_env = os.environ.copy()
camera_env.pop("PYTHONPATH", None)
camera_env.pop("LD_LIBRARY_PATH", None)
audio_env = os.environ.copy()
audio_env.pop("PYTHONPATH", None)
audio_env.pop("LD_LIBRARY_PATH", None)

app = ExecuteProcess(cmd=app_command, cwd=str(ROOT), output="screen",
                     sigterm_timeout='45', sigkill_timeout='45')
camera = ExecuteProcess(cmd=camera_command, cwd=str(Y2), env=camera_env, output="screen")
audio = ExecuteProcess(cmd=audio_command, cwd=str(ROOT), env=audio_env, output="screen")
hihat_audio = ExecuteProcess(cmd=hihat_audio_command, cwd=str(ROOT), env=audio_env, output="screen")
rsp = Node(
    package="robot_state_publisher", executable="robot_state_publisher",
    parameters=[{"robot_description": (ROOT / "model/openarmx.urdf").read_text()}],
)
rviz = Node(
    package="rviz2", executable="rviz2",
    arguments=["-d", str(ROOT / "config/safe_zone.rviz")],
)
processes = [rsp, rviz, app]
if perception_enabled:
    processes.extend([camera, audio, hihat_audio])
description = LaunchDescription(processes)
watched_processes = [
    (rsp, "Robot-state publisher exited"),
    (rviz, "RViz exited"),
]
if perception_enabled:
    watched_processes.insert(0, (camera, "Processed camera process exited"))
for process, label in watched_processes:
    notifier = ExecuteProcess(
        cmd=[
            "/usr/bin/python3", "-m", "camera_search.notify",
            "--socket", str(socket_path), "--detail", label,
        ],
        cwd=str(ROOT), output="screen",
    )
    description.add_action(RegisterEventHandler(OnProcessExit(
        target_action=process, on_exit=[notifier],
    )))
if perception_enabled:
    description.add_action(RegisterEventHandler(OnProcessExit(
        target_action=audio,
        on_exit=[ExecuteProcess(
            cmd=[
                "/usr/bin/python3", "-m", "camera_playback.audio_notify",
                "--socket", str(audio_socket_path),
                "--detail", "ST7/TONOR sound detector process exited",
            ],
            cwd=str(ROOT), output="screen",
        )],
    )))
    # Notify the hi-hat receiver, not the ride receiver. Normal hardware
    # calibration/synchronization owns the resulting supervised stop policy.
    description.add_action(RegisterEventHandler(OnProcessExit(
        target_action=hihat_audio,
        on_exit=[ExecuteProcess(
            cmd=[
                "/usr/bin/python3", "-m", "camera_playback.audio_notify",
                "--socket", str(hihat_audio_socket_path), "--instrument", "hihat",
                "--detail", "Hi-hat sound detector exited",
            ],
            cwd=str(ROOT), output="screen",
        )],
    )))
description.add_action(RegisterEventHandler(OnProcessExit(
    target_action=app,
    on_exit=[EmitEvent(event=Shutdown(reason="Right camera playback control exited"))],
)))
service = LaunchService()
service.include_launch_description(description)
sys.exit(service.run())
