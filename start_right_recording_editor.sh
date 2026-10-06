#!/usr/bin/env bash
set -e
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Offline JSON editor and RViz preview only. This launcher never opens CAN.
export ROS_DOMAIN_ID=92
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec /usr/bin/python3 launch_right_recording_editor.py "$@"
