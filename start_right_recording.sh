#!/usr/bin/env bash
set -e
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Query-only right-arm recorder. No controller or motor-enable process is launched.
export ROS_DOMAIN_ID=91
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec /usr/bin/python3 launch_right_recording.py "$@"
