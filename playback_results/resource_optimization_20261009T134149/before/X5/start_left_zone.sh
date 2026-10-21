#!/usr/bin/env bash
set -e
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Query-only recorder; isolated from motion-controller ROS graphs.
export ROS_DOMAIN_ID=85
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec /usr/bin/python3 launch_viewer.py "$@" --arm left
