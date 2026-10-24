#!/usr/bin/env bash
set -e
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Isolated from recording/playback; encoder requests only with --hardware.
export ROS_DOMAIN_ID=94
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec /usr/bin/python3 launch_destination.py "$@"
