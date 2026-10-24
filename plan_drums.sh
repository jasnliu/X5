#!/usr/bin/env bash
# Camera/MoveIt planner. Only explicit --hardware Start / live --execute permits motor control.
# Never configures CAN interfaces or launches vendor ROS hardware controllers.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 -m drum_planner.cli "$@"
