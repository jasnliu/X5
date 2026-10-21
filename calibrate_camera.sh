#!/usr/bin/env bash
# Isolated calibration utility: no ROS launch, CAN setup, torque or gripper action.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec /usr/bin/python3 -m camera_calibration.cli "$@"
