#!/usr/bin/env bash
# Separate OFFLINE operating-envelope study. Does not access or move the robot.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
exec .venv-planner-experiment/bin/python -m joint_limit_lab.cli "$@"
