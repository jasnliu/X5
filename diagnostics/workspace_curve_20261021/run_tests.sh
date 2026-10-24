#!/usr/bin/env bash
# Device-free regression suite; does not start any controller/camera.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
exec .venv-planner-experiment/bin/python -m pytest -q \
 tests/test_workspace_curve.py tests/test_planner_experiment.py \
 tests/test_planner_gesture.py tests/test_experiment_destinations.py \
 tests/test_singlearc_arms.py tests/test_single_arc_priority.py \
 tests/test_planner_performance.py tests/test_corner_smoothing.py \
 tests/test_timing.py tests/test_planner_arms.py tests/test_planner_zone.py \
 tests/test_voxel_clearance.py tests/test_baseline_single_arc.py \
 tests/test_repository_artifacts.py
