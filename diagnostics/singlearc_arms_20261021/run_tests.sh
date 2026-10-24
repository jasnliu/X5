#!/usr/bin/env bash
# Software-only regression suite; no actuator or camera integration runs.
set -e
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
exec .venv-planner-experiment/bin/python -m pytest -q \
 tests/test_singlearc_arms.py tests/test_repository_artifacts.py tests/test_planner_arms.py tests/test_left_tool.py \
 tests/test_timing.py tests/test_corner_smoothing.py tests/test_baseline_single_arc.py tests/test_single_arc_priority.py \
 tests/test_drum_planner.py tests/test_planner_execution.py tests/test_planner_experiment.py \
 tests/test_planner_gesture.py tests/test_planner_optimization.py tests/test_planner_style.py tests/test_planner_zone.py \
 tests/test_baseline_hardware.py tests/test_baseline_ui.py tests/test_baseline_workspace.py tests/test_baseline_computation.py tests/test_voxel_clearance.py \
 tests/test_live_depth_view.py tests/test_live_trajectory_preview.py tests/test_snapshot_mesh.py \
 tests/test_vtk_view.py tests/test_robot_visuals.py \
 tests/test_camera_calibration.py tests/test_camera_calibration_kinematics.py \
 tests/test_smooth_recording.py tests/test_recording_cache.py tests/test_dual_recording.py \
 tests/test_readonly_hardware.py tests/test_current_plan.py
