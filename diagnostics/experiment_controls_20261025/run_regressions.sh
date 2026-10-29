#!/usr/bin/env bash
# Software-only. The five historical hash fixtures are documented, not rewritten.
set -eo pipefail
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  .venv-planner-experiment/bin/python -m pytest -q \
  tests/test_experiment_motion_controls.py \
  tests/test_experiment_acquisition.py \
  tests/test_experiment_destinations.py \
  tests/test_experiment_checkpoints.py \
  tests/test_checkpoint_performance.py \
  tests/test_experiment_fast_timing.py \
  tests/test_planner_experiment.py \
  tests/test_fast_mode.py \
  tests/test_fast_gpu.py::test_only_new_experiment_menu_and_bypass \
  tests/test_fast_gpu.py::test_world_reuse_is_exact_except_selected_start_goal \
  tests/test_fast_gpu.py::test_stronger_proposals_are_lazy_and_not_repeated \
  tests/test_fast_gpu.py::test_invalid_endpoint_never_reaches_fitting_or_timing \
  tests/test_fast_gpu.py::test_protected_hardware_and_original_policies_unchanged \
  -k 'not hardware_timing_generators_and_saved_data_unchanged and not single_leg_worker_and_pool_identity_unchanged_without_checkpoint_fields and not policies_geometry_inputs_and_motor_control_are_unchanged and not only_experiment_sources_changed_since_this_task and not protected_hardware_and_original_policies_unchanged'
