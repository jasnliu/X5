#!/usr/bin/env bash
# All runs use saved scenes; the baseline UI uses simulated motor owners only.
set -e
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
root=planner_runs/single_arc_priority_20261021
python=/usr/bin/python3
"$python" -m diagnostics.baseline_single_arc_20261021.verify_ui \
  "$root/baseline_ui" "$root/baseline/plan.json"
"$python" -m diagnostics.arm_experiment_modes_20261021.verify_arm_ui "$root/experiment_right"
"$python" -m diagnostics.arm_experiment_modes_20261021.verify_arm_ui "$root/experiment_left"
./plan_drums.sh baseline --simulate "$root/baseline/plan.json" --seconds 3 --output "$root/cli_simulation"
