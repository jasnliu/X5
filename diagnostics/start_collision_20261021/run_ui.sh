#!/usr/bin/env bash
# Saved scene + simulated motors only; never runs normal hardware startup.
set -e
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
root=planner_runs/start_collision_20261021
plan="$root/calculation/plan/result/plan.json"
/usr/bin/python3 -m diagnostics.baseline_single_arc_20261021.verify_ui "$root/ui" "$plan"
./plan_drums.sh baseline --simulate "$plan" --seconds 3 --output "$root/cli_simulation"
