#!/usr/bin/env bash
# Native Qt/VTK + independent fake owners only. Never run a physical startup.
set -e
cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
for side in right left; do
  root="planner_runs/singlearc_arms_20261021/$side"
  plan="$root/calculation/plan/result/plan.json"
  /usr/bin/python3 -m diagnostics.singlearc_arms_20261021.verify_ui "$root/ui" "$plan" "$side"
  ./plan_drums.sh singlearc "--$side" --simulate "$plan" --seconds 3 --output "$root/cli_simulation"
done
