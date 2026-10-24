#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
for name in verified1 verified2 verified3 verified-cpu; do
 extra=(); [[ "$name" != verified-cpu ]] || extra=(--cpu-only)
 .venv-planner-experiment/bin/python diagnostics/fast_gpu_20261024/run_job.py diagnostics/fast_gpu_20261024/$name-job diagnostics/fast_gpu_20261024/cold_case.py --context planner_runs/20261024T002242Z-experiment-c440af/context.json --output diagnostics/fast_gpu_20261024/$name "${extra[@]}"
done
