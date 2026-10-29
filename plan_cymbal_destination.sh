#!/usr/bin/env bash
# Separate static prime destination preview. No motor control path.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ ! -x .venv-planner-experiment/bin/python || ! -x ../dt1/.venv/bin/python ]]; then
  echo 'This program reuses the existing X5 experiment and DT1 environments; one is missing.' >&2
  exit 1
fi
exec .venv-planner-experiment/bin/python -B -m cymbal_destination.cli "$@"
