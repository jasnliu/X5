#!/usr/bin/env bash
# Only the optional sparse fork; do not upgrade the original planner's packages.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -x .venv-planner-experiment/bin/python ]]; then
    echo 'First run ./setup_planner_experiment.sh to create the planner environment.' >&2
    exit 1
fi
.venv-planner-experiment/bin/python -m pip install --no-deps casadi==3.7.2
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv-planner-experiment/bin/python - <<'PY'
import casadi as ca
x=ca.MX.sym('x')
solver=ca.nlpsol('check','ipopt',{'x':x,'f':(x-1)**2},
    {'print_time':False,'ipopt.print_level':0,'ipopt.sb':'yes'})
assert abs(float(solver(x0=0)['x'])-1)<1e-8
print('Sparse IPOPT dependency ready; no camera or motor access.')
PY
