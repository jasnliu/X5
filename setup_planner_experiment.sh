#!/usr/bin/env bash
# Project-local Python dependencies; never starts camera or connects to CAN.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
/usr/bin/python3 -m venv --system-site-packages .venv-planner-experiment
.venv-planner-experiment/bin/pip install -r drum_planner/experiment/requirements.txt
source /opt/ros/jazzy/setup.bash
.venv-planner-experiment/bin/python -c 'from drum_planner.experiment.runtime import Job; from drum_planner.experiment.app import command,wait; from drum_planner.common import ROOT; wait(Job(command("build",ROOT/"drum_planner/build/experiment"),ROOT/"drum_planner/build/experiment/setup_job",timeout=240))'
