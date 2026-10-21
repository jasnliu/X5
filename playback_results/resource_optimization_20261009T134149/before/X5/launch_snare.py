#!/usr/bin/python3
"""Left-arm (J6) snare-strike launcher.

No RViz/robot_state_publisher visualization is wired up yet (unlike
launch_experiment.py's non-headless mode) -- this always runs headless,
printing phase transitions to stdout. See snare_lab/runner.py.
"""
import sys

from snare_lab.cli import main


if __name__ == '__main__':
    sys.exit(main())
