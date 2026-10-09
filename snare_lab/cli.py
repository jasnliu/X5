"""CLI for the standalone left-arm (J6) snare-strike test program.

No ROS/RViz launch is needed here (headless-only for now -- see runner.py's
module docstring); snare.sh calls this directly.
"""
import argparse
from pathlib import Path
import re
import sys

from .config import LEFT_RECORDING, validate_depth


class StrikeDepthAction(argparse.Action):
    def __call__(self, parser, namespace, value, option_string=None):
        if namespace.degrees is not None:
            parser.error('Specify only one strike depth (--11.5 or --degrees 11.5)')
        namespace.degrees = value


def parser():
    p = argparse.ArgumentParser(
        description='Standalone left-arm (J6) snare strike test (simulation by default).',
        epilog='Depth shorthand: --10, --11, --11.5, etc. These mean positive '
               'J6 movement beyond the recording endpoint, not negative angles. '
               'Omit the depth for +10 degrees. Maximum depth is the remaining '
               'positive left-J6 rotation from the selected recording endpoint; '
               'path and dynamic safety checks still apply.')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--hardware', action='store_true',
                      help='AUTHORIZED MOTION: drive the real physical left arm. '
                           'Never run without explicit, supervised authorization.')
    mode.add_argument('--simulate', action='store_true',
                      help='Synthetic dynamics only; no CAN access (default)')
    p.add_argument('--recording', type=Path, default=LEFT_RECORDING,
                   help='left-arm recording (default: left_recordings/record1.json)')
    p.add_argument('--degrees', type=float, action=StrikeDepthAction,
                   help='J6 strike depth beyond the recording endpoint (default: 10 degrees)')
    p.add_argument('--fast', action='store_true',
                   help='Simulation only: skip realtime pacing/sleeps')
    return p


def parse(argv=None):
    p = parser()
    argv = list(sys.argv[1:] if argv is None else argv)
    # The two dashes introduce an option, not a minus sign. Reuse the
    # existing positive-depth validation and runner argument unchanged.
    for index, token in enumerate(argv):
        if token == '--':
            break
        if re.fullmatch(r'--(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)', token):
            argv[index] = '--degrees=' + token[2:]
    a = p.parse_args(argv)
    if a.degrees is None:
        a.degrees = 10.
    if a.fast and a.hardware:
        p.error('--fast requires --simulate (no --hardware)')
    try:
        validate_depth(a.degrees)
    except ValueError as exc:
        p.error(str(exc))
    return a


def main(argv=None):
    args = parse(argv)
    from . import runner
    if args.hardware:
        return runner.run_hardware(args)
    return runner.run_simulate(args)


if __name__ == '__main__':
    sys.exit(main())
