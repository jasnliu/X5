"""Verification guard: forbid live hardware and child processes during offline solve."""
import atexit
import json
import os
from pathlib import Path
import socket
import subprocess
import sys

attempts = []
blocked_cpu_probes = []
def denied(kind, detail):
    attempts.append({'kind': kind, 'detail': str(detail)})
    raise RuntimeError('Offline verification forbids ' + kind)
def audit(event, args):
    if event in ('socket.__new__', 'subprocess.Popen', 'os.system'):
        denied(event, args)
    if event == 'open' and args and isinstance(args[0], (str, bytes)):
        path = os.fsdecode(args[0])
        if path.startswith('/dev/') and path not in ('/dev/null', '/dev/urandom', '/dev/random'):
            denied('device file open', path)
sys.addaudithook(audit)
def no_popen(*args, **kwargs):
    if args and args[0] == 'lscpu':
        blocked_cpu_probes.append('numpy optional CPU feature probe: not executed')
        raise OSError('Optional CPU feature probe disabled in offline verification')
    denied('Popen', args)
subprocess.Popen = no_popen
@atexit.register
def report():
    Path(os.environ['CALIBRATION_GUARD_REPORT']).write_text(json.dumps(dict(
        hardware_or_child_process_attempts=attempts, blocked_optional_cpu_probes=blocked_cpu_probes, passed=not attempts), indent=2)+'\n')
