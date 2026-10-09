"""User-authorized physical verification: exact snare.sh --hardware --10 only.

The CAN recorder is passive. Pre/post queries use the disabled-only Observer.
No altered controller or strike parameters are injected into the real program.
"""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from safe_zone.encoder import Observer

directory = Path(__file__).parent / sys.argv[1]
directory.mkdir(exist_ok=False)
command = ['./snare.sh', '--hardware', '--10']


def query_disabled(name):
    observer = Observer()
    try:
        state = observer.sample()
        result = dict(monotonic_s=time.monotonic(), wall_s=time.time(),
                      all_16_disabled=True, query_frames=observer.tx_count,
                      joints=dict(state))
        (directory/name).write_text(json.dumps(result, indent=2)+'\n')
    finally:
        observer.close()


query_disabled('preflight_disabled.json')
spec = importlib.util.spec_from_file_location('codex_ntfy', '/home/jason/.local/bin/codex-ntfy.py')
notifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(notifier)
delivered = notifier.post_ntfy(
    'Authorized physical snare verification starts in 10 seconds. '
    'LEFT arm: center, close gripper, recording, ONE 10-degree snare hit, '
    'reverse recording, center, relax. Keep clear of the robot and drumstick.',
    'Robot motion warning: snare 10 degrees', 'warning,robot')
(directory/'notification.json').write_text(json.dumps(dict(delivered=delivered, wall_s=time.time()))+'\n')
if not delivered:
    raise RuntimeError('Motion blocked: ntfy warning was not delivered')
print('Motion warning delivered; exact commanded strike = 10 degrees.', flush=True)
time.sleep(10)
query_disabled('prelaunch_disabled.json')

camera_log = (directory/'camera.log').open('w')
camera = subprocess.Popen([
    '/usr/bin/nice', '-n', '10', '/home/jason/Proyectos3/Y2/.venv/bin/python',
    str(Path(__file__).parent/'record_camera.py'), str(directory)],
    cwd=ROOT, stdout=camera_log, stderr=subprocess.STDOUT)
can_log = (directory/'can.log').open('w')
can_error = (directory/'can_capture_stderr.log').open('w')
capture = subprocess.Popen(['/usr/bin/candump', '-L', 'can0', 'can1'],
                           stdout=can_log, stderr=can_error)
try:
    deadline = time.monotonic() + 10
    while not (directory/'camera_ready').exists():
        if camera.poll() is not None or time.monotonic() > deadline:
            raise RuntimeError('Motion blocked: video evidence not ready')
        time.sleep(.05)
    if capture.poll() is not None:
        raise RuntimeError('Motion blocked: CAN capture not running')
    print('Launching:', ' '.join(command), flush=True)
    started = time.monotonic()
    process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1)
    lines = []
    with (directory/'program.log').open('w', buffering=1) as log:
        for line in process.stdout:
            stamped = f'{time.monotonic():.6f} {line}'
            log.write(stamped)
            print(line, end='', flush=True)
            lines.append(line)
    code = process.wait()
    summary = dict(command=command, commanded_depth_deg=10., exit_code=code,
                   started_monotonic=started, ended_monotonic=time.monotonic(),
                   faults_or_retries=[line.strip() for line in lines
                       if any(word in line.lower() for word in ('fault', 'retry', 'stopped', 'traceback', 'timed out'))],
                   csp_confirmed=any('J6 CSP restore confirmed' in line for line in lines),
                   relaxed_confirmed=any('relaxed (fresh disabled feedback confirmed)' in line for line in lines))
    # Query only after the real program has closed its locks and exited.
    if code == 0:
        query_disabled('postflight_disabled.json')
        summary['postflight_all_16_disabled'] = True
    summary['success'] = (code == 0 and not summary['faults_or_retries']
                          and summary['csp_confirmed'] and summary['relaxed_confirmed']
                          and summary.get('postflight_all_16_disabled', False))
    (directory/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2), flush=True)
finally:
    camera.terminate()
    capture.terminate()
    camera.wait(timeout=10)
    capture.wait(timeout=10)
    camera_log.close()
    can_log.close()
    can_error.close()
