"""Explicit physical SIGINT recovery check; never run as an offline unit test."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import signal
import subprocess
import time
from .trajectory import ROOT


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hardware',action='store_true')
    args=p.parse_args()
    if not args.hardware:
        print('No motion. --hardware launches one notified playback and interrupts it after 1.25 s.');return 0
    log=ROOT/'playback_results'/('live_cancel_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'.txt')
    child=subprocess.Popen([str(ROOT/'playback.sh'),'--method','paced_precise'],cwd=ROOT,
                           stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
    sent=False;directory=None
    with log.open('x') as f:
        for line in child.stdout:
            f.write(line);f.flush();print(line,end='',flush=True)
            if line.startswith('Evidence directory: '):directory=Path(line.strip().split(': ',1)[1])
            if line.startswith('PLAYBACK ') and not sent:
                time.sleep(1.25)
                child.send_signal(signal.SIGINT);sent=True
                print('TEST: SIGINT sent to the owned player process.',flush=True)
    code=child.wait()
    if directory is None:raise RuntimeError('No physical session was created')
    summary=json.loads((directory/'summary.json').read_text())
    events=[json.loads(s) for s in (directory/'events.jsonl').read_text().splitlines()]
    commands=[json.loads(s) for s in (directory/'commands.jsonl').read_text().splitlines()]
    phases=[e['phase'] for e in events]
    passed=(sent and code==1 and summary['error'].startswith('Cancelled:')
            and summary['relaxed_verified'] and summary['original_gains_restored']
            and 'CONTROLLED RECOVERY' in phases
            and phases.index('RECENTER SETTLED')<phases.index('RELAX')
            and all(c['side']=='right' for c in commands)
            and all(c['phase'] in ('ENABLE AT LIVE POSE','RELAX') for c in commands if c['kind']==4))
    verdict=dict(physical=True,passed=passed,signal='SIGINT',after_playback_s=1.25,
                 session=str(directory.relative_to(ROOT)),child_exit_code=code,
                 relaxed_verified=summary['relaxed_verified'],original_gains_restored=summary['original_gains_restored'])
    (directory/'cancellation_test.json').write_text(json.dumps(verdict,indent=2)+'\n')
    print(json.dumps(verdict,indent=2),flush=True)
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
