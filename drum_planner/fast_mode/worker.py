"""Contained --fast worker: saved inputs only, no cameras or motor interfaces."""
import argparse
import json
import os
import sys
from pathlib import Path
from ..common import digest
from ..experiment.config import no_can


def main():
    no_can()
    def guard(event,args):
        if event=='open' and isinstance(args[0],(str,bytes)) and os.fsdecode(args[0]).startswith(
                ('/dev/video','/dev/bus/usb','/dev/ttyUSB','/dev/ttyACM')):
            raise PermissionError('--fast planning has no device access')
    sys.addaudithook(guard)
    p=argparse.ArgumentParser();p.add_argument('--request',required=True);a=p.parse_args()
    config=json.loads(Path(a.request).read_text());context=config['context']
    for name in ['state','scene']:
        if digest(context[name])!=context[name+'_sha256']:raise ValueError('Frozen '+name+' changed')
    from .planner import run
    result=run(context,config['output'],config['bindings'],kind=config['task_constraint'],
               compare=config['compare'],seconds=config['budget_s'])
    return 0 if result['status']=='planned' else 2


if __name__=='__main__':raise SystemExit(main())
