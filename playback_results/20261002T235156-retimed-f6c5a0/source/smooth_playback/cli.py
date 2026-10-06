"""playback.sh entry point. Default invocation is the requested physical cycle."""
import argparse
import json
import os
from .trajectory import build, METHODS


def main():
    p=argparse.ArgumentParser(description='Center → record3 playback → verified center → relax. Sends ntfy before motion.')
    p.add_argument('--method',choices=METHODS,default='smooth')
    p.add_argument('--check',action='store_true',help='Validate trajectories offline; never open CAN or move')
    p.add_argument('--gripper',choices=('closed','untouched'),default='closed')
    p.add_argument('--no-video',action='store_true',help='Omit camera capture; motor evidence is still saved')
    args=p.parse_args()
    print('Validating record3, smooth trajectory, joint limits and center-return paths…',flush=True)
    g,trajectories=build()
    if args.check:
        print(json.dumps({k:t.metadata for k,t in trajectories.items()},indent=2));return 0
    if os.environ.get('PLAYBACK_OFFLINE_ONLY')=='1':raise RuntimeError('Hardware forbidden by PLAYBACK_OFFLINE_ONLY')
    from .runner import Runner,new_session
    path=new_session(args.method)
    print('Evidence directory:',path,flush=True)
    result=Runner(g,trajectories[args.method],path,args.gripper=='closed',not args.no_video).run()
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result['success'] else 1

if __name__=='__main__':raise SystemExit(main())
