"""Device-free preparation, replay, evidence analysis and route forecasts."""
import argparse
from pathlib import Path
import sys
import time
import uuid
import signal

from .schema import defaults,read,write,validate,identity


def parser():
    p=argparse.ArgumentParser(description='Joint operating-envelope study. Default/current commands are OFFLINE; no camera or motor access.')
    sub=p.add_subparsers(dest='command',required=True)
    for cmd in ('init','demo'):
        q=sub.add_parser(cmd);side=q.add_mutually_exclusive_group(required=True)
        side.add_argument('--right',dest='side',action='store_const',const='right')
        side.add_argument('--left',dest='side',action='store_const',const='left')
        q.add_argument('--output',required=True)
        if cmd=='demo':q.add_argument('--failure',choices=['tracking','stale','temperature','current','motor_fault','slow','command_gap'])
    q=sub.add_parser('prepare');q.add_argument('--config',required=True);q.add_argument('--plan',required=True)
    q.add_argument('--poses',nargs='+');q.add_argument('--joints',type=int,nargs='+',default=list(range(1,8)))
    q.add_argument('--output',required=True)
    q=sub.add_parser('simulate');q.add_argument('--campaign',required=True);q.add_argument('--output',required=True)
    q.add_argument('--failure',choices=['tracking','stale','temperature','current','motor_fault','slow','command_gap'])
    q.add_argument('--fail-level',type=float,default=.8)
    q=sub.add_parser('analyze');q.add_argument('--campaign',required=True);q.add_argument('--traces',required=True)
    q.add_argument('--output',required=True);q.add_argument('--allow-synthetic-profile',action='store_true')
    q=sub.add_parser('routes');q.add_argument('--plan',required=True);q.add_argument('--profile',required=True)
    q.add_argument('--quality',choices=['fast','balanced','thorough'],default='balanced');q.add_argument('--output',required=True)
    return p


def dispatch(args):
    from . import campaign,simulation,analysis,routes
    folder=Path(args.output)
    if folder.exists():raise ValueError('Output already exists; choose a new path to preserve prior evidence')
    if args.command=='init':write(folder,defaults(args.side));return dict(config=str(folder))
    if args.command=='prepare':return campaign.prepare(validate(read(args.config)),args.plan,args.poses,folder,args.joints)
    if args.command=='simulate':return simulation.simulate(campaign.load(args.campaign),folder,failure=args.failure,fail_level=args.fail_level)
    if args.command=='analyze':return analysis.analyze(campaign.load(args.campaign),args.traces,folder,allow_synthetic=args.allow_synthetic_profile)
    if args.command=='routes':return routes.estimate(args.plan,args.profile,folder,args.quality)
    if args.command=='demo':
        folder.mkdir(parents=True,exist_ok=False);c=simulation.demo_campaign(args.side);write(folder/'campaign.json',c)
        simulation.simulate(c,folder/'traces',failure=args.failure)
        return analysis.analyze(c,folder/'traces',folder/'analysis',allow_synthetic=True)
    raise ValueError('Unknown command')


def main(argv=None):
    args=parser().parse_args(argv)
    from drum_planner.experiment.config import no_can
    no_can()
    stop=[False]
    def interrupted(*_):stop[0]=True
    handlers={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    try:
        if args.command=='init':
            result=dispatch(args)
        else:
            # Keep all heavy work serial, out of the caller/UI, at the same
            # 3 GiB aggregate limit as existing experiments. No AF_CAN access.
            from drum_planner.experiment.runtime import Job,PYTHON
            from drum_planner.common import ROOT
            worker_folder=ROOT/'diagnostics'/('joint-limit-job-'+uuid.uuid4().hex[:12])
            command=[str(PYTHON),'-m','joint_limit_lab.worker',*(argv if argv is not None else sys.argv[1:])]
            job=Job(command,worker_folder,timeout=600)
            try:
                while job.poll() is None:
                    if stop[0]:raise KeyboardInterrupt
                    time.sleep(.05)
                if stop[0]:raise KeyboardInterrupt
                if job.process.returncode:raise RuntimeError('Offline worker failed; see '+str(job.log_path))
            finally:job.close()
            receipt=Path(args.output)/'completion.json'
            if (not receipt.is_file() or read(receipt).get('arguments_sha256')!=identity(vars(args))
                    or read(receipt).get('completed') is not True):
                raise RuntimeError('Worker stopped without a complete output receipt; partial evidence retained, not accepted as complete')
            print('Contained software-only job:',worker_folder)
            return 0
        if stop[0]:raise KeyboardInterrupt
        print(result);return 0
    except KeyboardInterrupt:
        print('Cancelled offline study; worker stopped. Partial outputs retained, not accepted as completed evidence.',file=sys.stderr)
        return 130
    except (ValueError,OSError,RuntimeError,KeyError) as exc:
        print('Joint-limit study:',exc,file=sys.stderr);return 1
    finally:
        for s,handler in handlers.items():signal.signal(s,handler)


if __name__=='__main__':raise SystemExit(main())
