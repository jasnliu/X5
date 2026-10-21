import argparse
import datetime as dt
import json
from pathlib import Path
import sys
import uuid

from .common import ROOT, DEFAULT_CALIBRATION, atomic_json, read_state


def output_folder(label):
    return ROOT/'planner_runs'/(dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+label+'-'+uuid.uuid4().hex[:6])


def main(argv=None):
    parser=argparse.ArgumentParser(description='Software-only depth/MoveIt planner. No CAN, torque or execution backend.')
    sub=parser.add_subparsers(dest='command')
    sub.add_parser('build',help='Compile camera-only SDK bridge and standalone MoveIt/OMPL backend')
    p=sub.add_parser('camera',help='Full-FOV native depth preview/snapshot; no robot connection')
    p.add_argument('--seconds',type=float,default=0,help='Duration after first usable frame; default until Q/Esc')
    p.add_argument('--headless',action='store_true');p.add_argument('--output')
    p.add_argument('--view',choices=['3d','2d'],default='3d',help='Live 2D + interactive 3D (default), or legacy 2D-only window')
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p=sub.add_parser('plan',help='Plan against a saved Gemini scene and explicit saved arm states; NO execution')
    p.add_argument('--scene',required=True);p.add_argument('--state',required=True)
    p.add_argument('--output');p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p.add_argument('--occupied-only-preview',action='store_true',help='Visualization experiment ONLY; unknown space not verified, never executable')
    p=sub.add_parser('demo',help='Synthetic scene + clearly labelled example pose; offline, no camera/robot')
    p.add_argument('--output')
    p=sub.add_parser('preview',help='3D software playback of an existing plan; no ROS or hardware')
    p.add_argument('plan');p.add_argument('--png')
    p=sub.add_parser('simulate',help='Verify the saved cycle/recovery policy without hardware')
    p.add_argument('plan')
    sub.add_parser('test',help='Offline tests only, including actual MoveIt/FCL planning')
    args=parser.parse_args(argv if argv is not None else (sys.argv[1:] or ['camera']))
    try:
        if args.command=='build':
            from .depth import build_depth
            from .planning import build_moveit
            print(build_depth());print(build_moveit())
        elif args.command=='camera':
            from .depth import capture
            print(capture(args.output or output_folder('depth'),args.seconds,args.headless,args.calibration,args.view))
        elif args.command=='plan':
            from .planning import plan
            folder=args.output or output_folder('plan')
            result=plan(folder,args.state,args.scene,args.calibration,args.occupied_only_preview)
            print(result['status'],str(Path(folder)/'plan.json'))
            print('SOFTWARE ONLY — motion is not authorized or available.')
            if args.occupied_only_preview:
                print('OCCUPIED-ONLY PREVIEW: unknown space was NOT checked. This is not a verified safe route.')
            if result['status']!='planned':print(json.dumps(result.get('details',result.get('error')),indent=2));return 1
        elif args.command=='demo':
            from .planning import request,finish_plan
            folder=Path(args.output or output_folder('demo')).resolve();folder.mkdir(parents=True,exist_ok=False)
            state=read_state(ROOT/'drum_planner/config/example_start.json')
            req,manifest=request(folder,state,dict(synthetic=True,obstacle_padding_m=.02,boxes=[]))
            result=finish_plan(folder,req,manifest,state,DEFAULT_CALIBRATION,dict(source='Synthetic empty-world software test, NOT the real kit'))
            print(result['status'],folder/'plan.json')
            return 0 if result['status']=='planned' else 1
        elif args.command=='preview':
            from .preview import preview
            preview(args.plan,args.png)
        elif args.command=='simulate':
            from .workflow import simulate
            report=simulate(json.loads(Path(args.plan).read_text()));print(json.dumps(report,indent=2))
        elif args.command=='test':
            from .planning import build_moveit
            build_moveit()
            import pytest
            return pytest.main([str(ROOT/'tests/test_drum_planner.py'),str(ROOT/'tests/test_live_depth_view.py'),'-q'])
        else:parser.print_help()
        return 0
    except Exception as e:
        print('Planner stopped safely (no motor commands): '+str(e),file=sys.stderr)
        return 1


if __name__=='__main__':raise SystemExit(main())
