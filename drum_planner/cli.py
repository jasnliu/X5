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
    from .planning import candidate_count, DEFAULT_CANDIDATES
    parser=argparse.ArgumentParser(description='Gemini/MoveIt planner. Hardware only through explicitly armed baseline or live --execute.')
    sub=parser.add_subparsers(dest='command')
    sub.add_parser('build',help='Compile camera-only SDK bridge and standalone MoveIt/OMPL backend')
    p=sub.add_parser('camera',help='Full-FOV native depth preview/snapshot; no robot connection')
    p.add_argument('--seconds',type=float,default=0,help='Duration after first usable frame; default until Q/Esc')
    p.add_argument('--headless',action='store_true');p.add_argument('--output')
    p.add_argument('--view',choices=['3d','2d'],default='3d',help='Live 2D + interactive 3D (default), or legacy 2D-only window')
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p=sub.add_parser('plan',help='Plan against a saved Gemini scene and explicit saved arm states; NO execution')
    p.add_argument('--candidates',type=candidate_count,default=DEFAULT_CANDIDATES,
                   help='Independent MoveIt seeds ranked by broad route quality (1–32; default 6)')
    p.add_argument('--scene',required=True);p.add_argument('--state',required=True)
    p.add_argument('--output');p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p.add_argument('--occupied-only-preview',action='store_true',help='Legacy preview-only flag; detected-obstacle checking is now the default')
    p=sub.add_parser('demo',help='Synthetic scene + clearly labelled example pose; offline, no camera/robot')
    p.add_argument('--candidates',type=candidate_count,default=DEFAULT_CANDIDATES,
                   help='Independent MoveIt seeds ranked by broad route quality (1–32; default 6)')
    p.add_argument('--output')
    p=sub.add_parser('preview',help='Saved trajectory over a single frozen Gemini surface mesh; no motor access')
    p.add_argument('plan');p.add_argument('--png')
    p.add_argument('--no-camera',action='store_true',help='Offline saved-scene playback (PNG export is always offline)')
    p.add_argument('--seconds',type=float,default=0,help='Close after this many seconds of mesh playback with camera OFF')
    p.add_argument('--output',help='New directory for the initial depth frame, surface mesh and diagnostic reports')
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p.add_argument('--autoplay',action='store_true',help='Start the SIMULATED arm animation when depth is ready')
    p.add_argument('--loop',action='store_true',help='Loop the SIMULATED trajectory, never the physical arm')
    p=sub.add_parser('hardware-check',help='READ ONLY: query disabled motors and capture a measured-pose/depth alignment snapshot; NEVER moves')
    p.add_argument('--output',help='New output directory; original recordings/models are never overwritten')
    p.add_argument('--query-seconds',type=float,default=3.,help='Disabled-motor observation duration, 1–600 seconds (default 3)')
    p.add_argument('--headless',action='store_true',help='Save measurements/reports without opening a window')
    p.add_argument('--no-camera',action='store_true',help='Encoder-only diagnostic; requires --headless')
    p.add_argument('--seconds',type=float,default=0.,help='Close the frozen alignment window after N seconds; 0 waits for Q')
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p=sub.add_parser('hardware-view',help='OFFLINE: inspect a saved measured-pose/depth snapshot; opens neither CAN nor camera')
    p.add_argument('folder');p.add_argument('--seconds',type=float,default=0.)
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p=sub.add_parser('live',help='Measure current disabled arms + fresh depth, then plan automatically with MoveIt')
    p.add_argument('--candidates',type=candidate_count,default=DEFAULT_CANDIDATES,
                   help='Independent MoveIt seeds ranked by broad route quality (1–32; default 6)')
    p.add_argument('--output');p.add_argument('--headless',action='store_true')
    p.add_argument('--seconds',type=float,default=0.,help='Frozen preview duration only, not an encoder soak')
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p.add_argument('--execute',action='store_true',help='PHYSICAL RIGHT ARM: requires commissioning review and typed confirmation; no GUI')
    p.add_argument('--commissioning',help='Hash-bound record of actual physical tool/alignment/support checks; required with --execute')
    p=sub.add_parser('planned-view',help='OFFLINE: animate a plan against its EXACT captured planning scene')
    p.add_argument('plan');p.add_argument('--seconds',type=float,default=0.)
    p.add_argument('--autoplay',action='store_true')
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p=sub.add_parser('simulate',help='Verify the saved cycle/recovery policy without hardware')
    p.add_argument('plan')
    p=sub.add_parser('experiment',help='SOFTWARE ONLY: saved scene/pose, endpoint-only gesture candidates')
    from .experiment.config import METHODS, budget, DEFAULT_METHOD
    p.add_argument('--method',choices=[*METHODS,'all'],default=DEFAULT_METHOD)
    p.add_argument('--list-methods',action='store_true')
    p.add_argument('--state',help='Saved measurement; default the frozen study pose; NEVER queries CAN')
    p.add_argument('--scene',help='Saved five-frame Gemini scene; defaults to the frozen study scene, camera stays closed')
    p.add_argument('--settings',help='JSON gesture preferences and candidate-pool settings')
    p.add_argument('--budget',type=budget,default=120.,help='Shared candidate-pool search budget in seconds (5–600); final verification/setup is separate')
    p.add_argument('--headless',action='store_true');p.add_argument('--output')
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p.add_argument('--seconds',type=float,default=0.,help='Close the GUI after N seconds; 0 until closed')
    p.add_argument('--autoplay',action='store_true',help='Play the simulated result when ready')
    p=sub.add_parser('baseline',help='Baseline-only GUI: current pose, preview, speed, explicitly armed right-arm play / hold / checked reverse')
    from .baseline.paths import speed as hardware_speed
    p.add_argument('--speed',type=hardware_speed,default=1.,help='Movement speed scale 0.25–1.0; never exceeds validated limits')
    p.add_argument('--output',help='New session directory')
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION))
    p.add_argument('--simulate',metavar='PLAN',help='OFFLINE fake-motor UI on an existing plan; no camera/CAN or physical commands')
    p.add_argument('--resume',metavar='SESSION',help='Reconnect to an existing motor-owner session without re-enabling or releasing it')
    p.add_argument('--seconds',type=float,default=0.,help='Simulation-only preview auto-close timer; NEVER closes an active hold')
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
            result=plan(folder,args.state,args.scene,args.calibration,args.occupied_only_preview,args.candidates)
            print(result['status'],str(Path(folder)/'plan.json'))
            print('SOFTWARE ONLY — motion is not authorized or available.')
            print('DETECTED OBSTACLES: full arm and stick checked; unseen space is not an automatic obstacle.')
            if args.occupied_only_preview:
                print('OCCUPIED-ONLY PREVIEW: unknown space was NOT checked. This is not a verified safe route.')
            if result['status']!='planned':
                from .diagnostics import blockage_summary
                print(blockage_summary(result));return 1
        elif args.command=='demo':
            from .planning import request,finish_plan
            folder=Path(args.output or output_folder('demo')).resolve();folder.mkdir(parents=True,exist_ok=False)
            state=read_state(ROOT/'drum_planner/config/example_start.json')
            req,manifest=request(folder,state,dict(synthetic=True,obstacle_padding_m=.02,boxes=[]),candidates=args.candidates)
            result=finish_plan(folder,req,manifest,state,DEFAULT_CALIBRATION,dict(source='Synthetic empty-world software test, NOT the real kit'))
            print(result['status'],folder/'plan.json')
            return 0 if result['status']=='planned' else 1
        elif args.command=='preview':
            from .preview import preview
            saved=preview(args.plan,args.png,live=not args.no_camera,seconds=args.seconds,output=args.output,
                          cal_path=args.calibration,autoplay=args.autoplay,loop=args.loop)
            if saved:print(saved)
        elif args.command=='hardware-check':
            from .hardware_check import run
            print(run(args.output or output_folder('readonly-hardware'),query_seconds=args.query_seconds,
                      seconds=args.seconds,headless=args.headless,no_camera=args.no_camera,cal_path=args.calibration))
        elif args.command=='hardware-view':
            from .alignment_view import show
            print(json.dumps(show(args.folder,seconds=args.seconds,cal_path=args.calibration),indent=2))
        elif args.command=='live':
            if args.execute and (not args.commissioning or args.seconds):
                raise ValueError('--execute requires --commissioning PATH and does not accept a preview timer')
            if args.commissioning and not args.execute:raise ValueError('--commissioning is only used with --execute')
            if args.execute and (not sys.stdin.isatty() or not Path(args.commissioning).is_file()):
                raise ValueError('Execution requires an attended terminal and an existing commissioning review')
            from .current_plan import run
            path=run(args.output or output_folder('current-plan'),headless=args.headless or args.execute,
                     seconds=args.seconds,cal_path=args.calibration,candidates=args.candidates)
            if json.loads(path.read_text())['status']!='planned':return 1
            if args.execute:
                from .execution import execute
                return 0 if execute(path,args.commissioning,args.calibration) else 1
            return 0
        elif args.command=='planned-view':
            from .planned_view import show
            show(args.plan,seconds=args.seconds,cal_path=args.calibration,autoplay=args.autoplay)
        elif args.command=='simulate':
            from .workflow import simulate
            report=simulate(json.loads(Path(args.plan).read_text()));print(json.dumps(report,indent=2))
        elif args.command=='experiment':
            from .experiment.config import METHODS, DESCRIPTIONS, settings
            from .experiment.app import prepare, headless, show
            if args.list_methods:
                for key in METHODS:print(key+' — '+METHODS[key]+'\n  '+DESCRIPTIONS[key])
                return 0
            import math
            if not math.isfinite(args.seconds) or args.seconds<0:raise ValueError('Invalid GUI duration')
            if args.method=='all' and not args.headless:raise ValueError('--method all requires --headless; GUI selects one method at a time')
            if args.headless and (args.seconds or args.autoplay):raise ValueError('Playback options need the GUI')
            params=settings(json.loads(Path(args.settings).read_text()) if args.settings else None)
            folder=Path(args.output or output_folder('experiment')).resolve()
            prepare(folder,args.state,args.scene,args.calibration)
            if args.headless:
                results=headless(folder,list(METHODS) if args.method=='all' else [args.method],params,args.budget)
                print(json.dumps(results,indent=2))
                return 1 if any(r['status'] in ('error','experiment_error') for r in results.values()) else 0
            show(folder,params,args.budget,args.method,args.autoplay,args.seconds)
        elif args.command=='baseline':
            import math
            if args.simulate and args.resume:raise ValueError('Choose --simulate or --resume, not both')
            if not math.isfinite(args.seconds) or args.seconds<0 or (args.seconds and not args.simulate):
                raise ValueError('--seconds is only for an offline simulation preview')
            from .baseline.workspace import show
            if args.resume:
                if args.output:raise ValueError('--resume uses its existing session; do not supply --output')
                folder=Path(args.resume).resolve();info=json.loads((folder/'control.json').read_text())
                cal=args.calibration if info['simulated'] else json.loads(Path(info['prepared']).read_text())['calibration']
                show(folder,cal,rate=info['rate'],resume=info)
            else:
                folder=Path(args.output or output_folder('baseline')).resolve()
                if args.simulate:
                    from .experiment.config import no_can
                    no_can()
                folder.mkdir(parents=True,exist_ok=False)
                show(folder,args.calibration,rate=args.speed,simulate_plan=args.simulate,seconds=args.seconds)
        elif args.command=='test':
            from .planning import build_moveit
            build_moveit()
            import pytest
            return pytest.main([str(ROOT/'tests/test_drum_planner.py'),str(ROOT/'tests/test_planner_zone.py'),str(ROOT/'tests/test_planner_optimization.py'),str(ROOT/'tests/test_planner_style.py'),str(ROOT/'tests/test_planner_experiment.py'),str(ROOT/'tests/test_live_depth_view.py'),
                                str(ROOT/'tests/test_live_trajectory_preview.py'),str(ROOT/'tests/test_robot_visuals.py'),
                                str(ROOT/'tests/test_vtk_view.py'),
                                str(ROOT/'tests/test_snapshot_mesh.py'),str(ROOT/'tests/test_readonly_hardware.py'),
                                str(ROOT/'tests/test_current_plan.py'),str(ROOT/'tests/test_planner_execution.py'),
                                str(ROOT/'tests/test_baseline_hardware.py'),str(ROOT/'tests/test_baseline_ui.py'),
                                str(ROOT/'tests/test_baseline_workspace.py'),'-q'])
        else:parser.print_help()
        return 0
    except Exception as e:
        label=('Baseline stopped; no automatic relaxation. If a controller is already active, reconnect to it; startup does not take over its ownership. '
               if getattr(args,'command',None)=='baseline' else
               'Execution stopped; check execution report and physical support. ' if getattr(args,'execute',False)
               else 'Planner stopped (no motor commands): ')
        print(label+str(e),file=sys.stderr)
        return 1


if __name__=='__main__':raise SystemExit(main())
