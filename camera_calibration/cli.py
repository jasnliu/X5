import argparse
import sys

from .core import ROOT, Board
from .io import SERIAL, build_bridge, load_session, copy_session_for_solve
from .jpeg import build_decoder


def main(argv=None):
    parser = argparse.ArgumentParser(description='ChArUco camera-to-robot calibration only; no motor control.')
    commands = parser.add_subparsers(dest='command')
    for name, help_text in [('capture', 'GUI: automatic distinct-pose capture after Start; first 5 validation, then unlimited training'),
                            ('preview', 'GUI: camera/board only; never opens CAN')]:
        p = commands.add_parser(name, help=help_text)
        p.add_argument('--output', default=str(ROOT / 'calibration_sessions'), help='Parent directory for a NEW session')
        p.add_argument('--serial', default=SERIAL, help='Exact Gemini 2 serial; never auto-select another camera')
        p.add_argument('--fps', type=int, choices=(10, 15, 30), default=15, help='Color rate; default 15, always full-FOV 1280x720; YUYV supports 10/15')
        p.add_argument('--color-format', choices=('yuyv', 'mjpeg'), default='yuyv',
                       help='Native color format; default uncompressed YUYV avoids damaged MJPEG. No automatic fallback.')
    commands.add_parser('build', help='Compile SDK bridge and strict JPEG decoder; no devices opened')
    p = commands.add_parser('solve', help='Reopen saved captures offline; no devices opened')
    p.add_argument('session', help='Session directory containing session.json')
    p.add_argument('--output', help='NEW reprocessed session directory; leaves original session/results unchanged')
    commands.add_parser('test', help='Run synthetic/unit tests only; no devices opened')
    args = parser.parse_args(argv if argv is not None else (sys.argv[1:] or ['capture']))
    try:
        if args.command in ('capture', 'preview'):
            from .app import App
            App(args).run()
        elif args.command == 'build':
            print(build_bridge())
            print(build_decoder())
        elif args.command == 'solve':
            session = (copy_session_for_solve(args.session, args.output) if args.output
                       else load_session(args.session))
            board = Board(session.path / 'target.json')
            if not session.data['fit']:
                session.fit(board)
            else:
                print('Using frozen training fit. Validation poses are never fitted.')
            n = sum(s['phase'] == 'validation' for s in session.data['samples'])
            if n >= 5:
                out = session.validate(board)
                print('PASS' if out['validated'] else 'FAIL', session.path / 'calibration.json')
                m = out['validation']['metrics']
                print(f"Held-out RMS: {m['translation_mm']['rms']:.3f} mm / "
                      f"{m['rotation_deg']['rms']:.3f} deg / {m['reprojection_px']['rms']:.3f} px")
                if out['requires_kinematic_correction']:
                    print('Includes calibration-local kinematic correction; motor settings are unchanged.')
                print('Consistency check only. Physical depth mapping and collision safety remain unverified.')
                return 0 if out['validated'] else 1
            print(f'Training saved; only {n}/5 held-out poses. No validated calibration exported.')
            return 1
        elif args.command == 'test':
            import pytest
            return pytest.main([str(ROOT / 'tests/test_camera_calibration.py'),
                                str(ROOT / 'tests/test_camera_calibration_kinematics.py'), '-q'])
        else:
            parser.print_help()
        return 0
    except Exception as e:
        print(f'Calibration error: {e}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
