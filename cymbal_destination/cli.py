"""Standalone RIGHT-arm prime destination, frozen scene, no robot movement."""
import argparse,datetime,json,sys,uuid
from pathlib import Path
from drum_planner.common import ROOT,DEFAULT_CALIBRATION
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.experiment.app import wait
from drum_planner.experiment.config import no_can


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--replay',type=Path,help='Explicit saved DT1 pair.npz; requires --state; no camera access')
    p.add_argument('--state',type=Path,help='Explicit saved BOTH-arm pose; skips encoder queries, visibly labeled unsynchronized')
    p.add_argument('--open',type=Path,help='Open a completed run; no capture, encoders or calculation')
    p.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION)
    p.add_argument('--output',type=Path,help='New run directory; existing directories are never overwritten')
    p.add_argument('--headless',action='store_true',help='Generate destination without opening preview')
    p.add_argument('--seconds',type=float,default=0,help='Close preview after N seconds (0 stays open)')
    a=p.parse_args(argv)
    if a.seconds<0 or not __import__('math').isfinite(a.seconds):p.error('--seconds must be finite and nonnegative')
    if a.open and (a.replay or a.state or a.output):p.error('--open cannot be mixed with acquisition/output arguments')
    if a.replay and not a.state:p.error('--replay requires --state; no silent saved-pose fallback')
    no_can()  # Only the explicitly isolated query-only acquisition child can query encoders.
    try:
        if a.open:folder=a.open.resolve()
        else:
            folder=(a.output or ROOT/'cymbal_destination_runs'/
                (datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:6])).resolve()
            folder.mkdir(parents=True,exist_ok=False)
            print('RIGHT ARM PRIME DESTINATION ONLY — no recording, strike or motor movement.',flush=True)
            print('One paired RGB/depth snapshot, then camera OFF. '+('Explicit saved pose.' if a.state else 'Query-only BOTH-arm encoders; motors must already be disabled.'),flush=True)
            cmd=[PYTHON,'-m','cymbal_destination.acquisition','--folder',folder/'acquisition','--calibration',a.calibration.resolve()]
            if a.state:cmd+=['--state',a.state.resolve()]
            if a.replay:cmd+=['--replay',a.replay.resolve()]
            wait(Job(cmd,folder/'acquisition_job',timeout=90,allow_can=a.state is None))
            wait(Job([PYTHON,'-m','cymbal_destination.worker','--folder',folder,'--calibration',a.calibration.resolve()],folder/'solve_job',timeout=180))
        from .output import verified
        result=verified(folder)[0]
        print('Result:',result['status'],'—',folder,flush=True)
        if result['status']=='ready':print('ONE destination:',folder/'destination.json',flush=True)
        else:print(result.get('error','No solution'),flush=True)
        if not a.headless:
            from .view import show
            show(folder,a.seconds)
        return 0 if result['status']=='ready' else 2
    except KeyboardInterrupt:
        print('Cancelled. No motor commands.',file=sys.stderr);return 130
    except Exception as exc:
        print('Destination preview stopped (no motor commands): '+str(exc),file=sys.stderr);return 1


if __name__=='__main__':raise SystemExit(main())
