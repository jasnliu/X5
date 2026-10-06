"""CLI shared by the GUI launcher and standalone/headless runner."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import signal
import sys
import time
from .config import ROOT, Rules, Limits
from .methods import METHODS


def parser():
    p=argparse.ArgumentParser(description='Standalone ten-degree J7 strike lab (simulation by default).')
    modes=p.add_mutually_exclusive_group()
    modes.add_argument('--hardware',action='store_true',help='Explicitly enable physical transport; warns through ntfy before setup')
    modes.add_argument('--simulate',action='store_true',help='Synthetic dynamics; no CAN, camera, microphone, or ESP32 (default)')
    p.add_argument('--method',choices=['all',*METHODS],default='powered')
    p.add_argument('--headless',action='store_true',help='Run a batch without Tk/RViz')
    p.add_argument('--no-rviz',action='store_true',help='Control window only')
    p.add_argument('--no-motion-notification',action='store_true',help='Explicitly waive the motion alert/countdown for an authorized session')
    p.add_argument('--fast',action='store_true',help='Headless virtual-time simulation only')
    p.add_argument('--auto-run',action='store_true',help='Automatically prepare and execute after UI opens')
    p.add_argument('--exit-after-batch',action='store_true',help='Close GUI/RViz after the batch, relax, and flush results')
    p.add_argument('--repetitions',type=int,default=3)
    p.add_argument('--interval',type=float,default=.25,help='Rest after each manual trial, seconds')
    p.add_argument('--campaign',action='store_true',help='Finite screening/refinement/validation/endurance plan')
    p.add_argument('--stage',choices=['screen','refine','validate','all'],default='all')
    p.add_argument('--config',type=Path,help='JSON: rules, limits, plant, parameters, and campaign plan overrides')
    p.add_argument('--output',type=Path,default=ROOT/'experiment_results')
    p.add_argument('--list-methods',action='store_true')
    p.add_argument('--rescore',type=Path,help='Create a new analysis of a saved session without moving anything')
    return p


def parse(argv=None):
    p=parser();a=p.parse_args(argv)
    if a.fast and (a.hardware or not a.headless):p.error('--fast requires --headless simulation')
    if not 1<=a.repetitions<=10000 or not 0<=a.interval<=60:p.error('Invalid repetition count/rest interval')
    if a.rescore and a.hardware:p.error('--rescore cannot be combined with --hardware')
    config=json.loads(a.config.read_text()) if a.config else {}
    unknown=set(config)-{'rules','limits','plant','parameters','plan'}
    if unknown:p.error('Unknown configuration keys: '+', '.join(sorted(unknown)))
    Rules(**config.get('rules',{}));Limits(**config.get('limits',{}))
    from .config import Plant
    from .campaign import Plan
    Plant(**config.get('plant',{}));Plan(**config.get('plan',{}))
    for mid,params in config.get('parameters',{}).items():
        if mid not in METHODS:p.error('Unknown method in parameters: '+mid)
        METHODS[mid].parameters(params)
    return a,config


def request_for(a,config):
    mids=list(METHODS) if a.method=='all' else [a.method]
    if a.campaign:
        return dict(action='campaign',methods=mids,stage=a.stage,plan=config.get('plan',{}),parameters=config.get('parameters',{}))
    return dict(action='run',methods=mids,repetitions=a.repetitions,interval=a.interval,
                parameters=config.get('parameters',{}))


def options_for(a,config):
    return dict(hardware=a.hardware,realtime=not a.fast,output=str(a.output),
                motion_notification=not a.no_motion_notification,
                rules=config.get('rules',{}),limits=config.get('limits',{}),plant=config.get('plant',{}),
                exit_after_batch=a.headless or a.exit_after_batch)


def main(argv=None):
    a,config=parse(argv)
    if a.list_methods:
        for mid,m in METHODS.items():print(f'{mid:18} {m.label}: {m.description}')
        return 0
    if a.rescore:
        from .storage import rescore_session
        # Reuse the original common rules/limits unless explicitly overridden.
        old=json.loads((a.rescore/'session.json').read_text())
        rules=Rules(**(old['rules']|config.get('rules',{})))
        limits=Limits(**(old['limits']|config.get('limits',{})))
        print(rescore_session(a.rescore,rules,limits));return 0
    if not a.headless:
        from .app import App
        app=App(a,config)
        app.run();return 1 if app.fault else 0
    from .runtime import Session
    session=Session(options_for(a,config));fault=False;started=False;complete=False
    signal.signal(signal.SIGINT,lambda *_:session.stop())
    signal.signal(signal.SIGTERM,lambda *_:session.stop())
    session.request('prepare')
    try:
        while session.process.is_alive():
            for msg in session.poll():
                phase=msg['phase']
                if phase=='READY' and not started:
                    session.request(**request_for(a,config));started=True
                if phase in ('TRIAL COMPLETE','BATCH COMPLETE','FAULT','CLOSED','UNPREPARED'):
                    print(json.dumps(msg),flush=True)
                if phase=='FAULT':fault=True
                if phase=='BATCH COMPLETE':complete=True
            time.sleep(.02)
        for msg in session.poll():
            print(json.dumps(msg),flush=True)
            fault|=msg['phase']=='FAULT';complete|=msg['phase']=='BATCH COMPLETE'
    finally:session.close()
    return 0 if complete and not fault else 1


if __name__=='__main__':sys.exit(main())
