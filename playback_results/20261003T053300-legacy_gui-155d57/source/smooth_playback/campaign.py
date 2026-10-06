"""Preserved, balanced nine-cycle physical validation campaign.

No movement unless --hardware is explicitly supplied. Every cycle independently
notifies/counts down and recenters/restores gains/relaxes. Stops on any failure.
"""
import argparse
from datetime import datetime,timezone
import json
import time
from .trajectory import ROOT,build
from .runner import Runner,new_session
from .analyze import analyze

ORDER=('baseline','paced_kp20','legacy_gui',
       'legacy_gui','baseline','paced_kp20',
       'paced_kp20','legacy_gui','baseline')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware',action='store_true')
    args=parser.parse_args()
    print('Balanced validation order:',ORDER,flush=True)
    if not args.hardware:
        print('Offline listing only. --hardware is required to execute.');return 0
    g,methods=build()
    log=ROOT/'playback_results'/('campaign_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'.jsonl')
    print('Campaign log:',log,flush=True)
    for number,method in enumerate(ORDER,1):
        path=new_session(method)
        print(f'RUN {number}/{len(ORDER)} {method} {path}',flush=True)
        result=Runner(g,methods[method],path).run()
        row=dict(index=number,method=method,path=str(path.relative_to(ROOT)),success=result['success'])
        if result['success']:row['analysis']=analyze(path)
        with log.open('a') as f:f.write(json.dumps(row)+'\n')
        if not result['success']:
            print('Campaign stopped; inspect last run before further motion.',flush=True);return 1
        a=row['analysis']
        print('RESULT',method,'position_ripple_deg',a['measured_position_ripple_3_15hz_vector_rms_deg'],
              'velocity_ripple_rad_s',a['firmware_velocity_ripple_vector_rms_rad_s'],flush=True)
        time.sleep(3)
    print('Campaign complete; all cycles recentered, original gains restored and relaxed.',flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
