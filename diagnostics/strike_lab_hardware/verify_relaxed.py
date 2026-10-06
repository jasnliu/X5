#!/usr/bin/env python3
"""Explicit live STATE-QUERY-ONLY check after controllers have closed.

No enable/disable/hold/motion commands. Observer rejects any enabled/faulted motor.
Run from the repository root, separately from all robot controllers.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from safe_zone.encoder import Observer, request_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    audit = []
    allowed = {request_frame(i) for i in range(1, 9)}

    class AuditedSocket:
        def __init__(self, raw, side):self.raw,self.side = raw,side
        def __getattr__(self, name):return getattr(self.raw,name)
        def send(self, data):
            if data not in allowed:raise AssertionError('Non-query frame blocked')
            sent = self.raw.send(data)
            audit.append([self.side,data.hex()])
            return sent

    observer = None
    result = dict(mode='state_queries_only', utc=datetime.now(timezone.utc).isoformat())
    try:
        observer = Observer()
        observer.sockets = {side:AuditedSocket(raw,side) for side,raw in observer.sockets.items()}
        for _ in range(30):
            positions = observer.sample()
            if len(positions)!=16:raise RuntimeError('Expected both complete arm state batches')
            time.sleep(.03)
        result.update(disabled_motors=16,requests=observer.tx_count,positions=positions,
                      position_units='joint radians; finger_joint1 linear meters', passed=True)
    except Exception as exc:
        result.update(passed=False,error=str(exc))
    finally:
        if observer:observer.close()
        result['audit']=audit
        with args.output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='audit'},indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':sys.exit(main())
