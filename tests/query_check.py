"""Explicit live query-only diagnostic; not included in unit-test discovery."""
import json
import socket
import time
from safe_zone.encoder import Observer, FRAME, EFF, PAYLOAD

def main():
    taps = {}
    frames = []
    observer = None
    result = {'mode':'query-only', 'samples':0, 'error':None}
    try:
        for bus in ('can0','can1'):
            s = socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
            s.bind((bus,)); s.setblocking(False); taps[bus]=s
        observer = Observer()
        for _ in range(60):
            q = observer.sample()
            result['samples'] += 1
            result['last_joint_positions'] = q
            for bus,s in taps.items():
                while True:
                    try:
                        cid,dlc,data = FRAME.unpack(s.recv(16))
                        frames.append({'bus':bus,'id':hex(cid),'dlc':dlc,'data':data.hex()})
                    except BlockingIOError: break
            time.sleep(.05)
    except Exception as e: result['error'] = str(e)
    finally:
        if observer:
            result['state_requests_sent'] = observer.tx_count
            observer.close()
        for s in taps.values(): s.close()
    commands = [f for f in frames if ((int(f['id'],16)>>8)&255)==0xfd]
    result['captured_requests'] = len(commands)
    result['request_audit_passed'] = all(int(f['id'],16) in [EFF|0x0200fd00|i for i in range(1,9)] and f['dlc']==8 and f['data']==PAYLOAD.hex() for f in commands)
    with open('diagnostics/query_check.json','w') as f: json.dump(result,f,indent=2)
    with open('diagnostics/query_frames.json','w') as f: json.dump(frames,f,indent=2)
    print(json.dumps(result,indent=2))
    if result['error'] or not result['request_audit_passed'] or result['captured_requests'] != result.get('state_requests_sent'): raise SystemExit(1)
if __name__=='__main__': main()
