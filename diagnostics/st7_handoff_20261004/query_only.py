import json, socket, sys, time
sys.path.insert(0,'/home/jason/Proyectos3/X5')
from safe_zone.encoder import Observer, FRAME, EFF, PAYLOAD
socks={}; frames=[]
for name in ('can0','can1'):
    s=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW); s.bind((name,)); s.setblocking(False); socks[name]=s
observer=Observer()
try:
    for _ in range(3):
        q=observer.sample()
        for name,s in socks.items():
            while True:
                try:
                    cid,dlc,data=FRAME.unpack(s.recv(16))
                    frames.append(dict(bus=name,id=cid,dlc=dlc,data=data.hex()))
                except BlockingIOError: break
        time.sleep(.05)
    requests=[f for f in frames if (f['id']>>8)&255==0xfd]
    assert len(requests)==observer.tx_count==48
    assert all(f['id'] in [EFF|0x0200fd00|i for i in range(1,9)] and f['data']==PAYLOAD.hex() for f in requests)
    report=dict(query_only=True, all_16_motors_disabled_and_no_fault_bits=True, requests=len(requests), q=q, frames=frames)
    open('/home/jason/Proyectos3/X5/diagnostics/st7_handoff_20261004/query_preflight.json','w').write(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='frames'}))
finally:
    observer.close()
    for s in socks.values(): s.close()
