"""Non-moving post-flash capability check; open-before-reset was done separately."""
from pathlib import Path
import json,subprocess,time
import serial
OUT=Path(__file__).resolve().parent
PORT='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
assert json.loads((OUT/'rest_preparation.json').read_text())['success']
r=subprocess.run(['fuser',str(Path(PORT).resolve())],capture_output=True,text=True)
assert r.returncode==1,(r.stdout,r.stderr)
lines=[]
with serial.Serial(PORT,115200,timeout=.1,exclusive=True) as s:
    s.dtr=False;s.rts=True;time.sleep(.15);s.reset_input_buffer();s.rts=False
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        line=s.readline().decode('utf8','replace').strip()
        if line:lines.append(line);print(line,flush=True)
        if line.startswith('Calibration v2:'):break
    else:raise RuntimeError('Calibration-v2 firmware banner missing')
    s.write(b'SQ');s.flush()
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        line=s.readline().decode('utf8','replace').strip()
        if line:lines.append(line);print(line,flush=True)
        if line.startswith('HIHAT '):
            values={k:int(v) for k,v in (x.split('=') for x in line.split()[1:])}
            assert values==dict(v=2,min=90,max=115,angle=100,counts=149,pos=0,target=0,released=1,opened=1,fault=0),values
            break
    else:raise RuntimeError('Calibration-v2 status missing')
    assert any(x.startswith('STOP -- all motors released') for x in lines)
    assert not any(x.startswith(('FAULT','LINK LOST','ERROR:')) for x in lines)
(OUT/'firmware_verification.json').write_text(json.dumps(dict(success=True,commands=['S','Q'],readback=lines,status=values,arm_access=False,closure_command_sent=False),indent=2))
