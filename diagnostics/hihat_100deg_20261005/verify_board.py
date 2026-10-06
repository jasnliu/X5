"""Read the newly flashed target banner; reset only, outputs stay released."""
from pathlib import Path
import json,subprocess,time
import serial
OUT=Path(__file__).resolve().parent
PORT='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
assert json.loads((OUT/'rest_preparation.json').read_text())['success']
p=subprocess.run(['fuser',str(Path(PORT).resolve())],capture_output=True,text=True)
if p.returncode!=1:raise RuntimeError('Serial port not idle: '+p.stdout+p.stderr)
lines=[]
with serial.Serial(PORT,115200,timeout=.1,exclusive=True) as s:
    # Standard ESP32 normal reset: IO0 high (DTR false), EN low then high.
    s.dtr=False;s.rts=True;time.sleep(.15);s.reset_input_buffer();s.rts=False
    until=time.monotonic()+5.
    expected='Motor2 targets: keys 100 deg=149 counts, beat 100 deg=149 counts'
    while time.monotonic()<until:
        line=s.readline().decode('utf8','replace').strip()
        if line:lines.append(line);print(line,flush=True)
        if expected in lines:break
    if expected not in lines:raise RuntimeError('New 100-degree firmware banner not observed')
    s.write(b'S');s.flush()
    until=time.monotonic()+2.
    while time.monotonic()<until:
        line=s.readline().decode('utf8','replace').strip()
        if line:lines.append(line);print(line,flush=True)
        if line.startswith('STOP -- all motors released'):break
    else:raise RuntimeError('Release acknowledgement missing')
    if any(line.startswith(('FAULT','LINK LOST')) for line in lines):raise RuntimeError('Firmware fault')
(OUT/'firmware_verification.json').write_text(json.dumps(dict(success=True,port=PORT,
    close_degrees=100,encoder_target_counts=149,readback=lines,commands_sent=['S'],
    normal_reset=True,closure_command_sent=False,physical_angle_measured=False),indent=2))
