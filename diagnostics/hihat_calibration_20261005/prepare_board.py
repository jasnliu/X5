"""Authorized hi-hat-only open/rest preparation before the firmware reset.

No CAN access, ride-arm commands, kicks or closure commands are permitted.
"""
from pathlib import Path
import importlib.util,json,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from camera_playback.hihat import HiHatController,DEFAULT_ESP_PORT
OUT=Path(__file__).resolve().parent
port=str(Path(DEFAULT_ESP_PORT).resolve())

def check_idle():
    result=subprocess.run(['fuser',port],capture_output=True,text=True)
    if result.returncode != 1:
        raise RuntimeError('Serial port is owned or cannot be checked; refusing update: '+result.stdout+result.stderr)

check_idle()
spec=importlib.util.spec_from_file_location('ntfy',Path.home()/'.local/bin/codex-ntfy.py')
ntfy=importlib.util.module_from_spec(spec);spec.loader.exec_module(ntfy)
body=('Hi-hat controller update in 10 seconds: return the hi-hat to its open/rest position, '
      'then install bounded 90-to-115-degree calibration firmware. Keep clear of the hi-hat. The robot arm will NOT move.')
ok=ntfy.post_ntfy(body,'X5 hi-hat: calibration firmware update','warning,robot')
(OUT/'notification.json').write_text(json.dumps(dict(delivered=bool(ok),at=time.monotonic(),message=body),indent=2))
if not ok:raise SystemExit('ntfy failed; no hardware access')
ntfy.play_sound('{"type":"agent-turn-complete"}')
time.sleep(10.)
check_idle()
commands=[];lines=[]
class Controller(HiHatController):
    def _write(self,command):
        if command not in (b'H',b'S',b'O'):raise RuntimeError('Motion other than OPEN forbidden')
        super()._write(command)
        commands.append(dict(at=time.monotonic(),command=command.decode()))
    def _handle_line(self,line):
        lines.append(dict(at=time.monotonic(),line=line))
        super()._handle_line(line)

c=Controller(DEFAULT_ESP_PORT)
try:
    if not c.connect():raise RuntimeError(c.detail)
    deadline=time.monotonic()+5.
    while not c.ready():
        c.tick()
        if c.state=='error' or time.monotonic()>deadline:raise RuntimeError(c.detail)
        time.sleep(.01)
    c._write(b'O')
    until=time.monotonic()+1.5
    while time.monotonic()<until:
        c.tick()
        if c.state=='error':raise RuntimeError(c.detail)
        time.sleep(.01)
    stopped_at=time.monotonic();c.release_all()
    until=time.monotonic()+2.
    while not any(r['at']>=stopped_at and r['line'].startswith('STOP -- all motors released') for r in lines):
        c.tick()
        if c.state=='error' or time.monotonic()>until:raise RuntimeError('Release ACK absent: '+c.detail)
        time.sleep(.01)
    result=dict(success=True,open_command=True,settle_seconds=1.5,release_ack=True,
        encoder_position_readback_available=False,arm_access=False,commands=commands,lines=lines)
    (OUT/'rest_preparation.json').write_text(json.dumps(result,indent=2))
    print('Hi-hat OPEN command, 1.5 s serviced return, release ACK verified. No arm commands.')
finally:
    c.close()
