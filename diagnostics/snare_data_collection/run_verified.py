"""Passive video/CAN witness around the real finite collector; no motor commands."""
from pathlib import Path
import subprocess,time,json,signal,sys
ROOT=Path(__file__).resolve().parents[2]
directory=ROOT/'diagnostics/snare_data_collection'/sys.argv[1]
directory.mkdir(exist_ok=True)
if (directory/'program.log').exists():raise RuntimeError('Refusing overwrite')
canlog=(directory/'can.log').open('w');camlog=(directory/'camera.log').open('w');log=(directory/'program.log').open('w',buffering=1)
can=subprocess.Popen(['candump','-L','can0','can1'],stdout=canlog,stderr=subprocess.STDOUT)
cam=subprocess.Popen(['nice','-n','10',str(ROOT.parent/'Y2/.venv/bin/python'),str(ROOT/'diagnostics/snare_data_collection/record_camera.py'),str(directory)],stdout=camlog,stderr=subprocess.STDOUT)
try:
    deadline=time.monotonic()+5
    while not (directory/'camera_ready').exists():
        if cam.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Camera not ready before collector')
        time.sleep(.1)
    command=['./collect_data.sh']+sys.argv[2:]
    robot=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
    for line in robot.stdout:
        log.write(f'{time.monotonic():.6f} {line}');print(line,end='',flush=True)
    code=robot.wait()
    (directory/'exit.json').write_text(json.dumps(dict(command=command,exit_code=code)))
finally:
    cam.terminate();can.terminate()
    cam.wait(timeout=5);can.wait(timeout=5)
    canlog.close();camlog.close();log.close()
sys.exit(code)
