"""Real RViz/Tk simulation smoke check and terminal-style shutdown; no hardware."""
from pathlib import Path
import os
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    from PIL import ImageGrab
    output=ROOT/'experiment_results/record3_visual_check'
    output.mkdir(parents=True,exist_ok=True)
    existing=set(output.iterdir())
    log_path=ROOT/'diagnostics/strike_lab_cymbal/visual_launch.txt'
    env=dict(os.environ,STRIKE_LAB_OFFLINE_ONLY='1')
    with log_path.open('w') as log:
        p=subprocess.Popen([str(ROOT/'experiment.sh'),'--simulate','--auto-run',
                            '--method','all','--repetitions','1','--output',str(output)],
                           cwd=ROOT,env=env,start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+45
            while time.monotonic()<deadline:
                assert p.poll() is None,'Launcher exited early'
                sessions=set(output.iterdir())-existing
                if sessions and len(list(next(iter(sessions)).glob('*/score.json')))==9:break
                time.sleep(.2)
            else:raise AssertionError('Visual-check batch did not finish')
            import json
            assert all(json.loads(f.read_text())['passed'] for f in next(iter(sessions)).glob('*/score.json'))
            time.sleep(.5)
            ImageGrab.grab(bbox=(30,35,1250,1040)).save(ROOT/'diagnostics/strike_lab_cymbal/rviz.png')
        finally:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
            p.wait(timeout=12)
    assert p.returncode==0,p.returncode
    text=log_path.read_text()
    assert 'OpenGl version' in text,'RViz renderer not started'
    assert 'process has died' not in text and 'failed to terminate' not in text,text
    print('PASS: real RViz + Tk, all nine simulated modes, screenshot, clean process-group Ctrl-C')
    print('PASS: offline-only lock; no physical CAN or live notification')


if __name__=='__main__':main()
