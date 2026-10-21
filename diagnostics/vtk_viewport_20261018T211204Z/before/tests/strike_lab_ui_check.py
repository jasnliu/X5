"""Actual Tk + ROS + spawned simulator integration. No physical interfaces allowed."""
import os
os.environ['STRIKE_LAB_OFFLINE_ONLY']='1'
import socket
import sys

def forbid_can(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:
        raise AssertionError('Physical CAN is forbidden in the offline UI check')
sys.addaudithook(forbid_can)

from pathlib import Path
import tempfile
import time
from strike_lab.cli import parse
from strike_lab.app import App


def main():
    with tempfile.TemporaryDirectory(prefix='strike-lab-ui-') as tmp:
        args,config=parse(['--simulate','--method','all','--auto-run','--repetitions','1','--output',tmp,'--no-rviz'])
        app=App(args,config)
        assert 'SIMULATION' in app.root.title()
        assert app.prepare_button.cget('text').startswith('1. CLOSE')
        assert len(app.method.get())>0
        deadline=time.monotonic()+45
        captured=[False];failed=[];custom_started=[False];phases=[]
        original_handle=app.handle
        def handle(message):
            phases.append(message['phase']);original_handle(message)
        app.handle=handle
        def check():
            if app.fault:
                failed.append(app.status.get());app.close_window();return
            if len(app.table.get_children())==9 and not app.busy:
                rows=[app.table.item(i)['values'] for i in app.table.get_children()]
                assert all(r[-1]=='PASS' for r in rows),rows
                assert 'SIMULATED RECORD3 PLAYBACK' in phases,phases
                assert not custom_started[0]
                assert str(app.depth_entry.cget('state'))=='normal'
                app.degrees.set('12');app.preview_goal()
                assert 'motion limit' in app.status.get()
                app.handle(dict(phase='READY'))
                assert 'motion limit' in app.status.get()
                app.degrees.set('3');app.preview_goal()
                assert app.input_error is None
                assert app.display_goal.degrees==3.
                assert 'zone >2.7°' in app.header.get()
                app.run_selected(all_methods=True);custom_started[0]=True
                assert str(app.depth_entry.cget('state'))=='disabled'
            if len(app.table.get_children())==18 and not app.busy:
                assert app.display_goal.degrees==3.
                assert any(app.canvas.itemcget(i,'text')=='3°' for i in app.canvas.find_all() if app.canvas.type(i)=='text')
                if not captured[0]:
                    app.root.update_idletasks()
                    try:
                        from PIL import ImageGrab
                        x,y=app.root.winfo_rootx(),app.root.winfo_rooty()
                        ImageGrab.grab(bbox=(x,y,x+app.root.winfo_width(),y+app.root.winfo_height())).save('diagnostics/strike_lab_cymbal/ui.png')
                    except ImportError:pass
                    captured[0]=True
                app.close_window();return
            if time.monotonic()>deadline:
                failed.append('UI timed out: '+app.status.get());app.close_window();return
            app.root.after(100,check)
        app.root.after(100,check)
        app.run()
        assert not failed,failed
        assert captured[0]
        assert len(list(Path(tmp).glob('*/*/score.json')))==18
        import json
        targets=[json.loads(p.read_text())['target_deg'] for p in Path(tmp).glob('*/*/metadata.json')]
        assert targets.count(10.)==9 and targets.count(3.)==9,targets
        print('PASS: nine modes at 10 and 3 degrees executed through the real Tk controls and spawned synthetic backend')
        print('PASS: robot joint states published; screenshot captured; clean stop and all records flushed')
        print('PASS: physical CAN forbidden; no camera, microphone, ESP32, or notification used')


if __name__=='__main__':main()
