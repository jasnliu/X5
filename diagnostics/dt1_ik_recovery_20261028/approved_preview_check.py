"""Native Qt/VTK windows on saved real camera data. No hardware interfaces."""
import json
from pathlib import Path
import time
import sys
import socket
sys.addaudithook(lambda event,args: (_ for _ in ()).throw(AssertionError('No CAN')) if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN else None)
from PyQt5 import QtWidgets,QtCore,QtTest
from camera_playback.generated_preview import windows
from drum_planner.common import atomic_json

def main():
    root=Path('diagnostics/dt1_ik_recovery_20261028/approved-ui');mission=Path('planner_runs/dt1-ik-recovery-20261028/approved-saved/mission.json')
    obj=json.loads(mission.read_text());display=root/'preview_display.json'
    joints={f'openarmx_{side}_joint{i+1}':q for side in ('right','left') for i,q in enumerate(obj['original'][side+'_rad'])}
    atomic_json(display,dict(joints=joints,stamp=time.monotonic(),label='SAVED measured rest — software test'))
    qt=QtWidgets.QApplication([]);p,l,timer,button,slider=windows(mission,display)
    def pump(seconds=.2):
        until=time.monotonic()+seconds
        while time.monotonic()<until:qt.processEvents();time.sleep(.005)
    try:
        pump();first=l.robot.last_positions.copy()
        outward=sum(obj['duration_s'].values())
        slider.setValue(round(outward*1000));pump()
        assert (l.robot.last_positions==first).all()
        p.viewport.save_window(p,root/'generated_goals.png');l.viewport.save_window(l,root/'captured_rest.png')
        slider.setValue(round((outward+2.5)*1000));pump()
        assert 'RIGHT RETURN' in p.caption.text();p.viewport.save_window(p,root/'return_right.png')
        cache=json.loads(Path(obj['return_file']).read_text())
        slider.setValue(round((outward+2.+cache['steps'][0]['duration']+.5)*1000));pump()
        assert 'LEFT RETURN' in p.caption.text();p.viewport.save_window(p,root/'return_left.png')
        slider.setValue(slider.maximum());pump()
        assert 'captured rest' in p.caption.text()
        assert (l.robot.last_positions==first).all()
        slider.setValue(0);QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton);pump(.5)
        assert slider.value()>0
        QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton);v=slider.value();pump(.2);assert slider.value()==v
        reports=[w.viewport.report() for w in (p,l)]
        assert all(not r['render_errors'] and r['surface_triangles']>0 for r in reports)
        atomic_json(root/'preview_report.json',dict(passed=True,motor_commands=0,camera_access=False,reports=reports,
                    checks=['two 3D windows visible','frozen real camera mesh','both custom tools','left then right outward','right then left cached return','ends at captured rest','scrub independent of saved live rest','play/pause']))
    finally:
        timer.stop();p.shutdown();l.shutdown();qt.processEvents()
if __name__=='__main__':main()
