"""Two device-free 3-D windows: frozen generated preview and measured feedback."""
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
from drum_planner.common import arm_joints
from drum_planner.experiment.runtime import PYTHON

_children={}
def close_previews():
    for child in _children.values():
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)
    _children.clear()

def open_preview(mission,session):
    key=str(mission)
    if key in _children and _children[key].poll() is None:return
    close_previews()
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    with (Path(session)/'preview.log').open('ab') as log:
        _children[key]=subprocess.Popen([str(PYTHON),'-m','camera_playback.generated_preview',str(mission),
                                       str(Path(session)/'display.json')],stdout=log,stderr=log,env=env,start_new_session=True)


class CombinedModel:
    def __init__(self,obj,curves):
        from safe_zone.geometry import Model
        self.obj=json.loads(Path(obj['plans']['right']).read_text())
        self.side='right';self.stick_link='right_drumstick';self.joints=arm_joints('right')
        model=Path(obj['plans']['right']).parent/'model/robot.urdf'
        self.model=Model(model);self.root=ET.parse(model).getroot();self.curves=curves
        self.outward_duration=sum(c.duration for c in curves.values());self.original=obj['original']
        self.return_steps=obj.get('return_plan',{}).get('steps',[])
        self.goal_pause=2. if self.return_steps else 0.
        self.duration=self.outward_duration+self.goal_pause+sum(s['duration'] for s in self.return_steps)
    def values(self,t):
        from drum_planner.baseline.paths import recovery_at
        left=self.curves['left'];right=self.curves['right']
        q={'left':left.at(min(left.duration,max(0.,t))),
           'right':right.at(min(right.duration,max(0.,t-left.duration)))}
        remaining=t-self.outward_duration-self.goal_pause
        if remaining>=0:
            for step in self.return_steps:
                side=step['side'];elapsed=min(remaining,step['duration'])
                q[side]=recovery_at(self.curves[side],step['spec'],elapsed)[0]
                remaining-=elapsed
                if remaining<=0:break
        return {name:float(v) for s in q for name,v in zip(arm_joints(s),q[s])}
    def phase(self,t):
        if self.return_steps and t>=self.duration-.001:return 'Both arms back at captured rest'
        if t<self.curves['left'].duration:return 'LEFT outbound; right at rest'
        if t<self.outward_duration:return 'RIGHT outbound; left at goal'
        remaining=t-self.outward_duration-self.goal_pause
        if remaining<0 or not self.return_steps:return 'Both prime goals — preview pause only; real beat continues until operator Return'
        for step in self.return_steps:
            if remaining<step['duration']:return step['side'].upper()+' RETURN — prepared before Start'
            remaining-=step['duration']
        return 'Both arms back at captured rest'
    def positions(self,t):return np.array([self.values(t)[j] for j in self.joints])
    def frames(self,t):return self.model.transforms(self.values(t))


def windows(mission,display):
    from PyQt5 import QtCore,QtWidgets
    from drum_planner.baseline.windows import ArmWindow
    from drum_planner.experiment.rgbd import load_scene
    from .generated_planning import load
    obj,curves=load(mission);model=CombinedModel(obj,curves)
    preview=ArmWindow('Generated beat — outbound LEFT → RIGHT / return RIGHT → LEFT — PREVIEW ONLY',model)
    live=ArmWindow('Generated beat — measured arms, frozen camera scene',model)
    packet,mesh=load_scene(Path(obj['capture'])/'camera/scene.json',obj['calibration'])
    for w in (preview,live):w.set_scene(packet,mesh)
    slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);slider.setRange(0,round(model.duration*1000));preview.layout_.addWidget(slider)
    button=QtWidgets.QPushButton('Play / Pause preview');preview.layout_.addWidget(button)
    playing=[False];last=[time.monotonic()]
    button.clicked.connect(lambda:playing.__setitem__(0,not playing[0]))
    def draw():
        now=time.monotonic();dt=now-last[0];last[0]=now
        if playing[0]:
            value=slider.value()+round(dt*1000)
            if value>=slider.maximum():value=slider.maximum();playing[0]=False
            slider.setValue(value)
        t=slider.value()/1000;preview.set_pose(dict(joints=model.values(t)))
        preview.caption.setText(f'{t:.3f} / {model.duration:.3f}s — '+model.phase(t))
        try:
            data=json.loads(Path(display).read_text());live.set_pose(data)
            age=time.monotonic()-data['stamp']
            live.caption.setText(('STALE FEEDBACK — ' if age>.5 else '')+data['label'])
        except (OSError,ValueError):live.caption.setText('Waiting for measured feedback; this window never commands motors.')
        preview.draw();live.draw()
    timer=QtCore.QTimer();timer.timeout.connect(draw);timer.start(50)
    preview.show();live.show();draw()
    return preview,live,timer,button,slider


def main():
    import sys
    from drum_planner.experiment.config import no_can
    no_can()
    from PyQt5 import QtWidgets
    app=QtWidgets.QApplication([]);items=windows(sys.argv[1],sys.argv[2]);app.exec_()
    for w in items[:2]:w.shutdown()

if __name__=='__main__':main()
