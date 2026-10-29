"""Hardware calculation controls. Editing invalidates; it NEVER calculates/moves."""
from PyQt5 import QtWidgets as Qt
import numpy as np
from . import settings


class PlanningControls(Qt.QWidget):
    def __init__(self,owner):
        super().__init__();self.owner=owner
        layout=Qt.QVBoxLayout(self);layout.setContentsMargins(0,0,0,0)
        row=Qt.QHBoxLayout();row.addWidget(Qt.QLabel('Timing effort:'))
        self.quality=Qt.QComboBox();self.quality.setObjectName('hardware_timing_quality')
        for key,label in settings.QUALITIES.items():self.quality.addItem(label,key)
        row.addWidget(self.quality);layout.addLayout(row)
        self.button=Qt.QPushButton('Joint limits / profiles…');self.button.setObjectName('hardware_joint_limits')
        layout.addWidget(self.button);self.label=Qt.QLabel();self.label.setWordWrap(True);layout.addWidget(self.label)
        self.refresh()
        self.quality.currentIndexChanged.connect(lambda _:owner.action(self.select_quality))
        self.button.clicked.connect(lambda:owner.action(lambda:self.dialog().exec_()))

    def refresh(self):
        p=self.owner.motion_profile
        self.quality.blockSignals(True);self.quality.setCurrentIndex(self.quality.findData(p['timing_quality']));self.quality.blockSignals(False)
        original=np.array_equal(settings.matrix(p),settings.DEFAULT_LIMITS)
        text='Start / return: measured physical pose.\n'+('Original velocity / acceleration / jerk limits.' if original else 'CUSTOM planning limits — hardware capability not certified.')
        if self.owner.side=='right':text+='\nRight J2: 0° operating floor; measured-rest transitions checked separately.'
        self.label.setText(text)

    def select_quality(self):
        try:self.owner.apply_motion_settings(dict(self.owner.motion_profile,timing_quality=self.quality.currentData()))
        finally:self.refresh()

    def dialog(self):
        owner=self.owner;dialog=Qt.QDialog(self);dialog.setWindowTitle(owner.side.title()+' hardware planning limits')
        layout=Qt.QVBoxLayout(dialog)
        label=Qt.QLabel('Velocity, acceleration and jerk constrain the planned route, not motor firmware.\n'
            'Use limits established for your loaded arm. Original values are retained by default.\n'
            'Apply invalidates the plan; Calculate again, inspect, then explicitly Start.\n'
            'The existing drive speed cap and fault/recovery behavior are unchanged.')
        label.setWordWrap(True);layout.addWidget(label)
        name=Qt.QLineEdit(owner.motion_profile['name']);layout.addWidget(name)
        grid=Qt.QGridLayout();fields={}
        for c,text in enumerate(('Joint','Velocity rad/s','Acceleration rad/s²','Jerk rad/s³')):grid.addWidget(Qt.QLabel(text),0,c)
        for j in range(7):
            grid.addWidget(Qt.QLabel('J'+str(j+1)),j+1,0)
            for d,key in enumerate(settings.FIELDS):
                box=Qt.QLineEdit(str(owner.motion_profile[key][j]));box.setObjectName(f'{key}_j{j+1}')
                fields[d,j]=box;grid.addWidget(box,j+1,d+1)
        layout.addLayout(grid);quality=Qt.QComboBox()
        for k,text in settings.QUALITIES.items():quality.addItem(text,k)
        quality.setCurrentIndex(quality.findData(owner.motion_profile['timing_quality']));layout.addWidget(quality)
        error=Qt.QLabel();error.setWordWrap(True);layout.addWidget(error)
        row=Qt.QHBoxLayout();buttons={}
        for k,text in [('defaults','Original defaults'),('load','Load…'),('save','Save…'),('apply','Apply'),('close','Close')]:
            button=Qt.QPushButton(text);button.setAutoDefault(False);button.setObjectName('hardware_profile_'+k)
            row.addWidget(button);buttons[k]=button
        layout.addLayout(row)
        def values():
            return settings.settings(owner.side,dict(name=name.text(),timing_quality=quality.currentData(),
                **{key:[float(fields[d,j].text()) for j in range(7)] for d,key in enumerate(settings.FIELDS)}))
        def populate(p):
            name.setText(p['name']);quality.setCurrentIndex(quality.findData(p['timing_quality']))
            for d,key in enumerate(settings.FIELDS):
                for j in range(7):fields[d,j].setText(str(p[key][j]))
            error.clear()
        def apply():
            try:owner.apply_motion_settings(values())
            except Exception as exc:error.setText(str(exc));return
            dialog.accept()
        def load():
            path,_=Qt.QFileDialog.getOpenFileName(dialog,'Load planning profile',str(settings.PROFILE_ROOT/owner.side),'JSON (*.json)')
            if path:
                try:populate(settings.load(path,owner.side))
                except Exception as exc:error.setText(str(exc))
        def save():
            try:p=values()
            except Exception as exc:error.setText(str(exc));return
            folder=settings.PROFILE_ROOT/owner.side;folder.mkdir(parents=True,exist_ok=True)
            path,_=Qt.QFileDialog.getSaveFileName(dialog,'Save planning profile',str(folder/'custom.json'),'JSON (*.json)')
            if path:
                try:settings.save(path,p);error.setText('Saved; Apply to select this profile.')
                except Exception as exc:error.setText(str(exc))
        buttons['defaults'].clicked.connect(lambda:populate(settings.settings(owner.side)))
        buttons['apply'].clicked.connect(apply);buttons['load'].clicked.connect(load);buttons['save'].clicked.connect(save)
        buttons['close'].clicked.connect(dialog.reject)
        dialog.fields=fields;dialog.buttons=buttons;dialog.error_label=error;dialog.resize(670,440)
        return dialog
