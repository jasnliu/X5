"""Experiment-only simulated spawn and explicit per-joint timing controls."""
from pathlib import Path

import numpy as np

from ..common import atomic_json
from . import motion_settings as motion, scenario, destinations


class MotionControls:
    def refresh_checkpoint_choices(self,selected):
        # Explicit CLI JSON targets may live outside the destination directory.
        # Keep them selectable without modifying the shared hardware selector.
        extras=[str(p) for p in self.context.get('initial_checkpoint_sources',[]) if str(p)!='dt1']
        if not extras and self.side!='right':return super().refresh_checkpoint_choices(selected)
        available=list(dict.fromkeys([*[str(p) for p in destinations.files(self.side)],*extras]))
        if self.side=='right':available.append('dt1')
        previous=None
        for i,box in enumerate(self.checkpoint_boxes):
            value=selected[i]
            if i and (previous is None or value==previous):value=None
            box.blockSignals(True);box.clear();box.addItem('Select a destination…',None)
            for path in available:
                if path!=previous:box.addItem('dt1' if path=='dt1' else Path(path).name,path)
            box.setCurrentIndex(max(0,box.findData(value) if value else 0));box.blockSignals(False)
            previous=box.currentData()
        self.add_checkpoint_button.setEnabled(len(self.checkpoint_boxes)<3)
        self.remove_checkpoint_button.setEnabled(len(self.checkpoint_boxes)>1)

    def setup_motion_controls(self):
        Qt=self.QtWidgets
        self.motion_profile=motion.from_context(self.context)
        self.context['motion_settings']=self.motion_profile
        atomic_json(self.session/'context.json',self.context)
        self.motion_container=Qt.QWidget();layout=Qt.QVBoxLayout(self.motion_container)
        layout.setContentsMargins(0,0,0,0);row=Qt.QHBoxLayout()
        row.addWidget(Qt.QLabel('Simulated start:'))
        self.start_box=Qt.QComboBox();self.start_box.setObjectName('experiment_start')
        row.addWidget(self.start_box,1);self.refresh_start_choices()
        self.start_box.currentIndexChanged.connect(self.select_start)
        row.addWidget(Qt.QLabel('Timing effort:'))
        self.timing_box=Qt.QComboBox();self.timing_box.setObjectName('timing_quality')
        for key,label in motion.QUALITIES.items():self.timing_box.addItem(label,key)
        self.timing_box.setCurrentIndex(self.timing_box.findData(self.motion_profile['timing_quality']))
        self.timing_box.currentIndexChanged.connect(self.select_timing_quality);row.addWidget(self.timing_box)
        self.limits_button=Qt.QPushButton('Joint limits / profiles…');self.limits_button.setObjectName('joint_limits')
        self.limits_button.clicked.connect(self.edit_motion_limits);row.addWidget(self.limits_button)
        layout.addLayout(row);self.motion_status=Qt.QLabel();self.motion_status.setWordWrap(True)
        layout.addWidget(self.motion_status);self.window.layout().insertWidget(3,self.motion_container)
        self.refresh_motion_status()

    def refresh_start_choices(self):
        if not hasattr(self,'start_box'):return
        selected=self.context.get('experiment_start',{}).get('destination_source')
        self.start_box.blockSignals(True);self.start_box.clear()
        self.start_box.addItem('Default (captured start)',None)
        paths=destinations.files(self.side)
        if selected and Path(selected) not in paths:paths.append(Path(selected))
        for path in paths:self.start_box.addItem(path.stem,str(path))
        self.start_box.setCurrentIndex(max(0,self.start_box.findData(selected)))
        self.start_box.blockSignals(False)

    def select_start(self,*_):
        if self.closing:return
        target=self.start_box.currentData()
        try:
            self.context=scenario.set_start(self.session,target)
        except Exception as exc:
            self.refresh_start_choices()
            self.status.setText('Starting position not loaded: '+str(exc)+'. Previous selection retained.')
            return
        # select_destination cancels/reaps old work, installs the new static
        # start immediately, and queues only the newest complete target sequence.
        self.events.append(dict(event='simulated_start_changed',start=self.context['experiment_start']))
        self.select_destination();self.refresh_motion_status()

    def apply_motion_settings(self,profile):
        profile=motion.settings(self.side,profile)
        changed=motion.identity(profile)!=motion.identity(self.motion_profile)
        self.motion_profile=profile;self.context['motion_settings']=profile
        atomic_json(self.session/'context.json',self.context)
        atomic_json(self.session/'motion_settings.json',profile)
        self.timing_box.blockSignals(True)
        self.timing_box.setCurrentIndex(self.timing_box.findData(profile['timing_quality']))
        self.timing_box.blockSignals(False)
        if changed:
            self.events.append(dict(event='motion_settings_changed',settings=profile))
            self.select_destination()
        self.refresh_motion_status()

    def select_timing_quality(self,*_):
        self.apply_motion_settings(dict(self.motion_profile,timing_quality=self.timing_box.currentData()))

    def motion_result_summary(self):
        return self.plan.obj.get('experiment',{}).get('retiming',{}).get('constraint_summary') if self.plan.playable else None

    def refresh_motion_status(self):
        if not hasattr(self,'motion_status'):return
        profile=self.motion_profile;start=self.context.get('experiment_start',{}).get('label','Default')
        original=np.array_equal(motion.matrix(profile),motion.DEFAULT_LIMITS)
        text=f'Spawn: {start} • '+('Original limits' if original else 'CUSTOM SIMULATION LIMITS — not hardware-validated')
        text+=' • '+motion.QUALITIES[profile['timing_quality']]+' • Return ends at this simulated start.'
        if self.side=='right':text+=' Right J2: 0° operating floor; measured-rest transitions checked separately.'
        summary=self.motion_result_summary()
        if summary:
            active=summary['highest_utilization'][0]
            names={'velocity_rad_s':'velocity','acceleration_rad_s2':'acceleration','jerk_rad_s3':'jerk'}
            text+=f' Highest utilization: J{active["joint"]} {names[active["derivative"]]} {100*active["utilization"]:.1f}%.'
        self.motion_status.setText(text)

    def motion_dialog(self):
        Qt=self.QtWidgets;dialog=Qt.QDialog(self.window);dialog.setObjectName('motion_limits_dialog')
        dialog.setWindowTitle(f'{self.side.title()} experiment — joint limits and profiles')
        layout=Qt.QVBoxLayout(dialog)
        note=Qt.QLabel('SIMULATION ONLY. These are timing constraints, not motor protection settings.\n'
            'Position bounds, collisions, current/torque protections and hardware programs are unchanged.\n'
            'Apply recalculates the selected trajectory. Every value must be positive and finite.')
        note.setWordWrap(True);layout.addWidget(note)
        row=Qt.QHBoxLayout();row.addWidget(Qt.QLabel('Profile name:'))
        name=Qt.QLineEdit(self.motion_profile['name']);name.setObjectName('motion_profile_name');row.addWidget(name)
        layout.addLayout(row);grid=Qt.QGridLayout();fields={}
        for c,label in enumerate(['Joint','Velocity (rad/s)','Acceleration (rad/s²)','Jerk (rad/s³)']):
            grid.addWidget(Qt.QLabel(label),0,c)
        for j in range(7):
            grid.addWidget(Qt.QLabel(f'J{j+1}'),j+1,0)
            for d,key in enumerate(motion.FIELDS):
                edit=Qt.QLineEdit(repr(float(self.motion_profile[key][j])))
                edit.setObjectName(f'{key}_j{j+1}');fields[d,j]=edit;grid.addWidget(edit,j+1,d+1)
        layout.addLayout(grid)
        row=Qt.QHBoxLayout();row.addWidget(Qt.QLabel('Timing effort:'));quality=Qt.QComboBox()
        for key,label in motion.QUALITIES.items():quality.addItem(label,key)
        quality.setCurrentIndex(quality.findData(self.motion_profile['timing_quality']));row.addWidget(quality)
        layout.addLayout(row)
        error=Qt.QLabel();error.setWordWrap(True);error.setStyleSheet('color: #b00020');layout.addWidget(error)
        result=self.motion_result_summary()
        if result:
            layout.addWidget(Qt.QLabel('Achieved peaks for the currently displayed result (all legs, including return):'))
            table=Qt.QTableWidget(7,3);table.setObjectName('achieved_joint_peaks')
            table.setHorizontalHeaderLabels(['Velocity (rad/s)','Acceleration (rad/s²)','Jerk (rad/s³)'])
            table.setVerticalHeaderLabels([f'J{j+1}' for j in range(7)])
            table.setEditTriggers(Qt.QAbstractItemView.NoEditTriggers)
            for j in range(7):
                for d in range(3):
                    table.setItem(j,d,Qt.QTableWidgetItem(f'{result["peaks"][d][j]:.5g} ({100*result["utilization"][d][j]:.1f}%)'))
            table.horizontalHeader().setSectionResizeMode(Qt.QHeaderView.Stretch)
            table.setMaximumHeight(235);layout.addWidget(table)
        row=Qt.QHBoxLayout();buttons={}
        for key,label in [('defaults','Original defaults'),('load','Load profile…'),('save','Save profile…'),('apply','Apply'),('close','Close')]:
            button=Qt.QPushButton(label);button.setObjectName('motion_'+key);button.setAutoDefault(False)
            row.addWidget(button);buttons[key]=button
        buttons['apply'].setDefault(True)
        layout.addLayout(row)

        def values():
            return motion.settings(self.side,dict(name=name.text(),timing_quality=quality.currentData(),
                **{key:[float(fields[d,j].text()) for j in range(7)] for d,key in enumerate(motion.FIELDS)}))

        def populate(profile):
            name.setText(profile['name']);quality.setCurrentIndex(quality.findData(profile['timing_quality']))
            for d,key in enumerate(motion.FIELDS):
                for j in range(7):fields[d,j].setText(repr(float(profile[key][j])))
            error.clear()

        def load_profile():
            path,_=Qt.QFileDialog.getOpenFileName(dialog,'Load '+self.side+' simulation profile',str(motion.PROFILE_ROOT/self.side),'JSON (*.json)')
            if path:
                try:populate(motion.load(path,self.side))
                except Exception as exc:error.setText(str(exc))

        def save_profile():
            try:profile=values()
            except Exception as exc:error.setText('Not saved: '+str(exc));return
            folder=motion.PROFILE_ROOT/self.side;folder.mkdir(parents=True,exist_ok=True)
            path,_=Qt.QFileDialog.getSaveFileName(dialog,'Save simulation profile',str(folder/'custom.json'),'JSON (*.json)')
            if path:
                try:motion.save(path,profile);error.setText('Saved profile; Apply to use it for this calculation.')
                except Exception as exc:error.setText('Not saved: '+str(exc))

        def apply():
            try:self.apply_motion_settings(values())
            except Exception as exc:error.setText('Not applied: '+str(exc));return
            dialog.accept()

        buttons['defaults'].clicked.connect(lambda:populate(motion.settings(self.side)))
        buttons['load'].clicked.connect(load_profile);buttons['save'].clicked.connect(save_profile)
        buttons['apply'].clicked.connect(apply);buttons['close'].clicked.connect(dialog.reject)
        dialog.fields=fields;dialog.error_label=error;dialog.buttons=buttons
        dialog.resize(690,700 if result else 440)
        return dialog

    def edit_motion_limits(self):
        dialog=self.motion_dialog();dialog.exec_()
