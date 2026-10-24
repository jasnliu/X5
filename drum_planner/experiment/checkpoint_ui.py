"""Small optional checkpoint selector, used only by experiment windows."""
from . import destinations
from .checkpoints import MAX_CHECKPOINTS


class CheckpointControls:
    def setup_checkpoint_controls(self):
        Qt=self.QtWidgets
        self.checkpoint_container=Qt.QWidget()
        layout=Qt.QVBoxLayout(self.checkpoint_container);layout.setContentsMargins(0,0,0,0)
        self.checkpoint_layout=layout;self.checkpoint_boxes=[];self.checkpoint_rows=[]
        controls=Qt.QHBoxLayout()
        self.add_checkpoint_button=Qt.QPushButton('Add checkpoint')
        self.remove_checkpoint_button=Qt.QPushButton('Remove last checkpoint')
        self.refresh_destinations_button=Qt.QPushButton('Refresh destinations')
        for button in (self.add_checkpoint_button,self.remove_checkpoint_button,self.refresh_destinations_button):
            controls.addWidget(button)
        controls.addStretch();controls.addWidget(Qt.QLabel('Maximum 3 destinations • 2 s hold each'))
        layout.addLayout(controls)
        self.window.layout().insertWidget(2,self.checkpoint_container)
        self.append_checkpoint_row()
        self.destination_box=self.checkpoint_boxes[0]  # original single-target API
        self.add_checkpoint_button.clicked.connect(self.add_checkpoint)
        self.remove_checkpoint_button.clicked.connect(self.remove_checkpoint)
        self.refresh_destinations_button.clicked.connect(self.refresh_destinations)

    def append_checkpoint_row(self):
        Qt=self.QtWidgets;n=len(self.checkpoint_boxes)+1
        widget=Qt.QWidget();row=Qt.QHBoxLayout(widget);row.setContentsMargins(0,0,0,0)
        row.addWidget(Qt.QLabel(f'Checkpoint {n}'+(' (first destination):' if n==1 else ':')))
        box=Qt.QComboBox();box.setMinimumContentsLength(25)
        box.setSizeAdjustPolicy(Qt.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        box.setObjectName(f'checkpoint_{n}');row.addWidget(box,1)
        self.checkpoint_layout.insertWidget(n-1,widget)
        self.checkpoint_boxes.append(box);self.checkpoint_rows.append(widget)
        box.currentIndexChanged.connect(self.select_destination)

    def checkpoint_selection(self):
        selected=[]
        for box in self.checkpoint_boxes:
            value=box.currentData()
            # Adjacent duplicates are excluded; A → B → A remains useful.
            if selected and (selected[-1] is None or value==selected[-1]):value=None
            selected.append(value)
        return selected

    def refresh_checkpoint_choices(self,selected):
        available=destinations.files(self.side);previous=None
        for i,box in enumerate(self.checkpoint_boxes):
            value=selected[i]
            if i and (previous is None or value==previous):value=None
            choices=[p for p in available if str(p)!=previous]
            box.blockSignals(True);box.clear()
            box.addItem('Select a destination…' if choices else 'No other saved destinations available',None)
            for path in choices:box.addItem(path.name,str(path))
            index=box.findData(value) if value else 0
            box.setCurrentIndex(max(0,index));box.blockSignals(False)
            previous=box.currentData()
        self.add_checkpoint_button.setEnabled(len(self.checkpoint_boxes)<MAX_CHECKPOINTS)
        self.remove_checkpoint_button.setEnabled(len(self.checkpoint_boxes)>1)

    def refresh_destinations(self,*_):
        before=self.checkpoint_selection();self.refresh_checkpoint_choices(before)
        if self.checkpoint_selection()!=before:self.select_destination()

    def add_checkpoint(self,*_):
        if len(self.checkpoint_boxes)>=MAX_CHECKPOINTS:return
        self.append_checkpoint_row();self.select_destination()

    def remove_checkpoint(self,*_):
        if len(self.checkpoint_boxes)<=1:return
        self.checkpoint_boxes.pop();widget=self.checkpoint_rows.pop()
        self.checkpoint_layout.removeWidget(widget);widget.hide();widget.deleteLater()
        self.select_destination()
