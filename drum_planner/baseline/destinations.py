"""Compact hardware destination selectors; editing never starts calculation."""
from PyQt5 import QtWidgets
from ..experiment import destinations
from ..experiment.checkpoints import MAX_CHECKPOINTS


class Selector(QtWidgets.QWidget):
    def __init__(self,owner):
        super().__init__();self.owner=owner;self.boxes=[];self.rows=[];self.accepted=[]
        self.layout_=QtWidgets.QVBoxLayout(self);self.layout_.setContentsMargins(0,0,0,0);self.layout_.setSpacing(5)
        self.layout_.addWidget(QtWidgets.QLabel('Destinations / checkpoints · maximum 3 · 2 s hold each'))
        controls=QtWidgets.QGridLayout()
        self.add=QtWidgets.QPushButton('Add checkpoint');self.remove=QtWidgets.QPushButton('Remove last')
        self.refresh=QtWidgets.QPushButton('Refresh destinations')
        controls.addWidget(self.add,0,0);controls.addWidget(self.remove,0,1);controls.addWidget(self.refresh,1,0,1,2)
        self.layout_.addLayout(controls)
        self.add.clicked.connect(lambda:owner.action(self.add_row));self.remove.clicked.connect(lambda:owner.action(self.remove_row))
        self.refresh.clicked.connect(lambda:owner.action(self.refresh_files))
        self.append_row();available=destinations.files(owner.side)
        self.populate([str(available[0]) if available else None]);self.accepted=self.paths()

    def paths(self):return [box.currentData() for box in self.boxes]

    def append_row(self):
        row=QtWidgets.QWidget();layout=QtWidgets.QHBoxLayout(row);layout.setContentsMargins(0,0,0,0)
        layout.addWidget(QtWidgets.QLabel(str(len(self.boxes)+1)+':'))
        box=QtWidgets.QComboBox();box.setMinimumContentsLength(12)
        box.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        layout.addWidget(box,1);self.boxes.append(box);self.rows.append(row)
        self.layout_.insertWidget(len(self.boxes),row)
        box.currentIndexChanged.connect(lambda _:self.owner.action(self.edited))

    def populate(self,values):
        available=destinations.files(self.owner.side);previous=None
        for i,box in enumerate(self.boxes):
            value=values[i]
            if i and (previous is None or value==previous):value=None
            choices=[p for p in available if str(p)!=previous]
            box.blockSignals(True);box.clear();box.addItem('Select destination…' if choices else 'No other saved destinations',None)
            for path in choices:box.addItem(path.name,str(path))
            box.setCurrentIndex(max(0,box.findData(value)));box.blockSignals(False);previous=box.currentData()
        self.add.setEnabled(len(self.boxes)<MAX_CHECKPOINTS);self.remove.setEnabled(len(self.boxes)>1)

    def editable(self):
        if self.owner.active:
            self.owner.message('The current run keeps its calculated destinations. Change them after returning to rest. Hold and Reverse remain available.')
            return False
        return True

    def edited(self):
        if not self.editable():self.populate(self.accepted);return
        self.populate(self.paths());self.accepted=self.paths();self.owner.destinations_changed()

    def add_row(self):
        if not self.editable() or len(self.boxes)>=MAX_CHECKPOINTS:return
        self.append_row();self.edited()

    def remove_row(self):
        if not self.editable() or len(self.boxes)<=1:return
        self.boxes.pop();row=self.rows.pop();self.layout_.removeWidget(row);row.hide();row.deleteLater();self.edited()

    def refresh_files(self):
        if not self.editable():return
        old=self.paths();self.populate(old)
        if old!=self.paths():self.edited()

    def restore(self,bindings):
        # Startup/resume only, before any commands or calculations.
        while len(self.boxes)<len(bindings):self.append_row()
        self.populate([b['destination_source'] for b in bindings]);self.accepted=self.paths()
