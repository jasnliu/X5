"""One static destination in its exact frozen scene. No animation/control API."""
import json,time,xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from PyQt5 import QtCore,QtGui,QtWidgets
from vtkmodules.vtkFiltersSources import vtkSphereSource,vtkLineSource
from vtkmodules.vtkRenderingCore import vtkActor,vtkPolyDataMapper
from drum_planner.common import JOINTS,LEFT_JOINTS,calibration,digest,atomic_json
from drum_planner.vtk_view import VtkViewport,RobotActors,MeshLayer
from safe_zone.geometry import Model
from . import inputs,output
from .config import MAX_INWARD_YAW_DEG,GOAL_JOINT_RESERVE_DEG


class Pose:
    def __init__(self,req,q):
        self.req=req;self.root=ET.parse(req['urdf']).getroot();self.model=Model(req['urdf'])
        self.stick_link='right_drumstick';self.duration=0.;self.q=np.asarray(q,float)
        self.static=dict(zip(LEFT_JOINTS,req['left']));self.static['openarmx_left_finger_joint1']=req['left_opening_m']
    def positions(self,_):return self.q
    def frames(self,_):return self.model.transforms(dict(self.static,**dict(zip(JOINTS,self.q))))


class Window(QtWidgets.QMainWindow):
    def __init__(self,folder):
        super().__init__();self.folder=Path(folder).resolve();self.closed=False
        self.result,self.req,state,world,self.target,green,normal=output.verified(self.folder);r=self.result
        self.ready=r['status']=='ready'
        self.setWindowTitle('Cymbal prime destination — RIGHT ARM — SIMULATION ONLY')
        central=QtWidgets.QWidget();self.setCentralWidget(central);layout=QtWidgets.QVBoxLayout(central)
        acquisition=json.loads((self.folder/'acquisition/acquisition_report.json').read_text())
        origin=('FROZEN REPLAY' if acquisition['camera_replay'] else 'FRESH FROZEN CAMERA')
        pose_origin='FRESH ENCODERS' if acquisition['synchronized_pose'] else 'EXPLICIT SAVED POSE (not synchronized)'
        self.banner=QtWidgets.QLabel(origin+' · '+pose_origin+' · DESTINATION ONLY · NO MOTOR COMMANDS')
        self.banner.setStyleSheet('font-weight:bold;color:#164565');layout.addWidget(self.banner)
        self.status=QtWidgets.QLabel();self.status.setWordWrap(True);layout.addWidget(self.status)
        body=QtWidgets.QHBoxLayout();layout.addLayout(body,1)
        left=QtWidgets.QVBoxLayout();body.addLayout(left,0)
        left.addWidget(QtWidgets.QLabel('Frozen RGB target · red: surface · green: prime tip target'))
        self.rgb=QtWidgets.QLabel();self.rgb.setFixedSize(420,237)
        self.rgb.setPixmap(QtGui.QPixmap(str(self.folder/'acquisition/camera/rgb.png')).scaled(420,237,QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))
        left.addWidget(self.rgb)
        self.info=QtWidgets.QLabel();self.info.setWordWrap(True);self.info.setMaximumWidth(420);left.addWidget(self.info)
        self.angles=QtWidgets.QTableWidget(7,2);self.angles.setHorizontalHeaderLabels(['Joint','Destination (deg)'])
        self.angles.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers);self.angles.horizontalHeader().setStretchLastSection(True)
        self.angles.setMaximumWidth(420);left.addWidget(self.angles,1)
        self.viewport=VtkViewport();body.addWidget(self.viewport,1)
        with np.load(self.folder/'acquisition/camera/display.npz',allow_pickle=False) as data:
            vertices,faces,highlight=data['vertices'],data['faces'],data['highlight']
        self.viewport.surface.set_geometry(vertices,faces)
        colors=np.tile([.54,.68,.77,1.],(len(faces),1));colors[highlight]=[.12,.32,.86,1.]
        self.viewport.surface.set_colors(colors);self.viewport.surface.set_visible(True)
        self.viewport.cloud.set_visible(False);self.viewport.colorbar.SetVisibility(False)
        self.pose=Pose(self.req,r['goal_rad'] if self.ready else self.req['start']);self.robot=RobotActors(self.viewport,self.pose)
        self.robot.draw(0.)
        self.markers=[]
        self.marker(np.array(self.target['target']['world_m']),(1.,.05,.05))
        self.marker(green,(.05,.9,.1))
        disc=self.target['flat_disc'];n=np.asarray(disc['normal_world']);c=np.asarray(disc['center_world_m'])
        self.marker(c,(1.,.05,.05))  # Fitted-disc center: visualization only, not a planning target.
        axis=np.eye(3)[np.argmin(abs(n))];u=np.cross(n,axis);u/=np.linalg.norm(u);v=np.cross(n,u)
        a=np.arange(128)*2*np.pi/128
        verts=np.vstack((c,c+float(disc['radius_m'])*(np.cos(a)[:,None]*u+np.sin(a)[:,None]*v)))
        faces=np.c_[np.zeros(128,int),np.arange(128)+1,(np.arange(128)+1)%128+1]
        self.disc=MeshLayer(self.viewport);self.disc.set_geometry(verts,faces);self.disc.set_colors(np.tile([.05,.8,.3,1.],(128,1)))
        self.disc.actor.GetProperty().SetOpacity(.35)
        self.axis_line=vtkLineSource();mapper=vtkPolyDataMapper();mapper.SetInputConnection(self.axis_line.GetOutputPort())
        self.axis_actor=vtkActor();self.axis_actor.SetMapper(mapper);self.axis_actor.GetProperty().SetColor(.6,.1,.75)
        self.axis_actor.GetProperty().SetLineWidth(4);self.viewport.renderer.AddActor(self.axis_actor)
        controls=QtWidgets.QHBoxLayout();layout.addLayout(controls)
        self.pose_box=QtWidgets.QComboBox()
        if self.ready:self.pose_box.addItem('Destination (simulated)','goal')
        self.pose_box.addItem('Captured start (snapshot)','start');controls.addWidget(self.pose_box)
        self.fit_button=QtWidgets.QPushButton('Fit arm + cymbal');controls.addWidget(self.fit_button)
        self.camera_button=QtWidgets.QPushButton('Camera view');controls.addWidget(self.camera_button)
        self.disc_box=QtWidgets.QCheckBox('Green disc estimate');self.disc_box.setChecked(True);controls.addWidget(self.disc_box)
        self.axis_box=QtWidgets.QCheckBox('J7 axis (purple)');self.axis_box.setChecked(True);controls.addWidget(self.axis_box)
        self.export_button=QtWidgets.QPushButton('Export destination…');self.export_button.setEnabled(self.ready);controls.addWidget(self.export_button)
        self.save_button=QtWidgets.QPushButton('Save view');controls.addWidget(self.save_button)
        self.close_button=QtWidgets.QPushButton('Close');controls.addWidget(self.close_button)
        note=QtWidgets.QLabel('Static preview only: green dot = DT1’s fixed 3-inch disc-normal offset. '
            '3D red dots = surface target and disc center; their projected line defines inward aiming. '
            'Green disc is inferred; measured obstacles remain unchanged. Purple line is J7’s axis, NOT a motion command.\n'
            'No recording, timing law or strike is produced. Unseen space and absolute camera/model accuracy are not certified.')
        note.setWordWrap(True);layout.addWidget(note)
        self.pose_box.currentIndexChanged.connect(self.change_pose)
        self.fit_button.clicked.connect(self.fit);self.camera_button.clicked.connect(self.camera_view)
        self.disc_box.toggled.connect(lambda x:(self.disc.set_visible(x),self.viewport.render()))
        self.axis_box.toggled.connect(lambda x:(self.axis_actor.SetVisibility(x),self.viewport.render()))
        self.export_button.clicked.connect(self.export_dialog);self.save_button.clicked.connect(self.save_view)
        self.close_button.clicked.connect(self.close)
        self.resize(1370,850);self.change_pose();self.show();QtWidgets.QApplication.processEvents();self.fit()

    def marker(self,point,color):
        source=vtkSphereSource();source.SetCenter(*point);source.SetRadius(.005);source.SetThetaResolution(24);source.SetPhiResolution(16)
        mapper=vtkPolyDataMapper();mapper.SetInputConnection(source.GetOutputPort());actor=vtkActor();actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color);actor.GetProperty().LightingOff();self.viewport.renderer.AddActor(actor)
        self.markers.append((source,mapper,actor))

    def change_pose(self):
        from drum_planner.fast_mode.kinematics import Kinematics
        r=self.result;goal=self.pose_box.currentData()=='goal'
        self.pose.q=np.asarray(r['goal_rad'] if goal else self.req['start']);self.robot.draw(0.)
        f=Kinematics(self.req['urdf'],'right').fk(self.pose.q,jac=True)
        origin=self.pose.frames(0.)['openarmx_right_link7'][:3,3];axis=f['Jrotation'][:,6]
        self.axis_line.SetPoint1(*(origin-.09*axis));self.axis_line.SetPoint2(*(origin+.09*axis))
        self.axis_actor.SetVisibility(self.axis_box.isChecked())
        self.viewport.title_label.setText('SIMULATED PRIME DESTINATION — static pose' if goal else 'CAPTURED START — static snapshot, not live tracking')
        for i in range(7):
            self.angles.setItem(i,0,QtWidgets.QTableWidgetItem(f'J{i+1}'))
            self.angles.setItem(i,1,QtWidgets.QTableWidgetItem(f'{np.degrees(r["goal_rad"][i]):+.3f}' if self.ready else '—'))
        if self.ready:
            g=r['geometry'];p=np.array(self.target['above_target']['world_m'])
            self.status.setText(f'READY · tip error {g["tip_error_m"]*1000:.4f} mm (numerical) · '
                f'shaft-above elevation {max(0.,g["stick_tilt_deg"]):.3f}° · '
                f'inward aim {g["inward_yaw_deg"]:+.3f}° (±{MAX_INWARD_YAW_DEG:g}°) · '
                f'J7 swing-plane error {g["j7_plane_error_deg"]:.4f}°')
            self.info.setText('Fixed green target in robot coordinates (m):\n'+' / '.join(f'{x:+.5f}' for x in p)+
                '\n\nEndpoint: full robot + stick collision/zone checked.\nApproach feasibility: checked internally; route discarded.\n'
                'J2 goal ≥ +1°; measured-rest transition checked.\n'
                f'Other bounds: {GOAL_JOINT_RESERVE_DEG:g}° endpoint reserve.\n'
                'Other arm: fixed at captured pose.\n\nShaft lies above the disc-parallel plane through the tip. '
                'Parallel is preferred; no hard elevation cap. '
                f'Stick must point inward within ±{MAX_INWARD_YAW_DEG:g}° of the disc-plane center direction. '
                'Shorter/easier approaches preferred; gentle bias toward either zone edge. '
                'Negative J7 direction points toward the disc; no strike is planned.\n\nDestination saved only in this run folder:\n'+str(self.folder/'destination.json'))
        else:self.status.setText('NO DESTINATION: '+r.get('error','Unknown failure'));self.info.setText('Captured start only. No invalid or stale goal is displayed/exported.')
        self.viewport.render()

    def fit(self):
        points=[t[:3,3] for t in self.pose.frames(0.).values()]
        points.extend([self.target['target']['world_m'],self.target['above_target']['world_m']]);points=np.asarray(points)
        bounds=np.array([points.min(axis=0)-.15,points.max(axis=0)+.2]);bounds[1,0]=max(bounds[1,0],.9)
        self.viewport.fit(bounds,elev=24,azim=-55);self.viewport.render()

    def camera_view(self):
        T=np.asarray(calibration(self.result['calibration'])['T_base_depth'])
        self.viewport.camera.SetPosition(*T[:3,3]);self.viewport.camera.SetFocalPoint(*(T[:3,3]+T[:3,2]))
        self.viewport.camera.SetViewUp(*(-T[:3,1]));self.viewport.renderer.ResetCameraClippingRange();self.viewport.render()

    def save_view(self):
        self.viewport.save_window(self,self.folder/'destination_preview.png')

    def export_to(self,path):
        if not self.ready:raise ValueError('No validated destination to export')
        return output.export(self.folder/'destination.json',path,self.result['destination_sha256'])

    def export_dialog(self):
        path,_=QtWidgets.QFileDialog.getSaveFileName(self,'Export one destination (never a recording)','cymbal_prime.json','Destination (*.json)')
        if path:
            try:self.export_to(path);self.status.setText('Exported one destination: '+path)
            except Exception as exc:self.status.setText('Not exported: '+str(exc))

    def closeEvent(self,event):
        if not self.closed:
            self.closed=True
            atomic_json(self.folder/'view_report.json',dict(endpoint_only=True,hardware_access=False,motor_commands=0,
                camera_updates=0,renderer=self.viewport.report(),displayed=self.pose_box.currentData()))
            self.viewport.shutdown()
        event.accept()


def show(folder,seconds=0):
    qt=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    view=Window(folder)
    if seconds:QtCore.QTimer.singleShot(round(seconds*1000),view.close)
    qt.exec_()
