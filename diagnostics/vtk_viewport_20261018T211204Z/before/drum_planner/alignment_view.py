"""Offline display of encoder-snapshot FK against the matching frozen depth.
No CAN or camera is opened here. Visual agreement is NOT automatic calibration.
"""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from safe_zone.geometry import Model
from .common import JOINTS,LEFT_JOINTS,DEFAULT_CALIBRATION
from .depth import check_metadata,deproject
from .hardware_check import load_snapshot
from .live_view import LiveDepthView,DisplayPacket,color_depth,sampled
from .snapshot_preview import SnapshotPlaybackView
from .surface_mesh import SurfaceMesh,depth_metres


class MeasuredPose:
    def __init__(self,folder,state,manifest):
        self.path=folder/'measured_start.json';self.state=state
        self.root=ET.parse(folder/'model/robot.urdf').getroot();self.model=Model(folder/'model/robot.urdf')
        self.obj={'model':manifest};self.duration=0.;self.playable=False
        self.static=dict(zip(JOINTS+LEFT_JOINTS,state['right_rad']+state['left_rad']))
        self.static['openarmx_left_finger_joint1']=state['left_gripper_opening_m']
        self.warning='MEASURED ENCODER SNAPSHOT — geometry/alignment NOT physically verified; NOT motion-ready'
    def positions(self,t):return np.array(self.state['right_rad'])
    def frames(self,t):return self.model.transforms(self.static)
    def phase(self,t):return 'MEASURED SNAPSHOT'


def load_scene(folder,cal):
    scene=json.loads((folder/'camera/scene.json').read_text())
    if scene.get('schema')!='x5-depth-surface-preview-v1' or scene.get('frames_used')!=1 or scene.get('preview_only') is not True:
        raise ValueError('Expected a single-frame preview scene')
    check_metadata(scene['camera'],cal)
    if not np.allclose(scene['T_base_depth'],cal['T_base_depth'],atol=1e-10,rtol=0):
        raise ValueError('Snapshot depth transform mismatch')
    with np.load(folder/'camera/raw_depth.npz',allow_pickle=False) as data:
        raw=data['depth'];scale=float(data['scale_m'])
    if raw.shape!=(800,1280) or raw.dtype!=np.dtype('<u2'):raise ValueError('Invalid native depth snapshot')
    depth=depth_metres(raw,scale);points=deproject(depth,scene['camera'],cal['T_base_depth'])
    if len(points)<1024:raise ValueError('Insufficient depth points in saved snapshot')
    rgb=color_depth(depth);h=scene['frame']
    packet=DisplayPacket(h['sequence'],h['received_monotonic'],sampled(points),np.empty((0,3)),
                         len(points),0,rgb,rgb,np.array([points.min(axis=0),points.max(axis=0)]),0.)
    with np.load(folder/'camera/surface_mesh.npz',allow_pickle=False) as data:
        vertices=data['vertices'];faces=data['faces'];pixels=data['source_pixels'];z=data['camera_depth_m']
    if (vertices.ndim!=2 or vertices.shape[1]!=3 or not np.isfinite(vertices).all() or
            faces.ndim!=2 or faces.shape[1]!=3 or not len(faces) or faces.dtype.kind not in 'iu' or
            faces.min()<0 or faces.max()>=len(vertices) or pixels.shape!=(len(vertices),2) or
            z.shape!=(len(vertices),) or not np.isfinite(z).all()):
        raise ValueError('Invalid saved mesh arrays')
    for array in (vertices,faces,pixels,z,packet.raw_points,packet.raw_image,packet.bounds):array.setflags(write=False)
    return packet,SurfaceMesh(vertices,faces,pixels,z,scene['mesh'])


def projected_image(rgb,coverage):
    """Show FK predictions only; never detect features or claim a match."""
    out=rgb.copy();height,width=out.shape[:2];points=coverage['landmarks']
    def segment(a,b,color):
        uv=[points[name]['pixel_uv'] for name in (a,b)]
        if any(value is None for value in uv):return
        ends=[tuple(np.clip(np.rint(value),-1000000,1000000).astype(int)) for value in uv]
        ok,p,q=cv2.clipLine((0,0,width,height),*ends)
        if ok:cv2.line(out,p,q,color,3,cv2.LINE_AA)
    for side,color in [('right',(255,150,0)),('left',(190,190,190))]:
        for i in range(7):segment(f'{side}_joint{i}',f'{side}_joint{i+1}',color)
        segment(side+'_joint7',side+'_hand',color)
    segment('right_stick_mount','right_stick_tip',(255,30,60))
    cv2.putText(out,'Predicted model projection - NOT detected features',(15,30),
                cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2,cv2.LINE_AA)
    return out


class AlignmentView(SnapshotPlaybackView):
    def __init__(self,cal,folder,pose,coverage):
        self.overlay=True;self.projection=None;self.coverage=coverage
        super().__init__(cal,folder,pose)
        self.window.setWindowTitle('X5 measured-pose alignment — READ ONLY / CAN and camera closed')
        self.banner.setText('ENCODER-SNAPSHOT MODEL + FROZEN DEPTH  •  CAN CLOSED / CAMERA OFF  •  NO MOTOR COMMANDS')
        self.info.setText('Measured at '+pose.state['captured_utc']+'\n'
                          'Solid robot = captured joint angles. Tan = modeled stick.\n'
                          'Lines on depth image are model predictions, NOT detections.\n'
                          'Compare visible surfaces; hidden parts cannot be verified.\n'
                          'No ongoing camera/encoder acquisition. Restart to resample.')
        # A measured pose is not an executable or animated trajectory. Hide the
        # entire playback row except its useful Fit arm + kit view control.
        row=self.window.layout().itemAt(2).layout()
        for i in range(row.count()-1):
            widget=row.itemAt(i).widget()
            if widget:widget.hide()
        self.overlay_box=self.QtWidgets.QCheckBox('Show encoder-derived robot / projected skeleton')
        self.overlay_box.setChecked(True);self.overlay_box.toggled.connect(self.toggle_overlay)
        self.window.layout().insertWidget(4,self.overlay_box)
        self.plan_label.setText(pose.warning)
        self.note.setText('READ-ONLY PREFLIGHT — successful feedback is NOT clearance, support or alignment approval. '
                          'Custom tool mount and left-stick geometry remain provisional; no motion can be started here.')

    def toggle_play(self):pass
    def seek(self,t):pass
    def change_speed(self,index):pass

    def toggle_overlay(self,checked):
        self.overlay=checked
        self.robot.set_visible(checked)
        self.refresh()

    def set_scene(self,packet,mesh):
        self.projection=projected_image(packet.raw_image,self.coverage)
        super().set_scene(packet,mesh)
        self.status.setText('READ-ONLY SNAPSHOT — both CAN sockets CLOSED — camera RELEASED — not live joint feedback')
        self.save_status.setText('Measured state and frozen scene: '+str(self.folder))

    def refresh(self,*args):
        super().refresh(*args)
        if self.packet is None:return
        rgb=self.projection if self.overlay and self.projection is not None else self.packet.raw_image
        image=self.QtGui.QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],self.QtGui.QImage.Format_RGB888).copy()
        self.image.setPixmap(self.QtGui.QPixmap.fromImage(image).scaled(
            self.image.size(),self.QtCore.Qt.KeepAspectRatio,self.QtCore.Qt.SmoothTransformation))
        self.image_title.setText('FROZEN DEPTH + PREDICTED MODEL' if self.overlay else 'FROZEN DEPTH — model overlay hidden')

    def draw_canvas(self):
        if self.robot is not None:
            self.robot.draw(0)
            self.ax.set_title('ENCODER-SNAPSHOT MODEL — NOT LIVE / NOT A TRAJECTORY\n'+self.layer_label(),fontsize=11)
            self.detail.setText(self.scene_detail())
        LiveDepthView.draw_canvas(self)
        self.last_animation_draw=time.monotonic()

    def scene_detail(self):
        missing=[name for name in ('right_hand','right_stick_mount','right_stick_tip')
                 if not self.coverage['landmarks'][name]['inside_depth_image']]
        msg='Outside depth image: '+', '.join(missing)+'. ' if missing else 'Right hand/stick landmarks project inside the image; visibility is NOT proven. '
        return msg+'Inspect physical alignment manually; this view cannot validate occluded surfaces, table support or motion clearance.'

    def report(self):
        report=super().report()
        report.update(type='measured_pose_alignment',robot_overlay_source='FK of saved measured encoder angles, not a live pose',
                      loaded_saved_mesh=True,mesh_builds=0,mesh_loads=1,
                      max_age_at_display_s=None,historical_snapshot=True,
                      animation_available=False,can_access=False,camera_access=False,
                      physical_alignment_verified=False)
        report['state_file']=report.pop('plan_file');report['state_warning']=report.pop('plan_warning')
        return report


def show(folder,*,seconds=0,cal_path=DEFAULT_CALIBRATION):
    if not np.isfinite(seconds) or seconds<0:raise ValueError('View duration must be finite and nonnegative')
    folder,cal,state,model,manifest=load_snapshot(folder,cal_path)
    if not manifest['camera_present']:raise ValueError('This encoder-only snapshot has no camera scene to display')
    packet,mesh=load_scene(folder,cal)
    coverage=json.loads((folder/'coverage.json').read_text())
    viewer=AlignmentView(cal,folder,MeasuredPose(folder,state,model),coverage)
    start=None
    try:
        viewer.set_scene(packet,mesh);start=time.monotonic()
        while not seconds or time.monotonic()-start<seconds:
            key=viewer.poll()
            if key==ord('q'):break
            if key==ord('s'):viewer.save_view()
            time.sleep(.01)
        viewer.save_view()
        report=viewer.report();report['duration_after_scene_ready_s']=time.monotonic()-start
        return report
    finally:viewer.close()
