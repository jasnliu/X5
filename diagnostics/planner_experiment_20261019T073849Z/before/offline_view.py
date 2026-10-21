"""Device-free VTK playback for preview --no-camera (including old demo plans)."""
import time
import numpy as np

from .live_playback import LivePlaybackView
from .live_view import DisplayPacket


class OfflinePlaybackView(LivePlaybackView):
    def __init__(self,plan,autoplay=False,loop=False):
        # No calibration/camera access needed: saved points are already in base coordinates.
        super().__init__({'T_base_depth':np.eye(4).tolist()},plan.path.parent,plan,autoplay,loop)
        self.worker.close();self.sensor_panel.hide()
        self.window.setWindowTitle('X5 saved-plan playback — OFFLINE / no devices')
        self.banner.setText('OFFLINE SAVED TRAJECTORY • NO CAMERA / CAN / MOTOR COMMANDS')
        self.plan_label.setText(plan.warning)
        self.freeze_button.setText('Saved scene — no acquisition');self.freeze_button.setEnabled(False)
        self.mode.setEnabled(False);self.color.setEnabled(False);self.color.setCurrentIndex(1)
        self.save_button.setText('Save view [S]');self.close_button.setText('Close [Q]')
        self.note.setText('OFFLINE SIMULATION — exact saved robot model and trajectory. '
                          'No acquisition or collision revalidation. CAD mount/depth alignment need physical verification.')
        scene=plan.req['scene']
        points=(np.empty((0,3)) if scene.get('synthetic') else
                np.fromfile(scene['points_file'],dtype='<f4').reshape(-1,3))
        bounds=np.array([points.min(axis=0),points.max(axis=0)]) if len(points) else None
        black=np.zeros((1,1,3),dtype=np.uint8)
        display_points=points[::max(1,len(points)//18000)]  # Match the previous offline display sample exactly.
        packet=DisplayPacket(0,time.monotonic(),display_points,np.empty((0,3)),len(points),0,black,black,bounds,0.)
        self.box_layers=[]
        from .vtk_view import MeshLayer
        for box in scene.get('boxes',[]):
            corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
            corners=corners*np.array(box['size'])/2+box['xyz']
            layer=MeshLayer(self.viewport,points=True);layer.actor.GetProperty().SetPointSize(7.)
            layer.set_geometry(corners);layer.set_colors(np.tile([1.,0.,0.,1.],(8,1)))
            self.box_layers.append(layer)
        self.accept(packet)
        self.viewport.colorbar.SetVisibility(bool(len(points)))
        self.image_title.setText('No camera connection')
        self.status.setText('OFFLINE — saved scene and plan only; no device connections')

    def depth_label(self):return 'SAVED SCENE — NO DEVICES'
    def layer_label(self):return 'saved points'
    def scene_detail(self):return 'Saved scene only. No fresh camera data or live collision check.'
    def toggle_freeze(self):pass

    def save_view(self):
        path=self.folder/'preview_view.png';self.viewport.save_window(self.window,path)
        self.save_status.setText('Saved offline view: '+str(path))

    def poll(self,camera=None):
        from .snapshot_preview import SnapshotPlaybackView
        return SnapshotPlaybackView.poll(self)

    def report(self):
        report=super().report()
        report.update(type='offline_saved_trajectory_preview',can_access=False,camera_access=False,camera_updates=False)
        return report


def show(plan,autoplay=False,loop=False):
    viewer=OfflinePlaybackView(plan,autoplay,loop)
    try:
        while True:
            key=viewer.poll()
            if key==ord('q'):break
            if key==ord('s'):viewer.save_view()
            time.sleep(.005)
    finally:viewer.close()
