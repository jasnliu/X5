"""First native depth frame -> frozen mesh + saved trajectory; no motor access.

The SDK process is stopped BEFORE building/rendering the mesh. After that the
window has no camera polling, frame jobs, freshness timer or resume-capture path.
This single-frame preview is deliberately NOT a five-frame planning scene.
"""
from pathlib import Path
import time

import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from camera_calibration.core import digest
from .common import atomic_json,calibration,DEFAULT_CALIBRATION
from .depth import DepthCamera,deproject
from .live_playback import LivePlaybackView
from .live_view import DisplayPacket,color_depth,sampled
from .surface_mesh import build_surface,depth_metres


def prepare_scene(header,raw,meta,transform):
    started=time.monotonic()
    depth=depth_metres(raw,header['value_scale_m'])
    if np.count_nonzero(depth)<1024:raise ValueError('Too few valid depth returns for the initial scene')
    mesh=build_surface(raw,header['value_scale_m'],meta,transform)
    if not len(mesh.faces):raise ValueError('No continuous depth surfaces in the initial frame')
    points=deproject(depth,meta,transform);rgb=color_depth(depth)
    bounds=np.array([points.min(axis=0),points.max(axis=0)])
    packet=DisplayPacket(header['sequence'],header['received_monotonic'],sampled(points),
                         np.empty((0,3)),len(points),0,rgb,rgb,bounds,time.monotonic()-started)
    for array in (packet.raw_points,packet.voxels,rgb,bounds):array.setflags(write=False)
    return packet,mesh


def save_scene(folder,header,raw,meta,cal,cal_path,mesh):
    np.savez_compressed(folder/'raw_depth.npz',depth=raw,scale_m=header['value_scale_m'])
    np.savez_compressed(folder/'surface_mesh.npz',vertices=mesh.vertices,faces=mesh.faces,
                        source_pixels=mesh.source_pixels,camera_depth_m=mesh.camera_depth_m)
    obj=dict(schema='x5-depth-surface-preview-v1',preview_only=True,synthetic=False,
             camera=meta,frame=header,frames_used=1,calibration_sha256=digest(cal_path),
             T_base_depth=cal['T_base_depth'],mesh=mesh.stats,
             captured_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
             camera_released_before_meshing=True,collision_scene=False,
             unknown_space_policy='holes and unseen surfaces remain unknown; not a planning scene',
             files_sha256={name:digest(folder/name) for name in ('raw_depth.npz','surface_mesh.npz')},
             physical_depth_alignment_verified=False,motion_authorized=False,can_access=False,motor_commands=0)
    atomic_json(folder/'scene.json',obj)


class SnapshotPlaybackView(LivePlaybackView):
    def __init__(self,cal,folder,plan,autoplay=False,loop=False):
        self.mesh=None;self.surface=None;self.mesh_builds=0;self.visible_triangles=0
        super().__init__(cal,folder,plan,autoplay,loop)
        # This worker is never used in snapshot mode; do not leave a live pipeline.
        self.worker.close()
        self.window.setWindowTitle('X5 trajectory + frozen depth mesh — SIMULATION ONLY / no motors')
        self.banner.setText('SIMULATED TRAJECTORY + INITIAL DEPTH MESH  •  CAMERA STOPS AFTER ONE FRAME  •  NO MOTORS')
        self.info.setText('One native depth frame, saved automatically. No RGB stream.\n'
                          'The camera is released before the mesh is built.\n'
                          'Gaps and black pixels are unobserved, not detected obstacles.\n'
                          'Restart this preview to capture a different scene.')
        self.freeze_button.setText('Single snapshot — no refresh');self.freeze_button.setEnabled(False)
        self.mode.blockSignals(True);self.mode.clear();self.mode.addItems(['Surface mesh','Depth points','Mesh + points'])
        self.mode.blockSignals(False)
        self.save_button.setText('Save view [S]');self.close_button.setText('Close [Q]')
        self.image_title.setText('Waiting for the first native depth frame…')
        self.plan_label.setText(plan.warning+' • SNAPSHOT MESH DOES NOT REVALIDATE THIS PATH')
        self.note.setText('DISPLAY ONLY — visible-surface approximation, not collision geometry or a clearance verdict. '
                          'No hole filling or hidden surfaces. Saved-plan robot pose is simulated, not measured.')
        self.surface=Poly3DCollection([],edgecolors='none',linewidths=0,alpha=1.)
        self.ax.add_collection3d(self.surface)

    def depth_label(self):
        if self.packet is None:return 'waiting for initial snapshot'
        return f'INITIAL SNAPSHOT {self.packet.sequence:,} — CAMERA OFF'

    def layer_label(self):
        return ('surface mesh','depth points','mesh + points')[self.mode.currentIndex()]

    def scene_detail(self):
        if self.mesh is None:return 'Waiting for one native frame; no robot connection.'
        return (f'Frozen initial scene: {self.mesh.stats["triangles"]:,} triangles '
                f'({self.visible_triangles:,} in this view). No further camera acquisition or mesh rebuilding. '
                'Missing/hidden space remains UNKNOWN. Not a collision check.')

    def toggle_freeze(self):
        # Space cannot accidentally resume camera acquisition in this viewer.
        pass

    def set_scene(self,packet,mesh):
        if self.packet is not None:raise RuntimeError('The initial scene is immutable; restart to recapture')
        self.mesh=mesh;self.mesh_builds=1
        self.frame_count=1
        self.accept(packet)
        self.status.setText(f'CAMERA OFF / RELEASED — 1 captured frame — 1 mesh build — scene never updates')
        self.save_status.setText('Initial depth and mesh saved automatically: '+str(self.folder/'scene.json'))

    def accept(self,packet):
        if self.packet is not None:raise RuntimeError('Snapshot preview cannot accept another frame')
        return super().accept(packet)

    def refresh(self,*_):
        if self.packet is None or self.mesh is None:return
        p=self.packet;mesh=self.mesh
        if self.fit_needed and p.bounds is not None:
            bounds=np.array([np.minimum(p.bounds[0],self.focus_box[0]),np.maximum(p.bounds[1],self.focus_box[1])])
            mid=bounds.mean(axis=0);half=max(.1,float(np.max(np.ptp(bounds,axis=0)))*.53)
            for i,setter in enumerate((self.ax.set_xlim3d,self.ax.set_ylim3d,self.ax.set_zlim3d)):
                setter(mid[i]-half,mid[i]+half)
            self.fit_needed=False
        height=self.color.currentIndex()==1
        values=mesh.vertices[:,2] if height else mesh.camera_depth_m
        lo,hi=(float(values.min()),float(values.max())) if height else (.15,5.)
        self.cloud.set_clim(lo,max(lo+.01,hi));self.colorbar.set_label('World height (m)' if height else 'Camera depth (m)')
        points=self.display_points(p.raw_points)
        self.cloud._offsets3d=tuple(points.T)
        colors=points[:,2] if height else (points-self.transform[:3,3])@self.transform[:3,2]
        self.cloud.set_array(colors);self.cloud.set_visible(self.mode.currentIndex()!=0)
        # Matplotlib 3.6 otherwise draws triangles outside the chosen axes. This
        # clips ONLY the display and never changes the saved mesh/camera data.
        triangles=mesh.vertices[mesh.faces]
        limits=np.array([self.ax.get_xlim3d(),self.ax.get_ylim3d(),self.ax.get_zlim3d()])
        inside=np.all((triangles>=limits[:,0])&(triangles<=limits[:,1]),axis=(1,2))
        self.visible_triangles=int(np.count_nonzero(inside))
        self.surface.set_verts(triangles[inside])
        self.surface.set_facecolors(self.cloud.to_rgba(values[mesh.faces[inside]].mean(axis=1)))
        self.surface.set_visible(self.mode.currentIndex()!=1)
        rgb=p.raw_image
        image=self.QtGui.QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],self.QtGui.QImage.Format_RGB888).copy()
        self.image.setPixmap(self.QtGui.QPixmap.fromImage(image).scaled(
            self.image.size(),self.QtCore.Qt.KeepAspectRatio,self.QtCore.Qt.SmoothTransformation))
        self.image_title.setText('INITIAL DEPTH SNAPSHOT — CAMERA OFF')
        self.modes_rendered.add(self.layer_label());self.draw_canvas()

    def request_save(self):
        self.save_requested=True

    def save_view(self):
        path=self.folder/'preview_view.png'
        if not self.window.grab().save(str(path)):raise RuntimeError('Could not save preview image')
        self.save_status.setText('Saved current view (no camera access): '+str(path))

    def poll(self,camera=None):
        # Intentionally NOT LiveDepthView.poll: no camera object is needed.
        self.app.processEvents()
        if self.close_requested:return ord('q')
        if self.save_requested and self.packet is not None:
            self.save_requested=False;return ord('s')
        if self.clock.playing and time.monotonic()-self.last_animation_draw>=.05:self.draw_canvas()
        return -1

    def report(self):
        report=super().report()
        report.update(type='snapshot_mesh_trajectory_preview',camera_updates=False,mesh_builds=self.mesh_builds,
                      mesh=None if self.mesh is None else self.mesh.stats,maximum_pending_jobs=0,
                      scene_immutable=True,collision_mesh=False)
        return report


def run(folder,plan,*,seconds=0,cal_path=DEFAULT_CALIBRATION,autoplay=False,loop=False,
        viewer_factory=SnapshotPlaybackView):
    if not np.isfinite(seconds) or seconds<0:raise ValueError('Preview duration must be finite and nonnegative')
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False)
    cal=calibration(cal_path)
    viewer=None;camera=None;error=None;ready=None;released=None;header=None
    try:
        viewer=viewer_factory(cal,folder,plan,autoplay,loop)
        camera=DepthCamera(folder,cal,single_frame=True)
        started=time.monotonic()
        while camera.latest is None:
            if camera.error:raise RuntimeError(camera.error)
            if time.monotonic()-started>15:raise RuntimeError('Initial depth camera startup timeout')
            if viewer.poll()==ord('q'):raise RuntimeError('Preview closed before the initial depth snapshot')
            time.sleep(.005)
        header,raw=camera.latest
        if time.monotonic()-header['received_monotonic']>1.:raise RuntimeError('Initial depth frame is stale')
        camera.close()
        if camera.error:raise RuntimeError(camera.error)
        if camera.thread.is_alive() or (camera.process and camera.process.poll() is None):
            raise RuntimeError('Camera failed to stop after the initial frame')
        if camera.process and camera.process.poll()!=0:
            raise RuntimeError(f'Camera bridge did not shut down cleanly: exit {camera.process.poll()}')
        if camera.count!=1:raise RuntimeError('Snapshot camera acquired more than one frame')
        released=time.monotonic()
        packet,mesh=prepare_scene(header,raw,camera.meta,cal['T_base_depth'])
        save_scene(folder,header,raw,camera.meta,cal,cal_path,mesh)
        viewer.set_scene(packet,mesh);ready=time.monotonic()
        print(f'Initial depth mesh ready: {len(mesh.faces):,} triangles. Camera OFF / released. {folder}',flush=True)
        while not seconds or time.monotonic()-ready<seconds:
            key=viewer.poll()
            if key==ord('q'):break
            if key==ord('s'):viewer.save_view()
            time.sleep(.01)
    except BaseException as exc:
        error=str(exc) or type(exc).__name__;raise
    finally:
        duration=0. if ready is None else time.monotonic()-ready
        if camera:camera.close()
        if viewer:viewer.close()
        atomic_json(folder/'capture_report.json',dict(passed=error is None,error=error,
                    duration_after_mesh_ready_s=duration,
                    camera_released_before_meshing=released is not None,
                    camera_stopped=bool(camera and not camera.thread.is_alive() and
                                        (camera.process is None or camera.process.poll() is not None)),
                    camera_pid=None if not camera or not camera.process else camera.process.pid,
                    camera_exit_code=None if not camera or not camera.process else camera.process.poll(),
                    frames=0 if camera is None else camera.count,
                    captured_sequence=None if header is None else header['sequence'],
                    native_depth_resolution=[1280,800],snapshot_only=True,
                    camera_only=True,can_access=False,motor_commands=0,
                    visualization=None if viewer is None else viewer.report(),
                    warning='This is one-frame visualization, not a collision scene. SDK startup warnings are retained in logs.'))
    return folder/'scene.json'
