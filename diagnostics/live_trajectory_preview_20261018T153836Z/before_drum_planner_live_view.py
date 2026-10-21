"""Live, camera-only depth visualization. Display sampling never feeds planning.

The acquisition thread remains independent. A single background job computes a
bounded display packet; no frame-job queue builds up while the user interacts.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
import time

import cv2
import numpy as np

from .depth import depth_window, deproject, occupied_voxels


MAX_DISPLAY_POINTS = 18000


def sampled(points, maximum=MAX_DISPLAY_POINTS):
    """Uniform display decimation AFTER full-resolution obstacle generation."""
    if maximum < 1: raise ValueError('Point budget must be positive')
    if len(points) <= maximum: return points
    return points[np.linspace(0, len(points)-1, maximum, dtype=int)]


def color_depth(depth):
    image=cv2.applyColorMap(np.uint8(np.clip((depth-.15)/4.85,0,1)*255),cv2.COLORMAP_TURBO)
    image[depth==0]=0
    return cv2.cvtColor(image,cv2.COLOR_BGR2RGB)


@dataclass
class DisplayPacket:
    sequence: int
    received: float
    raw_points: np.ndarray
    voxels: np.ndarray
    raw_count: int
    voxel_count: int
    raw_image: np.ndarray
    voxel_image: np.ndarray
    bounds: np.ndarray | None
    compute_seconds: float


def build_packet(meta, frames, transform, maximum=MAX_DISPLAY_POINTS):
    started=time.monotonic()
    _,_,minimum,_=depth_window(frames)
    header,last=frames[-1]
    raw=last.astype(np.float32)*header['value_scale_m']
    raw[(last==0)|(last==65535)|(raw<.15)|(raw>5.)]=0
    raw_points=deproject(raw,meta,transform)
    all_points=deproject(minimum,meta,transform)
    voxels=occupied_voxels(all_points)
    bounds=None if not len(all_points) else np.array([all_points.min(axis=0),all_points.max(axis=0)])
    return DisplayPacket(header['sequence'],header['received_monotonic'],
                         sampled(raw_points,maximum),sampled(voxels,maximum),
                         len(raw_points),len(voxels),color_depth(raw),color_depth(minimum),
                         bounds,time.monotonic()-started)


class LatestCloudWorker:
    """One active job, no waiting jobs. Callers submit the newest camera window."""
    def __init__(self, transform, builder=build_packet):
        self.transform=transform; self.builder=builder
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='depth-display')
        self.future=None; self.last_sequence=None; self.submissions=0

    def submit(self, meta, frames):
        if self.future is not None or len(frames)<5: return False
        seq=frames[-1][0]['sequence']
        if seq==self.last_sequence: return False
        self.last_sequence=seq; self.submissions+=1
        self.future=self.pool.submit(self.builder,meta,frames,self.transform)
        return True

    def take(self):
        if self.future is None or not self.future.done(): return None
        future=self.future; self.future=None
        return future.result()  # Surface errors; never silently show a dead feed.

    def close(self):
        self.pool.shutdown(wait=True,cancel_futures=True)


class LiveDepthView:
    """Qt window polled by the existing capture loop; it has no robot interface."""
    def __init__(self, cal, folder):
        # Lazy GUI imports keep headless capture/headless unit tests GUI-free.
        from PyQt5 import QtCore, QtGui, QtWidgets
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
        from matplotlib.figure import Figure
        self.QtCore=QtCore; self.QtGui=QtGui; self.QtWidgets=QtWidgets
        self.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.app.setQuitOnLastWindowClosed(False)
        self.folder=Path(folder); self.transform=np.array(cal['T_base_depth'])
        self.worker=LatestCloudWorker(self.transform)
        self.packet=None; self.frozen=False; self.close_requested=False; self.save_requested=False
        self.rendered=0; self.first_sequence=None; self.last_sequence=None
        self.first_render_time=None; self.last_render_time=0.; self.max_display_age_s=0.
        self.freeze_count=0; self.resume_count=0; self.mode_switches=0
        self.modes_rendered=set(); self.frame_count=0; self.fit_needed=True; self.closed=False
        self.next_submit=0.; self.last_status=0.; self.latest_received=None
        self.discard_pending=False
        self.render_draw_seconds=[]

        owner=self
        class Window(QtWidgets.QWidget):
            def closeEvent(self,event):
                owner.close_requested=True
                event.accept()

        self.window=Window(); self.window.setWindowTitle('X5 live 3D depth — CAMERA ONLY / no motors')
        self.window.resize(1380,840)
        layout=QtWidgets.QVBoxLayout(self.window)
        banner=QtWidgets.QLabel('LIVE GEMINI DEPTH  •  1280 × 800  •  CAMERA ONLY — NO CAN / NO MOTOR COMMANDS')
        banner.setStyleSheet('font-weight: bold; color: #154c70; padding: 5px;')
        layout.addWidget(banner)
        controls=QtWidgets.QHBoxLayout()
        self.freeze_button=QtWidgets.QPushButton('Freeze view [Space]')
        self.freeze_button.clicked.connect(self.toggle_freeze); controls.addWidget(self.freeze_button)
        controls.addWidget(QtWidgets.QLabel('3D layer:'))
        self.mode=QtWidgets.QComboBox(); self.mode.addItems(['Raw depth points','Obstacle voxels (10 mm)'])
        self.mode.currentIndexChanged.connect(self.change_mode); controls.addWidget(self.mode)
        controls.addWidget(QtWidgets.QLabel('Color:'))
        self.color=QtWidgets.QComboBox(); self.color.addItems(['Camera depth','World height'])
        self.color.currentIndexChanged.connect(self.refresh); controls.addWidget(self.color)
        fit=QtWidgets.QPushButton('Reset / fit view [R]');fit.clicked.connect(self.reset_view);controls.addWidget(fit)
        save=QtWidgets.QPushButton('Save LIVE scene [S]');save.clicked.connect(self.request_save);controls.addWidget(save)
        close=QtWidgets.QPushButton('Save + close [Q]');close.clicked.connect(self.request_close);controls.addWidget(close)
        layout.addLayout(controls)

        splitter=QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        left=QtWidgets.QWidget(); ll=QtWidgets.QVBoxLayout(left)
        self.image_title=QtWidgets.QLabel('Native depth — waiting for five frames…');ll.addWidget(self.image_title)
        self.image=QtWidgets.QLabel();self.image.setMinimumSize(300,200)
        self.image.setAlignment(QtCore.Qt.AlignCenter);self.image.setStyleSheet('background:#111;')
        ll.addWidget(self.image,1)
        info=QtWidgets.QLabel('Black = missing / out-of-range depth, NOT free space.\n'
                             '2D colors: camera distance, 0.15–5 m. No RGB stream.\n'
                             '3D color: selectable distance or world height.\n'
                             'Freeze affects both views, not camera acquisition.\n'
                             'Save always uses the latest LIVE full-resolution frames.')
        info.setWordWrap(True);ll.addWidget(info)
        splitter.addWidget(left)
        right=QtWidgets.QWidget();rl=QtWidgets.QVBoxLayout(right)
        self.figure=Figure(figsize=(8,6));self.canvas=FigureCanvasQTAgg(self.figure)
        self.ax=self.figure.add_subplot(111,projection='3d')
        self.figure.subplots_adjust(left=.01,right=.96,bottom=.13,top=.91)
        self.ax.set(xlabel='Base X (m)',ylabel='Base Y (m)',zlabel='Base Z (m)')
        self.ax.set_box_aspect((1,1,1));self.ax.view_init(elev=24,azim=-55)
        self.cloud=self.ax.scatter([],[],[],s=2,c=[],cmap='turbo',depthshade=False)
        self.colorbar=self.figure.colorbar(self.cloud,ax=self.ax,shrink=.65,pad=.06)
        self.colorbar.set_label('Camera depth (m)')
        self.ax.set_title('Waiting for live depth…')
        self.canvas.mpl_connect('scroll_event',self.zoom)
        rl.addWidget(self.canvas,1)
        guide=QtWidgets.QLabel('Left-drag: rotate   •   Middle-drag: pan   •   Wheel or right-drag: zoom')
        rl.addWidget(guide);splitter.addWidget(right);splitter.setSizes([420,940])
        layout.addWidget(splitter,1)
        self.status=QtWidgets.QLabel('Starting native depth camera…');layout.addWidget(self.status)
        self.detail=QtWidgets.QLabel('Visible surfaces only — hidden and missing regions remain UNKNOWN.')
        self.detail.setWordWrap(True);layout.addWidget(self.detail)
        self.note=QtWidgets.QLabel('VISUALIZATION ONLY — display points are sampled. Planner/saved scenes retain all valid pixels. '
                                  'Voxels are before robot self-filtering/padding; this is not a clearance or safety verdict.')
        self.note.setWordWrap(True);self.note.setStyleSheet('color:#9b2922;');layout.addWidget(self.note)
        self.save_status=QtWidgets.QLabel('Scenes will be saved to: '+str(self.folder));self.save_status.setWordWrap(True)
        layout.addWidget(self.save_status)
        self.shortcuts=[]
        for key,callback in [('Space',self.toggle_freeze),('R',self.reset_view),('S',self.request_save),
                             ('Q',self.request_close),('Escape',self.request_close)]:
            shortcut=QtWidgets.QShortcut(QtGui.QKeySequence(key),self.window)
            shortcut.activated.connect(callback);self.shortcuts.append(shortcut)
        self.window.show();self.app.processEvents()

    def request_save(self):
        self.save_requested=True
        self.save_status.setText('Saving the latest LIVE frames (not the frozen view)…')

    def request_close(self):
        self.close_requested=True

    def saved(self):
        self.save_status.setText('Saved LIVE full-resolution scene: '+str(self.folder/'scene.json'))

    def toggle_freeze(self):
        if self.packet is None: return
        self.frozen=not self.frozen
        if self.frozen:
            self.freeze_count+=1
            self.discard_pending=self.worker.future is not None
        else: self.resume_count+=1
        self.freeze_button.setText('Resume LIVE [Space]' if self.frozen else 'Freeze view [Space]')
        self.refresh()

    def change_mode(self, *_):
        self.mode_switches+=1;self.refresh()

    def reset_view(self):
        self.fit_needed=True;self.ax.view_init(elev=24,azim=-55);self.refresh()

    def zoom(self,event):
        if event.inaxes is not self.ax: return
        factor=.82 if event.button=='up' else 1/.82
        for getter,setter in [(self.ax.get_xlim3d,self.ax.set_xlim3d),(self.ax.get_ylim3d,self.ax.set_ylim3d),
                              (self.ax.get_zlim3d,self.ax.set_zlim3d)]:
            lo,hi=getter();mid=(lo+hi)/2;half=np.clip((hi-lo)*factor/2,.02,20)
            setter(mid-half,mid+half)
        self.canvas.draw_idle()

    def refresh(self,*_):
        packet=self.packet
        if packet is None: return
        voxel=self.mode.currentIndex()==1
        points=packet.voxels if voxel else packet.raw_points
        full_count=packet.voxel_count if voxel else packet.raw_count
        rgb=packet.voxel_image if voxel else packet.raw_image
        self.cloud._offsets3d=(points[:,0],points[:,1],points[:,2])
        height=self.color.currentIndex()==1
        colors=points[:,2] if height else (points-self.transform[:3,3])@self.transform[:3,2]
        self.cloud.set_array(colors)
        lo,hi=(float(colors.min()),float(colors.max())) if height and len(colors) else (.15,5.)
        self.cloud.set_clim(lo,max(lo+.01,hi))
        self.colorbar.set_label('World height (m)' if height else 'Camera depth (m)')
        if self.fit_needed and packet.bounds is not None:
            mid=packet.bounds.mean(axis=0);half=max(.1,float(np.max(np.ptp(packet.bounds,axis=0)))*.53)
            for i,setter in enumerate((self.ax.set_xlim3d,self.ax.set_ylim3d,self.ax.set_zlim3d)):
                setter(mid[i]-half,mid[i]+half)
            self.fit_needed=False
        layer='10 mm occupied voxel centers' if voxel else 'Raw depth returns'
        prefix='FROZEN' if self.frozen else 'LIVE'
        self.ax.set_title(f'{prefix} — {layer}\nFrame {packet.sequence:,} | showing {len(points):,} / {full_count:,}')
        self.image_title.setText(prefix+' — '+('nearest return from 5 frames' if voxel else 'latest displayed depth frame'))
        image=self.QtGui.QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],self.QtGui.QImage.Format_RGB888).copy()
        self.image.setPixmap(self.QtGui.QPixmap.fromImage(image).scaled(
            self.image.size(),self.QtCore.Qt.KeepAspectRatio,self.QtCore.Qt.SmoothTransformation))
        self.detail.setText(('Five-frame obstacle union, 10 mm voxel centers. ' if voxel else 'Single-frame raw returns. ')+
                            'Empty/hidden regions are UNKNOWN; no reconstructed back surfaces or robot pose is implied.')
        self.modes_rendered.add('voxels' if voxel else 'raw')
        # Synchronous bounded draw ensures at most one queued display job and
        # makes render timing honest; acquisition never runs on this thread.
        start=time.monotonic();self.canvas.draw()
        self.render_draw_seconds.append(time.monotonic()-start)
        self.render_draw_seconds=self.render_draw_seconds[-1000:]

    def accept(self,packet):
        if self.frozen: return False
        self.packet=packet;self.rendered+=1
        now=time.monotonic()
        if self.first_sequence is None:self.first_sequence=packet.sequence;self.first_render_time=now
        self.last_sequence=packet.sequence;self.last_render_time=now
        self.max_display_age_s=max(self.max_display_age_s,now-packet.received)
        self.refresh();return True

    def poll(self,camera):
        self.app.processEvents()
        now=time.monotonic();self.frame_count=camera.count
        self.latest_received=camera.latest[0]['received_monotonic'] if camera.latest else None
        packet=self.worker.take()
        # Frozen views discard completed work; resuming requests the newest
        # camera window, never a backlog of frames collected while frozen.
        if packet is not None:
            if not self.discard_pending:self.accept(packet)
            self.discard_pending=False
        if not self.frozen and now>=self.next_submit and not self.close_requested:
            if camera.meta and self.worker.submit(camera.meta,list(camera.frames)):
                self.next_submit=now+.25
        if now-self.last_status>.15:
            self.last_status=now
            age='—' if self.packet is None else f'{now-self.packet.received:.2f} s'
            live_age='—' if self.latest_received is None else f'{now-self.latest_received:.2f} s'
            state='FROZEN — acquisition continues' if self.frozen else 'LIVE (latest available; no frame queue)'
            self.status.setText(f'{state} | acquired {camera.count:,} frames | 3D updates {self.rendered:,} | '
                                f'camera age {live_age} | displayed age {age}')
        if self.close_requested:return ord('q')
        if self.save_requested and len(camera.frames)>=5:
            self.save_requested=False;return ord('s')
        return -1

    def report(self):
        return dict(type='live_3d',rendered_updates=self.rendered,
                    first_displayed_sequence=self.first_sequence,last_displayed_sequence=self.last_sequence,
                    max_age_at_display_s=self.max_display_age_s,
                    last_frame_compute_s=None if self.packet is None else self.packet.compute_seconds,
                    maximum_draw_s=max(self.render_draw_seconds,default=0.),
                    worker_submissions=self.worker.submissions,maximum_pending_jobs=1,
                    modes_rendered=sorted(self.modes_rendered),freeze_count=self.freeze_count,resume_count=self.resume_count,
                    mode_switches=self.mode_switches,display_point_limit=MAX_DISPLAY_POINTS,
                    planning_points_downsampled=False,robot_overlay=False)

    def close(self):
        if self.closed:return
        self.closed=True;self.worker.close();self.window.close();self.app.processEvents()
