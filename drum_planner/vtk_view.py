"""Qt/OpenGL display only: retained meshes, GPU depth testing, no devices.

Meshes are uploaded once. Camera navigation changes the view, and robot playback
changes actor matrices, never the model/scene geometry or planning data.
"""
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from vtkmodules import vtkRenderingOpenGL2, vtkInteractionStyle  # register OpenGL/interaction factories
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.vtkCommonCore import vtkPoints, vtkLookupTable
from vtkmodules.vtkCommonDataModel import vtkPolyData, vtkCellArray, vtkPlane
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkRenderingCore import vtkActor, vtkPolyDataMapper, vtkRenderer, vtkRenderWindow, vtkWindowToImageFilter
from vtkmodules.vtkRenderingAnnotation import vtkCubeAxesActor, vtkScalarBarActor
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy

from .visual_mesh import visual_surfaces


class MeshLayer:
    def __init__(self, viewport, *, lighting=False, points=False):
        self.viewport=viewport;self.polydata=vtkPolyData();self.mapper=vtkPolyDataMapper()
        self.mapper.SetInputData(self.polydata);self.mapper.SetColorModeToDirectScalars()
        self.mapper.SetScalarModeToUseCellData();self.mapper.ScalarVisibilityOn()
        self.actor=vtkActor();self.actor.SetMapper(self.mapper)
        prop=self.actor.GetProperty();prop.SetOpacity(1.);prop.SetLighting(lighting)
        prop.SetAmbient(.8);prop.SetDiffuse(.2);prop.SetSpecular(0.)
        prop.BackfaceCullingOff();prop.FrontfaceCullingOff();prop.EdgeVisibilityOff()
        prop.SetPointSize(2.);prop.SetInterpolationToFlat()
        self.points=points;self.geometry_uploads=0;self.color_uploads=0;self.transform_updates=0
        self.triangle_count=0;self._last_transform=None
        self.mapper.AddObserver('ErrorEvent',viewport.record_error)
        viewport.renderer.AddActor(self.actor)

    def set_geometry(self, vertices, faces=None):
        vertices=np.asarray(vertices,dtype=np.float64).reshape(-1,3)
        points=vtkPoints();points.SetData(numpy_to_vtk(vertices,deep=True))
        self.polydata.SetPoints(points)
        cells=vtkCellArray()
        if faces is None:
            indices=np.column_stack((np.ones(len(vertices),dtype=np.int64),np.arange(len(vertices),dtype=np.int64)))
            cells.SetCells(len(vertices),numpy_to_vtkIdTypeArray(indices.ravel(),deep=True))
            self.polydata.SetVerts(cells);self.triangle_count=0
        else:
            faces=np.asarray(faces,dtype=np.int64).reshape(-1,3)
            indices=np.column_stack((np.full(len(faces),3,dtype=np.int64),faces))
            cells.SetCells(len(faces),numpy_to_vtkIdTypeArray(indices.ravel(),deep=True))
            self.polydata.SetPolys(cells);self.triangle_count=len(faces)
        self.polydata.Modified();self.geometry_uploads+=1

    def set_colors(self, colors):
        colors=np.asarray(colors)
        if colors.dtype!=np.uint8:colors=np.uint8(np.rint(np.clip(colors,0,1)*255))
        scalars=numpy_to_vtk(np.ascontiguousarray(colors),deep=True);scalars.SetName('display_rgba')
        self.polydata.GetCellData().SetScalars(scalars);self.color_uploads+=1

    def set_transform(self, transform):
        if self._last_transform is not None and np.array_equal(transform,self._last_transform):return
        matrix=vtkMatrix4x4();matrix.DeepCopy(np.asarray(transform,dtype=float).ravel())
        self.actor.SetUserMatrix(matrix);self._last_transform=np.array(transform,copy=True)
        self.transform_updates+=1

    def set_visible(self,visible):self.actor.SetVisibility(bool(visible))
    def get_visible(self):return bool(self.actor.GetVisibility())


class VtkViewport(QtWidgets.QWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.errors=[];self.closed=False;self.frame_times=[];self.render_count=0;self.capability_lines=[]
        self.surface_source=None;self.surface_color_key=None;self.title=''
        layout=QtWidgets.QVBoxLayout(self);layout.setContentsMargins(0,0,0,0)
        self.title_label=QtWidgets.QLabel();self.title_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.title_label)
        self.render_window=vtkRenderWindow();self.render_window.SetMultiSamples(4)
        self.renderer=vtkRenderer();self.renderer.SetBackground(1.,1.,1.)
        self.render_window.AddRenderer(self.renderer)
        for obj in (self.render_window,self.renderer):obj.AddObserver('ErrorEvent',self.record_error)
        # Qt's offscreen plugin has no native X window to embed into. Tests use
        # an offscreen VTK context + image label, never a fake renderer. Normal
        # desktop use embeds the OpenGL window directly, with no image copies.
        self.offscreen=QtWidgets.QApplication.platformName() in ('offscreen','minimal')
        if self.offscreen:
            self.render_window.SetOffScreenRendering(1)
            self.widget=QtWidgets.QLabel();self.widget.setMinimumSize(500,350)
            self.widget.setAlignment(QtCore.Qt.AlignCenter)
        else:
            self.widget=QVTKRenderWindowInteractor(self,rw=self.render_window)
            self.widget.SetInteractorStyle(vtkInteractorStyleTrackballCamera())
            # Suppress VTK's built-in W/S wireframe/surface shortcuts so the
            # application's Save shortcut never changes rendering quality.
            self.widget.GetInteractorStyle().AddObserver('CharEvent',lambda *_:None)
            self.widget.Initialize()
        layout.addWidget(self.widget,1)
        self.camera=self.renderer.GetActiveCamera()
        self.axes=vtkCubeAxesActor();self.axes.SetCamera(self.camera)
        self.axes.SetXTitle('Base X (m)');self.axes.SetYTitle('Base Y (m)');self.axes.SetZTitle('Base Z (m)')
        self.axes.SetFlyModeToOuterEdges();self.axes.SetGridLineLocation(2)
        self.axes.DrawXGridlinesOn();self.axes.DrawYGridlinesOn();self.axes.DrawZGridlinesOn()
        self.axes.XAxisMinorTickVisibilityOff();self.axes.YAxisMinorTickVisibilityOff();self.axes.ZAxisMinorTickVisibilityOff()
        for i in range(3):
            self.axes.GetTitleTextProperty(i).SetColor(.15,.15,.15)
            self.axes.GetTitleTextProperty(i).SetFontSize(14)
            self.axes.GetLabelTextProperty(i).SetColor(.25,.25,.25)
            self.axes.GetLabelTextProperty(i).SetFontSize(12)
        for axis in ('X','Y','Z'):
            getattr(self.axes,f'Get{axis}AxesLinesProperty')().SetColor(.5,.5,.5)
            getattr(self.axes,f'Get{axis}AxesGridlinesProperty')().SetColor(.85,.85,.85)
        self.renderer.AddActor(self.axes)
        from matplotlib import colormaps
        self.colors=colormaps['turbo'](np.linspace(0,1,256))
        self.lut=vtkLookupTable();self.lut.SetNumberOfTableValues(256);self.lut.Build()
        for i,color in enumerate(self.colors):self.lut.SetTableValue(i,*color)
        self.colorbar=vtkScalarBarActor();self.colorbar.SetLookupTable(self.lut)
        self.colorbar.SetPosition(.88,.23);self.colorbar.SetWidth(.1);self.colorbar.SetHeight(.55)
        self.colorbar.SetNumberOfLabels(5)
        self.colorbar.UnconstrainedFontSizeOn()
        for prop in (self.colorbar.GetTitleTextProperty(),self.colorbar.GetLabelTextProperty()):
            prop.SetColor(.1,.1,.1);prop.SetFontSize(12);prop.ItalicOff();prop.BoldOff();prop.ShadowOff()
        self.renderer.AddActor2D(self.colorbar)
        self.cloud=MeshLayer(self,points=True);self.surface=MeshLayer(self)
        self.surface.set_visible(False)
        self.fit(np.array([[-1.,-1.,-.5],[1.,1.,1.5]]))
        self.set_color_range(.15,5.,'Camera depth (m)')

    def record_error(self,caller,event):self.errors.append(caller.GetClassName()+': '+event)

    def set_title(self,text):
        self.title=text;self.title_label.setText(text)

    def set_color_range(self,lo,hi,label):
        self.color_range=(float(lo),max(float(lo)+.01,float(hi)))
        self.color_label=label;self.lut.SetTableRange(*self.color_range)
        self.colorbar.SetTitle(label)

    def rgba(self,values):
        lo,hi=self.color_range
        indices=np.minimum((np.clip((np.asarray(values)-lo)/(hi-lo),0,1)*256).astype(int),255)
        return self.colors[indices]

    def set_points(self,points,values,visible=True):
        self.cloud.set_geometry(points);self.cloud.set_colors(self.rgba(values));self.cloud.set_visible(visible)

    def set_surface(self,mesh,values,color_key,visible=True):
        if self.surface_source is not mesh:
            self.surface.set_geometry(mesh.vertices,mesh.faces)
            self.surface_source=mesh;self.surface_color_key=None
        key=(color_key,self.color_range)
        if self.surface_color_key!=key:
            self.surface.set_colors(self.rgba(np.asarray(values)[mesh.faces].mean(axis=1)))
            self.surface_color_key=key
        self.surface.set_visible(visible)

    def fit(self,bounds,elev=24.,azim=-55.):
        self.bounds=np.asarray(bounds,dtype=float).copy()
        self.axes.SetBounds(*self.bounds.T.ravel())
        # Preserve the previous display-only world-box clipping, but do it on
        # the GPU. All scene triangles stay uploaded and available after refit.
        for layer in (self.cloud,self.surface):
            layer.mapper.RemoveAllClippingPlanes()
            for axis in range(3):
                for end,sign in ((0,1.),(1,-1.)):
                    normal=np.zeros(3);normal[axis]=sign
                    plane=vtkPlane();plane.SetNormal(*normal);plane.SetOrigin(*self.bounds[end])
                    layer.mapper.AddClippingPlane(plane)
        self.renderer.ResetCamera(*self.bounds.T.ravel())
        self.set_angles(elev,azim)

    def set_angles(self,elev,azim):
        center=np.array(self.camera.GetFocalPoint());distance=self.camera.GetDistance()
        e,a=np.deg2rad([elev,azim]);direction=np.array([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)])
        self.camera.SetPosition(*(center+max(distance,.01)*direction));self.camera.SetViewUp(0,0,1)
        self.renderer.ResetCameraClippingRange()

    def zoom(self,factor):
        self.camera.Dolly(float(factor));self.renderer.ResetCameraClippingRange();self.render()

    def render(self):
        if self.closed:return
        start=time.perf_counter()
        if self.offscreen:self.render_window.SetSize(max(1,self.widget.width()),max(1,self.widget.height()))
        self.renderer.ResetCameraClippingRange();self.render_window.Render()
        if self.errors:raise RuntimeError('VTK rendering failed: '+'; '.join(self.errors))
        if not self.capability_lines:
            self.capability_lines=[s.strip() for s in self.render_window.ReportCapabilities().splitlines()
                                   if s.startswith(('OpenGL vendor','OpenGL renderer','OpenGL version'))]
        self.render_count+=1;self.frame_times.append(time.perf_counter()-start);self.frame_times=self.frame_times[-1000:]
        if self.offscreen:self.widget.setPixmap(QtGui.QPixmap.fromImage(self.capture_image()))

    def capture_image(self):
        capture=vtkWindowToImageFilter();capture.SetInput(self.render_window)
        capture.SetInputBufferTypeToRGB();capture.ReadFrontBufferOff();capture.ShouldRerenderOff();capture.Update()
        out=capture.GetOutput();w,h,_=out.GetDimensions()
        rgb=np.ascontiguousarray(vtk_to_numpy(out.GetPointData().GetScalars()).reshape(h,w,3)[::-1])
        return QtGui.QImage(rgb.data,w,h,rgb.strides[0],QtGui.QImage.Format_RGB888).copy()

    def save_window(self,window,path):
        # Native OpenGL buffers are not reliably included in QWidget.grab on
        # every Qt/X11 driver. Composite the actual framebuffer into the UI shot.
        self.render();shot=window.grab();painter=QtGui.QPainter(shot)
        corner=self.widget.mapTo(window,QtCore.QPoint(0,0))
        painter.drawImage(QtCore.QRect(corner,self.widget.size()),self.capture_image());painter.end()
        if not shot.save(str(path)):raise RuntimeError('Could not save preview image')

    def report(self):
        return dict(engine='VTK/OpenGL',capabilities=list(self.capability_lines),qt_offscreen=self.offscreen,
                    render_count=self.render_count,maximum_frame_s=max(self.frame_times,default=0.),
                    median_frame_s=float(np.median(self.frame_times)) if self.frame_times else None,
                    surface_geometry_uploads=self.surface.geometry_uploads,surface_color_uploads=self.surface.color_uploads,
                    surface_triangles=self.surface.triangle_count,render_errors=list(self.errors))

    def shutdown(self):
        if self.closed:return
        self.closed=True
        if not self.offscreen:
            self.widget.Disable();self.widget.Finalize();self.widget.close()
        else:self.render_window.Finalize()


class RobotActors:
    """Same URDF visual surfaces as the old preview, unchanged triangle counts."""
    def __init__(self,viewport,plan):
        self.viewport=viewport;self.plan=plan;self.mesh_entries=[]
        self.last_positions=None;self.last_tip=None
        for name,triangles,local,colors in visual_surfaces(plan.root):
            layer=MeshLayer(viewport,lighting=True)
            layer.set_geometry(triangles.reshape(-1,3),np.arange(triangles.size//3).reshape(-1,3))
            layer.set_colors(colors);self.mesh_entries.append((name,triangles,local,layer))
            if name=='right_drumstick':self.stick_artist=layer

    def set_visible(self,visible):
        for _,_,_,layer in self.mesh_entries:layer.set_visible(visible)

    def draw(self,t):
        self.draw_frames(self.plan.frames(t),self.plan.positions(t))

    def draw_joints(self,joints):
        from .common import JOINTS
        self.draw_frames(self.plan.model.transforms(joints),np.array([joints[j] for j in JOINTS]))

    def draw_frames(self,frames,positions):
        self.last_positions=positions
        T=frames['right_drumstick'];self.last_tip=T[:3,3]+T[:3,2]*self.plan.obj['model']['stick_length_from_mount_m']/2
        for name,_,local,layer in self.mesh_entries:layer.set_transform(frames[name]@local)

    def focus_bounds(self):
        # Shared viewing envelope only; this never changes motion constraints.
        from .trajectory_view import RobotArtists
        return RobotArtists.focus_bounds(self)
