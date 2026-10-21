"""Local 3-D software preview. No ROS publishers or robot interfaces."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .common import JOINTS, LEFT_JOINTS
from .model import stl_vertices
from safe_zone.geometry import Model


def preview(plan_path, png=None):
    if png:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, Button
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    path=Path(plan_path).resolve();obj=json.loads(path.read_text())
    req=json.loads((path.parent/'backend_request.json').read_text())
    model=Model(req['urdf']);root=ET.parse(req['urdf']).getroot()
    fig=plt.figure(figsize=(11,8));ax=fig.add_subplot(111,projection='3d')
    fig.subplots_adjust(bottom=.17)
    cloud=None
    if not req['scene'].get('synthetic'):
        cloud=np.fromfile(req['scene']['points_file'],dtype='<f4').reshape(-1,3)
        cloud=cloud[::max(1,len(cloud)//18000)]  # Display only, not collision downsampling.
        ax.scatter(*cloud.T,s=.4,c=cloud[:,2],cmap='viridis',alpha=.4)
    for box in req['scene'].get('boxes',[]):
        size=np.array(box['size']);c=np.array(box['xyz'])
        corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*size/2+c
        ax.scatter(*corners.T,s=45,c='red',marker='s')
    curves=obj.get('cycle') or [dict(time_s=0.,positions_rad=req['start'],phase='BLOCKED')]
    times=np.array([p['time_s'] for p in curves]);values=np.array([p['positions_rad'] for p in curves])
    q=dict(zip(LEFT_JOINTS,req['left']));q['openarmx_left_finger_joint1']=req['left_opening_m']
    artists=[];mesh_entries=[]
    for link in root.findall('link'):
        name=link.get('name')
        for collision in link.findall('collision'):
            geom=collision.find('geometry');mesh=geom.find('mesh')
            if mesh is None: continue
            f=mesh.get('filename').removeprefix('file://')
            if not f.lower().endswith('.stl'): continue
            triangles=stl_vertices(f).astype(float)*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            triangles=triangles[::max(1,len(triangles)//350)]
            origin=collision.find('origin');T=np.eye(4)
            if origin is not None:
                T[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            color='orange' if 'right' in name else 'slategray'
            poly=Poly3DCollection([],alpha=.5,facecolor=color,edgecolor='none');ax.add_collection3d(poly)
            mesh_entries.append((name,triangles,T,poly))
    right_line,=ax.plot([],[],[],color='darkorange',linewidth=3)
    left_line,=ax.plot([],[],[],color='slategray',linewidth=2)
    stick_line,=ax.plot([],[],[],color='crimson',linewidth=4,label='14.8 mm stick collision cylinder')
    ax.set(xlim=(-.65,.9),ylim=(-.9,.6),zlim=(-.15,1.15),xlabel='world X (m)',ylabel='world Y (m)',zlabel='world Z (m)')
    ax.set_box_aspect((1.55,1.5,1.3));ax.view_init(elev=24,azim=-55)
    fig.text(.02,.02,'SOFTWARE ONLY — no motor interface. CAD mount/left stick/depth alignment need physical verification.',fontsize=10,color='darkred')
    def draw(t):
        vals=np.array([np.interp(t,times,values[:,j]) for j in range(7)])
        joints=dict(q,**dict(zip(JOINTS,vals)));frames=model.transforms(joints)
        for side,line in [('right',right_line),('left',left_line)]:
            pts=np.array([frames[f'openarmx_{side}_link{i}'][:3,3] for i in range(8)]+[frames[f'openarmx_{side}_hand'][:3,3]])
            line.set_data_3d(*pts.T)
        T=frames['right_drumstick'];length=obj['model']['stick_length_from_mount_m']
        tip=np.array([[0,0,-length/2],[0,0,length/2]])@T[:3,:3].T+T[:3,3]
        stick_line.set_data_3d(*tip.T)
        for name,triangles,local,poly in mesh_entries:
            T=frames[name]@local;poly.set_verts(triangles@T[:3,:3].T+T[:3,3])
        i=min(np.searchsorted(times,t,side='right')-1,len(curves)-1)
        phase=curves[max(i,0)]['phase']
        if obj.get('status')=='planned' and obj['outward'][-1]['time_s']<=t<=obj['outward'][-1]['time_s']+2:
            phase='HOLD (2 seconds)'
        label='SYNTHETIC TEST SCENE' if obj.get('synthetic') else 'Gemini snapshot — not a live safety monitor'
        if not obj.get('synthetic') and not obj.get('strict_unknown_space'):
            label+=' — UNKNOWN SPACE NOT CHECKED'
        ax.set_title(f"{obj['status']} | {phase} | t={t:.2f}s\n"+label)
        fig.canvas.draw_idle()
    draw(float(times[-1]/2) if png and len(times)>1 else 0.)
    if png:
        fig.savefig(png,dpi=130);plt.close(fig);return
    slider=Slider(fig.add_axes([.15,.08,.65,.03]),'Time',0,max(.01,float(times[-1])),valinit=0)
    slider.on_changed(draw)
    button=Button(fig.add_axes([.82,.07,.13,.05]),'Play / pause')
    playing=[False];button.on_clicked(lambda _:playing.__setitem__(0,not playing[0]))
    timer=fig.canvas.new_timer(interval=50)
    def tick():
        if playing[0]: slider.set_val((slider.val+.05)%max(.01,float(times[-1])))
    timer.add_callback(tick);timer.start();plt.show()
