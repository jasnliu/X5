"""Saved-plan animation with a one-shot depth mesh, or offline rendering."""
import numpy as np

from .common import DEFAULT_CALIBRATION
from .trajectory_view import PlanDisplay, PlaybackClock, RobotArtists


def preview(plan_path,png=None,*,live=True,seconds=0,output=None,
            cal_path=DEFAULT_CALIBRATION,autoplay=False,loop=False):
    plan=PlanDisplay(plan_path)
    # The legacy internal `live` argument means use the camera at startup only.
    # PNG export is always offline.
    if png or not live:
        if seconds or output:raise ValueError('--seconds/--output require a camera snapshot preview')
        return saved_preview(plan,png,autoplay,loop)
    plan.check_live_calibration(cal_path)  # Reject mismatch before opening camera.
    from .snapshot_preview import run
    from .cli import output_folder
    folder=output or output_folder('snapshot-mesh')
    return run(folder,plan,seconds=seconds,cal_path=cal_path,autoplay=autoplay,loop=loop)


def saved_preview(plan,png=None,autoplay=False,loop=False):
    if png:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, Button
    req=plan.req;obj=plan.obj
    fig=plt.figure(figsize=(11,8));ax=fig.add_subplot(111,projection='3d')
    fig.subplots_adjust(bottom=.17)
    if not req['scene'].get('synthetic'):
        cloud=np.fromfile(req['scene']['points_file'],dtype='<f4').reshape(-1,3)
        cloud=cloud[::max(1,len(cloud)//18000)] # Display only, not planner sampling.
        ax.scatter(*cloud.T,s=.4,c=cloud[:,2],cmap='viridis',alpha=.4)
    for box in req['scene'].get('boxes',[]):
        size=np.array(box['size']);c=np.array(box['xyz'])
        corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*size/2+c
        ax.scatter(*corners.T,s=45,c='red',marker='s')
    robot=RobotArtists(ax,plan)
    ax.set(xlim=(-.65,.9),ylim=(-.9,.6),zlim=(-.15,1.15),xlabel='world X (m)',ylabel='world Y (m)',zlabel='world Z (m)')
    ax.set_box_aspect((1.55,1.5,1.3));ax.view_init(elev=24,azim=-55)
    fig.text(.02,.02,'OFFLINE SIMULATION — no camera or motors. CAD mount/left stick/depth alignment need physical verification.',fontsize=10,color='darkred')
    def draw(t):
        robot.draw(t)
        ax.set_title(f"{obj['status']} | {plan.phase(t)} | t={t:.2f}s\n"+plan.warning)
        fig.canvas.draw_idle()
    draw(plan.duration/2 if png else 0.)
    if png:
        fig.savefig(png,dpi=130);plt.close(fig);return
    clock=PlaybackClock(plan.duration);clock.loop=loop
    slider=Slider(fig.add_axes([.15,.08,.65,.03]),'Time',0,max(.01,plan.duration),valinit=0)
    updating=[False]
    def seek(value):
        if not updating[0]:clock.pause();clock.seek(value)
        draw(value)
    slider.on_changed(seek)
    button=Button(fig.add_axes([.82,.07,.13,.05]),'Play / pause')
    def toggle(_):
        if not plan.playable:return
        if clock.playing:clock.pause()
        else:clock.play()
    button.on_clicked(toggle)
    if autoplay and plan.playable:clock.play()
    timer=fig.canvas.new_timer(interval=50)
    def tick():
        if clock.playing:
            updating[0]=True
            try:slider.set_val(clock.sample())
            finally:updating[0]=False
    timer.add_callback(tick);timer.start();plt.show();timer.stop()
