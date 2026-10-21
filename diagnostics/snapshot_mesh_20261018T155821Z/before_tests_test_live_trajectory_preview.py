"""No devices: combined trajectory/depth preview, timing and compatibility."""
import copy
import json
import socket
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from drum_planner import common,depth,cli,preview
from drum_planner.trajectory_view import PlanDisplay,PlaybackClock,RobotArtists
from drum_planner.live_playback import LivePlaybackView
from drum_planner.live_view import build_packet


DEMO=common.ROOT/'planner_runs/software_demo/plan.json'


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    real=socket.socket
    def guarded(family=socket.AF_INET,*args,**kwargs):
        if family==socket.AF_CAN:raise AssertionError('Preview cannot open CAN')
        return real(family,*args,**kwargs)
    def denied(*a,**kw):raise AssertionError('No live camera in offline tests')
    monkeypatch.setattr(socket,'socket',guarded);monkeypatch.setattr(depth,'DepthCamera',denied)


@pytest.fixture(scope='module')
def plan():return PlanDisplay(DEMO)


def packet(seq=5):
    cal=common.calibration();meta=cal['camera']
    frames=[(dict(sequence=seq-4+i,value_scale_m=.001,received_monotonic=10.),np.full((16,24),1000,dtype='<u2')) for i in range(5)]
    return build_packet(meta,frames,cal['T_base_depth'])


def test_plan_preserves_exact_goal_return_and_hold(plan):
    goal=plan.obj['outward'][-1]['time_s']
    assert np.array_equal(plan.positions(0),plan.req['start'])
    assert np.array_equal(plan.positions(goal),plan.req['goal'])
    assert np.array_equal(plan.positions(goal+1),plan.req['goal'])
    assert plan.phase(goal+1)=='HOLD (2 seconds)'
    assert np.array_equal(plan.positions(plan.duration),plan.req['start'])
    assert 'SYNTHETIC' in plan.warning
    assert 'UNKNOWN' in PlanDisplay(common.ROOT/'planner_runs/camera_preview/plan.json').warning


def test_wall_clock_timing_independent_of_render_rate():
    now=[10.];clock=PlaybackClock(10,lambda:now[0]);clock.play()
    now[0]+=2.3;assert clock.sample()==pytest.approx(2.3)
    clock.pause();now[0]+=8;assert clock.sample()==pytest.approx(2.3)
    clock.seek(4);clock.set_rate(2);clock.play();now[0]+=.75
    assert clock.sample()==pytest.approx(5.5)
    now[0]+=10;assert clock.sample()==10 and not clock.playing and clock.completed_cycles==1
    clock.play();assert clock.position==0
    clock.loop=True;now[0]+=12.5
    assert clock.sample()==pytest.approx(5.) and clock.completed_cycles==3
    clock.seek(-2);assert clock.position==0
    with pytest.raises(ValueError):clock.set_rate(float('nan'))


@pytest.mark.parametrize('bad',['execute','model_hash','timestamps','nonfinite'])
def test_bad_plan_rejected_before_camera(tmp_path,bad):
    obj=json.loads(DEMO.read_text());req=json.loads((DEMO.parent/'backend_request.json').read_text())
    if bad=='execute':req['execute']=True
    if bad=='model_hash':obj['model']['calibrated_urdf_sha256']='wrong'
    if bad=='timestamps':obj['cycle'][1]['time_s']=0
    if bad=='nonfinite':obj['cycle'][1]['positions_rad'][0]=float('nan')
    p=tmp_path/'plan.json';p.write_text(json.dumps(obj));(tmp_path/'backend_request.json').write_text(json.dumps(req))
    with pytest.raises(ValueError):preview.preview(p)


def test_live_calibration_mismatch_prevents_capture(tmp_path,plan):
    cal=tmp_path/'other_calibration.json';cal.write_text('{}')
    with pytest.raises(ValueError,match='calibration differ'):preview.preview(DEMO,cal_path=cal)


def test_blocked_plan_has_static_pose_and_no_playback(tmp_path):
    obj=json.loads(DEMO.read_text());obj['status']='blocked_start';obj.pop('cycle')
    (tmp_path/'plan.json').write_text(json.dumps(obj))
    (tmp_path/'backend_request.json').write_bytes((DEMO.parent/'backend_request.json').read_bytes())
    p=PlanDisplay(tmp_path/'plan.json')
    assert not p.playable and p.duration==0 and p.phase(0)=='BLOCKED'
    assert np.array_equal(p.positions(0),p.req['start'])


@pytest.fixture
def gui(tmp_path,monkeypatch,plan):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')
    v=LivePlaybackView(common.calibration(),tmp_path,plan)
    now=[20.];v.clock.now=lambda:now[0];v.clock.last=20.;v.test_time=now
    try:yield v
    finally:v.close()


def test_live_robot_overlay_uses_saved_model_and_stick(gui):
    assert not gui.play_button.isEnabled()
    gui.accept(packet());assert gui.play_button.isEnabled()
    t=gui.plan.obj['outward'][-1]['time_s'];gui.seek(t)
    assert np.array_equal(gui.robot.last_positions,gui.plan.req['goal'])
    frames=gui.plan.frames(t);T=frames['right_drumstick']
    tip=T[:3,3]+T[:3,2]*gui.plan.obj['model']['stick_length_from_mount_m']/2
    assert np.allclose(gui.robot.last_tip,tip)
    report=gui.report()
    assert report['robot_overlay'] and not report['live_collision_revalidation']
    assert not report['motion_authorized'] and not report['hardware_execution_available']
    assert 'SIMULATED ARM' in gui.ax.get_title()


def test_animation_and_depth_freeze_are_independent(gui):
    gui.accept(packet());gui.toggle_play();assert gui.clock.playing
    gui.toggle_freeze();assert gui.frozen
    gui.test_time[0]+=1.;gui.draw_canvas()
    assert gui.clock.position==1. and gui.packet.sequence==5
    assert not gui.accept(packet(15)) and gui.packet.sequence==5
    gui.toggle_freeze();gui.accept(packet(25));assert gui.packet.sequence==25
    gui.toggle_play();assert not gui.clock.playing
    at_pause=gui.clock.position;gui.test_time[0]+=2;gui.accept(packet(50))
    assert gui.clock.position==at_pause and gui.packet.sequence==50


def test_scrub_speed_loop_and_view_persistence(gui):
    from PyQt5 import QtCore,QtTest
    gui.accept(packet());gui.timeline.setValue(2000)
    assert gui.clock.position==2. and not gui.clock.playing
    gui.speed.setCurrentIndex(2);assert gui.clock.rate==2
    gui.loop_box.setChecked(True);assert gui.clock.loop
    gui.window.activateWindow();gui.window.setFocus();gui.app.processEvents()
    QtTest.QTest.keyClick(gui.window,QtCore.Qt.Key_P);gui.app.processEvents();assert gui.clock.playing
    gui.ax.view_init(elev=12,azim=42);limits=gui.ax.get_xlim3d()
    gui.test_time[0]+=.5;gui.accept(packet(50))
    assert gui.clock.position==3. and gui.ax.elev==12 and gui.ax.azim==42
    assert gui.ax.get_xlim3d()==limits
    gui.window.grab().save(str(gui.folder/'combined.png'))
    assert (gui.folder/'combined.png').stat().st_size>1000


def test_focus_clips_only_display_not_packet(gui):
    p=packet();gui.accept(p);raw=p.raw_points.copy()
    pts=np.array([[0,0,0],[10,10,10]])
    assert np.array_equal(gui.display_points(pts),[[0,0,0]])
    gui.fit_needed=True;assert np.array_equal(gui.display_points(pts),pts)
    gui.focus_arm();assert np.array_equal(p.raw_points,raw)


def test_autoplay_waits_for_depth(tmp_path,monkeypatch,plan):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')
    v=LivePlaybackView(common.calibration(),tmp_path,plan,autoplay=True,loop=True)
    try:
        assert not v.clock.playing
        v.accept(packet());assert v.clock.playing and v.clock.loop
        assert not v.autoplay_pending
    finally:v.close()


def test_preview_routes_live_and_offline_without_hardware(monkeypatch,tmp_path):
    calls=[]
    def capture(folder,**kw):calls.append((folder,kw));return Path(folder)/'scene.json'
    monkeypatch.setattr(depth,'capture',capture)
    result=preview.preview(DEMO,seconds=65,output=tmp_path,autoplay=True,loop=True)
    assert result==tmp_path/'scene.json' and calls[0][1]['seconds']==65
    assert callable(calls[0][1]['viewer_factory'])
    offline=[];monkeypatch.setattr(preview,'saved_preview',lambda *a:offline.append(a))
    preview.preview(DEMO,live=False);preview.preview(DEMO,png=str(tmp_path/'p.png'))
    assert len(calls)==1 and len(offline)==2
    with pytest.raises(ValueError):preview.preview(DEMO,live=False,seconds=10)


def test_offline_png_never_opens_camera(tmp_path):
    target=tmp_path/'plan.png'
    preview.preview(DEMO,png=str(target))
    assert target.stat().st_size>10000


def test_cli_preview_live_default_and_camera_opt_out(monkeypatch,tmp_path):
    calls=[]
    monkeypatch.setattr(preview,'preview',lambda *args,**kwargs:calls.append((args,kwargs)))
    assert cli.main(['preview',str(DEMO),'--autoplay','--loop','--seconds','65','--output',str(tmp_path)])==0
    assert calls[-1][1]['live'] and calls[-1][1]['autoplay'] and calls[-1][1]['loop']
    assert cli.main(['preview',str(DEMO),'--no-camera'])==0
    assert not calls[-1][1]['live']
