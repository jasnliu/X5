"""Offline-only CAN wire audit, snapshot synchronization and alignment display."""
import io
import json
from pathlib import Path
import socket
from types import SimpleNamespace

import numpy as np
import pytest

from safe_zone import encoder
from safe_zone.gripper_feedback import EncoderState,motor8_feedback
from drum_planner import common,read_only as ro,hardware_check as hw,cli


@pytest.fixture(autouse=True)
def forbid_devices(monkeypatch):
    real=socket.socket
    def guarded(family=socket.AF_INET,*args,**kwargs):
        if family==socket.AF_CAN:raise AssertionError('Offline test attempted real CAN access')
        return real(family,*args,**kwargs)
    monkeypatch.setattr(socket,'socket',guarded)
    monkeypatch.setattr(hw,'DepthCamera',lambda *a,**kw:(_ for _ in ()).throw(AssertionError('No real camera in unit tests')))


def feedback(motor,count=32767,mode=0,fault=0):
    cid=encoder.EFF|(2<<24)|(mode<<22)|(fault<<16)|(motor<<8)|0xfd
    return encoder.FRAME.pack(cid,8,count.to_bytes(2,'big')+b'\x80\x00\x80\x00\x00\xfa')


class Socket:
    def __init__(self,*a,**kw):self.pending=[];self.sent=[];self.closed=False
    def send(self,frame):
        assert frame in ro.ALLOWED;self.sent.append(frame)
        motor=encoder.FRAME.unpack(frame)[0]&255
        self.pending.append(feedback(motor,32760+motor));return 16
    def recv(self,size):
        if not self.pending:raise BlockingIOError()
        return self.pending.pop(0)
    def fileno(self):return 12
    def close(self):self.closed=True
    def bind(self,address):self.address=address
    def setblocking(self,value):pass
    def setsockopt(self,*a):pass


def test_exact_wire_allowlist_and_no_delegated_send_bypass():
    sock=Socket();tx=io.StringIO();rx=io.StringIO();gate=ro.QuerySocket(sock,'can0',tx,rx)
    for m in range(1,9):
        frame=encoder.request_frame(m)
        assert gate.send(frame)==16
        cid,dlc,data=encoder.FRAME.unpack(frame)
        assert cid==0x8200fd00|m and dlc==8 and data==bytes.fromhex('0100000000000000')
        gate.recv(16)
    assert gate.sent==8 and gate.feedback[8]['mode']==0 and gate.feedback[8]['fault_bits']==0
    for kind in (1,3,4,6,17,18):
        with pytest.raises(RuntimeError,match='non-query'):
            gate.send(encoder.FRAME.pack(encoder.EFF|(kind<<24)|0xfd01,8,bytes(8)))
    for data in (bytes(16),encoder.request_frame(1)[:-1],bytearray(encoder.request_frame(1))):
        with pytest.raises(RuntimeError):gate.send(data)
    with pytest.raises(AttributeError):gate.sendall(b'anything')
    assert len(sock.sent)==8
    before=len(sock.sent);gate.close();assert sock.closed and len(sock.sent)==before
    audit=[json.loads(s) for s in tx.getvalue().splitlines()]
    assert len(audit)==16 and sum(e['event']=='sent' for e in audit)==8


def test_unexpected_controller_traffic_is_not_ignored():
    s=Socket();s.pending=[encoder.FRAME.pack(encoder.EFF|(3<<24)|0xfd01,8,bytes(8))]
    gate=ro.QuerySocket(s,'can0',io.StringIO(),io.StringIO())
    with pytest.raises(RuntimeError,match='other controllers'):gate.recv(16)


@pytest.fixture
def fake_can(tmp_path,monkeypatch):
    sockets=[];real_open=encoder.os.open
    def opened(path,*a,**kw):
        if str(path).startswith('/tmp/openarmx-cartesian-'):path=tmp_path/Path(path).name
        return real_open(path,*a,**kw)
    def factory(*a,**kw):
        s=Socket();sockets.append(s);return s
    monkeypatch.setattr(encoder.os,'open',opened)
    monkeypatch.setattr(encoder.socket,'socket',factory)
    monkeypatch.setattr(encoder.select,'select',lambda r,w,e,t:([s for s in r if s._socket.pending],[],[]))
    return sockets


def test_existing_observer_conversion_disabled_states_and_audited_close(tmp_path,fake_can):
    obs=ro.ReadOnlyObserver(tmp_path)
    try:
        sample=obs.sample()
        for side in ('left','right'):
            for m in range(1,8):
                raw=(32760+m)/65535*25.14-12.57
                assert sample['joints'][f'openarmx_{side}_joint{m}']==encoder.encoder_to_joint(side,m,raw)
            assert sample['motor8_feedback'][side]['encoder_count']==32768
        assert sample['all_16_motors_disabled']
        assert sum(len(s.sent) for s in fake_can)==16
    finally:obs.close()
    assert all(s.closed for s in fake_can)
    assert sum(len(s.sent) for s in fake_can)==16  # close never sends disable/relax
    assert obs.report()['closed'] and obs.report()['mode_changes']==0
    assert len((tmp_path/'can_rx_audit.jsonl').read_text().splitlines())==16


@pytest.mark.parametrize('mode,fault',[(2,0),(0,1)])
def test_enabled_or_faulting_motor_stops_without_relax(tmp_path,fake_can,monkeypatch,mode,fault):
    old=Socket.send
    def send(self,frame):
        count=old(self,frame);motor=encoder.FRAME.unpack(frame)[0]&255
        if motor==3:self.pending[-1]=feedback(motor,mode=mode,fault=fault)
        return count
    monkeypatch.setattr(Socket,'send',send)
    obs=ro.ReadOnlyObserver(tmp_path)
    with pytest.raises(RuntimeError):obs.sample()
    obs.close()
    assert all(frame in ro.ALLOWED for s in fake_can for frame in s.sent)


def test_second_bus_lock_failure_closes_without_any_packet(tmp_path,fake_can,monkeypatch):
    real=encoder.fcntl.flock;calls=[0]
    def flock(fd,flags):
        calls[0]+=1
        if calls[0]==2:raise BlockingIOError('owned by another controller')
        return real(fd,flags)
    monkeypatch.setattr(encoder.fcntl,'flock',flock)
    with pytest.raises(BlockingIOError):ro.ReadOnlyObserver(tmp_path)
    assert fake_can and all(s.closed and not s.sent for s in fake_can)


def samples(end=101.,n=31):
    out=[]
    for t in end-np.arange(n-1,-1,-1)*.05:
        joints=dict(zip(common.JOINTS+common.LEFT_JOINTS,[0.]*14))
        joints.update(openarmx_left_finger_joint1=0.,openarmx_right_finger_joint1=0.)
        out.append(dict(start=float(t-.005),end=float(t),joints=joints,
                   motor8_feedback={s:motor8_feedback(32767) for s in ('right','left')},
                   feedback={s:[dict(motor=m,mode=0,fault_bits=0) for m in range(1,9)] for s in ('right','left')},
                   all_16_motors_disabled=True))
    return out


def test_stationary_real_sample_selection_no_angle_interpolation():
    batch=samples();selected,e=ro.select_stationary(batch,100.2)
    assert selected in batch and e['nearest_midpoint_skew_s']<=.1
    assert e['bracketing_indices'][0]<e['bracketing_indices'][1]
    assert ro.select_stationary(batch)[0] is batch[-1]


@pytest.mark.parametrize('bad',['movement','gap','enabled','nan','unbracketed','short'])
def test_bad_sync_rejected(bad):
    batch=samples();stamp=100.2
    if bad=='movement':batch[-1]['joints'][common.JOINTS[2]]=.1
    if bad=='gap':batch=batch[:10]+batch[17:]
    if bad=='enabled':batch[3]['all_16_motors_disabled']=False
    if bad=='nan':batch[5]['joints'][common.JOINTS[0]]=float('nan')
    if bad=='unbracketed':stamp=110
    if bad=='short':batch=batch[:5]
    with pytest.raises(ValueError):ro.select_stationary(batch,stamp)


def test_state_retains_raw_gripper_does_not_claim_command_or_support(tmp_path):
    sample,e=ro.select_stationary(samples())
    sample['joints'][common.JOINTS[0]]=10.
    state=hw.measured_state(sample,e,common.DEFAULT_CALIBRATION)
    path=tmp_path/'state.json';path.write_text(json.dumps(state))
    assert common.read_state(path)['right_rad'][0]==10.
    assert state['measured'] and state['preview_only'] and not state['supported_rest_verified']
    assert state['right_gripper_closed_motor_joint_rad']==float(np.radians(7))
    assert state['motor8_feedback']['right']['raw_rad']!=state['right_gripper_closed_motor_joint_rad']
    assert state['gripper_commands']==0
    assert not hw.limit_report(state,common.ROOT/'model/openarmx.urdf')['within_urdf_limits']


def test_visibility_is_not_alignment_or_clearance():
    raw=np.full((10,10),1000,np.uint16)
    meta={'depth':{'K':[[10,0,5],[0,10,5],[0,0,1]]}}
    r=hw.coverage_report({'inside':np.array([0,0,1]),'outside':np.array([4,0,1]),'behind':np.array([0,0,-1])},meta,np.eye(4),raw,.001)
    assert r['landmarks']['inside']['inside_depth_image']
    assert r['landmarks']['inside']['observed_surface_depth_m']==1.
    assert not r['landmarks']['outside']['inside_depth_image']
    assert r['landmarks']['behind']['pixel_uv'] is None
    assert not r['alignment_verified'] and not r['clearance_verified']


@pytest.fixture
def mocked_run(tmp_path,monkeypatch):
    now=[100.];objects=[]
    monkeypatch.setattr(hw,'time',SimpleNamespace(monotonic=lambda:now[0],
        sleep=lambda seconds:now.__setitem__(0,now[0]+seconds),
        strftime=hw.time.strftime,gmtime=hw.time.gmtime))
    class Worker:
        def __init__(self,folder):
            self.error=None;self.count=31;self.closed=False;self.thread=SimpleNamespace(is_alive=lambda:not self.closed)
            for f in ('encoder_batches.jsonl','can_query_audit.jsonl','can_rx_audit.jsonl'):(folder/f).write_text('mock unit test\n')
            objects.append(self)
        def samples(self):return samples(end=now[0])
        def close(self):self.closed=True
        def report(self):return dict(mock=True,thread_stopped=self.closed)
    class Camera:
        def __init__(self,folder,cal,*,single_frame):
            assert single_frame;self.error=None;self.count=1;self.closed=False;self.meta=dict(cal['camera'],protocol=1,alignment='native_depth',depth_stream_enabled=True,color_stream_enabled=False)
            self.latest=({'sequence':1,'received_monotonic':now[0],'value_scale_m':.001},np.full((800,1280),1000,dtype='<u2'))
            self.thread=SimpleNamespace(is_alive=lambda:not self.closed)
            self.process=SimpleNamespace(poll=lambda:0 if self.closed else None)
            objects.append(self)
        def close(self):self.closed=True
    monkeypatch.setattr(hw,'Telemetry',Worker);monkeypatch.setattr(hw,'DepthCamera',Camera)
    return tmp_path/'run',objects


def test_full_headless_lifecycle_closes_devices_saves_measured_snapshot(mocked_run):
    folder,objects=mocked_run
    report=hw.run(folder,query_seconds=1.,headless=True)
    r=json.loads(report.read_text())
    assert r['passed'] and r['encoder_sockets_closed'] and r['camera_released']
    assert not r['execution_ready'] and r['motor_commands']==0
    assert all(o.closed for o in objects)
    f,cal,state,model,manifest=hw.load_snapshot(folder)
    assert state['measured'] and not state['source'].startswith('EXAMPLE') and manifest['camera_present']
    assert (f/'camera/surface_mesh.npz').exists()
    with (folder/'measured_start.json').open('a') as out:out.write(' ')
    with pytest.raises(ValueError,match='hash'):hw.load_snapshot(folder)


def test_encoder_only_does_not_open_camera(mocked_run):
    folder,objects=mocked_run
    hw.run(folder,query_seconds=1.,headless=True,no_camera=True)
    assert len(objects)==1 and objects[0].closed and not (folder/'camera').exists()
    assert not hw.load_snapshot(folder)[4]['camera_present']


@pytest.mark.parametrize('bad',[0,-1,float('nan'),float('inf'),601])
def test_invalid_duration_before_device_access(tmp_path,bad):
    with pytest.raises(ValueError):hw.run(tmp_path/'run',query_seconds=bad,headless=True)
    assert not (tmp_path/'run').exists()


def test_cli_never_exposes_motion_flag(monkeypatch,tmp_path):
    calls=[];monkeypatch.setattr(hw,'run',lambda *a,**kw:calls.append((a,kw)))
    assert cli.main(['hardware-check','--headless','--no-camera','--query-seconds','65','--output',str(tmp_path)])==0
    assert calls[0][1]['query_seconds']==65 and calls[0][1]['no_camera']
    with pytest.raises(SystemExit):cli.main(['hardware-check','--execute'])
    for name in ('read_only.py','hardware_check.py','alignment_view.py'):
        text=(common.ROOT/'drum_planner'/name).read_text()
        assert 'from centering' not in text and 'import rclpy' not in text
        assert '.enable(' not in text and '.relax(' not in text


def test_alignment_gui_is_static_measured_pose_and_replay_is_device_free(mocked_run,monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')
    folder,objects=mocked_run;hw.run(folder,query_seconds=1.,headless=True)
    from drum_planner import alignment_view as av
    folder,cal,state,model,manifest=hw.load_snapshot(folder)
    packet,mesh=av.load_scene(folder,cal)
    coverage=json.loads((folder/'coverage.json').read_text())
    v=av.AlignmentView(cal,folder,av.MeasuredPose(folder,state,model),coverage)
    try:
        v.set_scene(packet,mesh)
        assert np.array_equal(v.robot.last_positions,state['right_rad'])
        assert not v.plan.playable and not v.play_button.isVisible()
        assert 'ENCODER-SNAPSHOT' in v.viewport.title and 'CAN CLOSED' in v.banner.text()
        v.toggle_play();v.seek(2);assert not v.clock.playing and v.clock.position==0
        for index in (1,2,0):v.mode.setCurrentIndex(index)
        v.overlay_box.setChecked(False);assert not v.robot.stick_artist.get_visible()
        assert all(not artist.get_visible() for _,_,_,artist in v.robot.mesh_entries)
        v.overlay_box.setChecked(True);assert v.robot.stick_artist.get_visible()
        assert all(artist.get_visible() for _,_,_,artist in v.robot.mesh_entries)
        v.color.setCurrentIndex(1);v.reset_view();v.focus_arm()
        v.save_view();assert (folder/'preview_view.png').stat().st_size>1000
        report=v.report();assert not report['can_access'] and not report['camera_access']
        assert report['animation_draws']==0 and report['rendered_updates']==1
        assert report['mesh_builds']==0 and report['mesh_loads']==1 and report['historical_snapshot']
    finally:v.close()
    count=len(objects)
    report=av.show(folder,seconds=.02)
    assert len(objects)==count and report['loaded_saved_mesh']


@pytest.mark.parametrize('failure',['encoder','camera'])
def test_startup_error_closes_resources_and_never_saves_fake_state(mocked_run,monkeypatch,failure):
    folder,objects=mocked_run
    if failure=='encoder':
        original=hw.Telemetry
        def worker(*a):
            w=original(*a);w.count=0;w.error='Missing fresh feedback';return w
        monkeypatch.setattr(hw,'Telemetry',worker)
    else:
        original=hw.DepthCamera
        def camera(*a,**kw):
            c=original(*a,**kw);c.latest=None;c.error='camera unavailable';return c
        monkeypatch.setattr(hw,'DepthCamera',camera)
    with pytest.raises(RuntimeError):hw.run(folder,query_seconds=1.,headless=True)
    assert all(o.closed for o in objects)
    assert not (folder/'measured_start.json').exists()
    report=json.loads((folder/'preflight_report.json').read_text())
    assert not report['passed'] and not report['execution_ready'] and report['mode_changes']==0


def test_short_query_write_fails_and_is_audited():
    s=Socket();s.send=lambda frame:8
    tx=io.StringIO();gate=ro.QuerySocket(s,'can0',tx,io.StringIO())
    with pytest.raises(RuntimeError,match='Short'):gate.send(encoder.request_frame(1))
    rows=[json.loads(row) for row in tx.getvalue().splitlines()]
    assert rows[-1]['bytes_sent']==8 and gate.sent==0


def test_overtemperature_stops_read_only_observer(tmp_path,fake_can,monkeypatch):
    old=Socket.send
    def send(self,frame):
        count=old(self,frame)
        cid,dlc,payload=encoder.FRAME.unpack(self.pending[-1])
        self.pending[-1]=encoder.FRAME.pack(cid,dlc,payload[:6]+(650).to_bytes(2,'big'))
        return count
    monkeypatch.setattr(Socket,'send',send)
    obs=ro.ReadOnlyObserver(tmp_path)
    with pytest.raises(RuntimeError,match='temperature'):obs.sample()
    obs.close();assert all(frame in ro.ALLOWED for s in fake_can for frame in s.sent)


@pytest.mark.parametrize('fail',[False,True])
def test_telemetry_thread_flushes_audit_and_closes_on_failure(tmp_path,fail):
    import time
    made=[]
    class MockObserver:
        def __init__(self,folder):self.count=0;self.closed=False;made.append(self)
        def sample(self):
            self.count+=1
            if fail and self.count==3:raise RuntimeError('mock feedback loss')
            return samples(end=time.monotonic(),n=1)[0]
        def close(self):self.closed=True
        def report(self):return {'mock':True,'closed':self.closed}
    worker=ro.Telemetry(tmp_path,observer_factory=MockObserver)
    deadline=time.monotonic()+3
    try:
        while worker.count<18 and not worker.error and time.monotonic()<deadline:time.sleep(.01)
    finally:worker.close()
    assert made[0].closed and not worker.thread.is_alive()
    assert len((tmp_path/'encoder_batches.jsonl').read_text().splitlines())==worker.count
    if fail:assert worker.error=='mock feedback loss' and worker.count==2
    else:
        assert worker.error is None and worker.count>=18
        ro.select_stationary(worker.samples())
