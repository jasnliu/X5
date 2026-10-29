"""Device-free DT1 selection, paired-scene integrity and serial queue contracts."""
import copy,json
from pathlib import Path
import numpy as np
import pytest
from drum_planner import cli,depth,planning
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,atomic_json,calibration,digest
from drum_planner.experiment import app,dt1_goal,rgbd
from test_planner_experiment import queued_view,destination_for


def test_dt1_right_only_before_any_acquisition(monkeypatch):
    assert dt1_goal.resolve('dt1','right')=='dt1'
    assert dt1_goal.resolve('DT1','right')=='dt1'
    monkeypatch.setattr(app,'prepare',lambda *a,**kw:pytest.fail('Invalid selection reached devices'))
    assert cli.main(['experiment','--left','--destinations','dt1'])==1


@pytest.fixture
def paired_scene(tmp_path):
    """Synthetic numerical fixture only; never cited as actual-camera evidence."""
    cal=calibration(DEFAULT_CALIBRATION);meta=copy.deepcopy(cal['camera'])
    meta.update(protocol=1,alignment='native_depth',frame_sync=True,encoding='mjpeg')
    meta['depth'].update(fps=10,format_enum=8);meta['color'].update(fps=10,format_enum=5)
    raw=np.zeros((800,1280),dtype='<u2');raw[100:180,400:480]=1000
    color=np.zeros((720,1280,3),dtype='u1')
    hs=[dict(sequence=i+1,received=100.+i*.1,scale_m=.001,
        depth_timestamp_us=1000000+i*100000,color_timestamp_us=1010000+i*100000) for i in range(5)]
    frames=rgbd.headers(hs)
    obj=depth.snapshot(tmp_path,meta,[(h,raw.copy()) for h in frames],DEFAULT_CALIBRATION)
    np.savez_compressed(tmp_path/'pair.npz',depth=raw,color=color,meta=json.dumps(meta),
        header=json.dumps(hs[-1]),calibration=json.dumps(cal))
    obj.update(schema=rgbd.SCHEMA,dt1_pair_file='pair.npz',dt1_frame_sequence=5)
    obj['files_sha256']['pair.npz']=digest(tmp_path/'pair.npz')
    atomic_json(tmp_path/'scene.json',obj)
    return tmp_path/'scene.json'


def test_paired_scene_preserves_native_five_frame_geometry_not_hardware_schema(paired_scene):
    obj=rgbd.scene(paired_scene,DEFAULT_CALIBRATION)
    assert obj['voxel_size_m']==.005 and obj['obstacle_padding_m']==.02
    assert len(obj['frames'])==5 and obj['depth_fusion']['minimum_support']==3
    assert obj['source_scene_sha256']==digest(paired_scene)
    with np.load(paired_scene.parent/'raw_depth.npz') as data:
        frames=list(zip(obj['frames'],data['depth']))
    _,_,minimum,free=depth.depth_window(frames)
    np.testing.assert_array_equal(minimum,np.fromfile(paired_scene.parent/'depth_m.bin','<f4').reshape(800,1280))
    points=depth.occupied_voxels(depth.deproject(minimum,obj['camera'],obj['T_base_depth']),.005)
    np.testing.assert_array_equal(points.astype('<f4'),np.fromfile(obj['points_file'],'<f4').reshape(-1,3))
    with pytest.raises(ValueError,match='Real planning scenes'):planning.scene(paired_scene,DEFAULT_CALIBRATION)


@pytest.mark.parametrize('change',['hash','frame','raw','fusion','transform','pair_path','scale'])
def test_paired_scene_rejects_tamper_and_mismatched_rgb_depth(paired_scene,change):
    path=paired_scene;obj=json.loads(path.read_text())
    if change=='hash':(path.parent/'pair.npz').write_bytes(b'corrupt')
    if change=='frame':obj['dt1_frame_sequence']+=1
    if change=='fusion':obj['depth_fusion']['minimum_support']=1
    if change=='transform':obj['T_base_depth'][0][3]+=.1
    if change=='pair_path':obj['dt1_pair_file']='../old.npz'
    if change=='scale':obj['frames'][0]['scale_m']=float('nan')
    if change=='raw':
        with np.load(path.parent/'raw_depth.npz') as data:arrays=data['depth'];scales=data['scales_m']
        arrays[-1,100,400]+=1
        np.savez_compressed(path.parent/'raw_depth.npz',depth=arrays,scales_m=scales)
        obj['files_sha256']['raw_depth.npz']=digest(path.parent/'raw_depth.npz')
    # atomic_json rightly refuses nan: use it to test a corrupt external file.
    path.write_text(json.dumps(obj))
    with pytest.raises(ValueError):rgbd.scene(path,DEFAULT_CALIBRATION)


def test_old_depth_only_dispatch_unchanged(monkeypatch,tmp_path):
    path=tmp_path/'scene.json';atomic_json(path,dict(schema='x5-native-depth-scene-v1'))
    calls=[];monkeypatch.setattr(planning,'scene',lambda *a:calls.append(a) or {'old':True})
    assert rgbd.scene(path,DEFAULT_CALIBRATION)=={'old':True}
    assert calls==[(path,DEFAULT_CALIBRATION)]


def test_dt1_rejects_no_rgb_before_inference(monkeypatch,tmp_path):
    context=dict(moving_side='right',state_sha256='S',scene_sha256='D',state='state',scene='scene',calibration=str(DEFAULT_CALIBRATION))
    monkeypatch.setattr(dt1_goal,'identity',lambda *a:{})
    monkeypatch.setattr(dt1_goal,'digest',lambda p:{'state':'S','scene':'D'}[p])
    from drum_planner.experiment import config
    monkeypatch.setattr(config,'no_can',lambda:None)
    monkeypatch.setattr(rgbd,'scene',lambda *a:{'schema':'x5-native-depth-scene-v1'})
    monkeypatch.setattr(dt1_goal.subprocess,'run',lambda *a,**kw:pytest.fail('No inference or camera for depth-only scene'))
    with pytest.raises(ValueError,match='no paired RGB'):dt1_goal.generate(tmp_path,context,['dt1'])
    report=json.loads((tmp_path/'result.json').read_text())
    assert report['status']=='failed' and report['motor_commands']==0 and not (tmp_path/'dt1.json').exists()


def fake_dt1(v,monkeypatch):
    # Real controller methods, contained-job doubles: no camera, CAN or native IK.
    monkeypatch.setattr(dt1_goal,'identity',lambda c,s:dict(selection=s,scene=c['scene_sha256'],state=c['state_sha256']))
    def start(session,context,selected):
        target,job=app.start_plan(session,'dt1',v.params,v.planning_seconds)
        return target,job
    monkeypatch.setattr(dt1_goal,'start',start)
    result=destination_for(v,'generated_dt1.json')
    monkeypatch.setattr(dt1_goal,'result',lambda *a:([str(result)],{'ready':True}))
    return result


def test_dt1_serial_generation_then_ordinary_path_worker(queued_view,monkeypatch):
    v=queued_view;path=fake_dt1(v,monkeypatch)
    v.destination_box.path='dt1';v.select_destination()
    if v.side=='left':
        assert not v.job and not v.test_jobs;return
    assert v.job_kind=='dt1' and v.destination_path is None and not v.pending_methods
    v.finish()
    assert v.job_kind=='plan' and v.destination_path==Path('dt1')
    assert v.context['destination_source']==str(path) and v.context['dt1_generation']=={'ready':True}
    assert json.loads((v.session/'context.json').read_text())==v.context
    v.cancel_job();v.poll()


def test_dt1_cancel_retry_and_switch_do_not_install_late_goals(queued_view,monkeypatch):
    v=queued_view
    if v.side=='left':return
    fake_dt1(v,monkeypatch);v.destination_box.path='dt1';v.select_destination()
    first=v.job;v.cancel_job();v.poll();assert first.closed and not v.job and v.destination_path is None
    v.generate();assert v.job_kind=='dt1'
    old=v.job;old.cancel=lambda:setattr(old,'cancelled',True)
    saved=destination_for(v,'saved.json');v.destination_box.path=str(saved);v.select_destination()
    assert v.job is old and not v.cache
    v.finish();assert old.closed and v.job_kind=='plan' and 'dt1_generation' not in v.context
    assert v.context['destination_source']==str(saved)
    v.cancel_job();v.poll()


def test_dt1_failure_has_no_stale_route_and_retry_button(queued_view,monkeypatch):
    v=queued_view
    if v.side=='left':return
    fake_dt1(v,monkeypatch);v.destination_box.path='dt1';v.select_destination()
    atomic_json(v.job_target/'result.json',dict(status='failed',error='No valid cymbal in frozen image'))
    v.finish(1)
    assert not v.job and not v.cache and not v.plan.playable and v.destination_path is None
    assert 'No valid cymbal' in v.status.text and 'No old destination' in v.status.text
    assert v.generate_button.enabled
    v.generate();assert v.job_kind=='dt1';v.cancel_job();v.poll()


def test_selection_change_reaps_old_plan_before_dt1_starts(queued_view,monkeypatch):
    v=queued_view
    if v.side=='left':return
    fake_dt1(v,monkeypatch);v.cache_all();old=v.job
    old.cancel=lambda:setattr(old,'cancelled',True)
    v.destination_box.path='dt1';v.select_destination()
    assert v.job is old and v.pending_dt1==['dt1'] and len(v.test_jobs)==1
    v.finish();assert old.closed and v.job_kind=='dt1' and not v.pending_dt1
    v.cancel_job();v.poll()


def test_stale_dt1_result_rejected_on_scene_change(queued_view,monkeypatch):
    v=queued_view
    if v.side=='left':return
    fake_dt1(v,monkeypatch);v.destination_box.path='dt1';v.select_destination()
    v.context['scene_sha256']='new scene';v.finish()
    assert not v.job and not v.plan.playable and not v.cache
    assert 'Selection changed' in v.status.text


def test_result_integrity_rechecks_scene_start_files_and_angle_policy(tmp_path):
    from drum_planner.model import generate
    from cymbal_destination import output,solver
    generate(tmp_path/'model',DEFAULT_CALIBRATION)
    kin=solver.Kinematics(tmp_path/'model/robot.urdf','right')
    q=np.array([-.4,.4,.2,1.,.006,-.022,1.3]);f=kin.fk(q,jac=True)
    normal=f['Jtip'][:,6];normal/=np.linalg.norm(normal)
    inward=f['axis']-normal*(f['axis']@normal);inward/=np.linalg.norm(inward)
    geometry=solver.geometry(kin,q,f['tip'],normal,inward)
    path=tmp_path/'generated.json'
    atomic_json(path,output.destination(q,geometry,f['tip'],normal,DEFAULT_CALIBRATION,'scene',
                                       joint_bounds=(kin.lower,kin.upper)))
    ctx=dict(scene_sha256='scene',state_sha256='start',calibration=str(DEFAULT_CALIBRATION),moving_side='right')
    obj=dict(status='ready',identity=dt1_goal.identity(ctx,['dt1']),
        resolved=[str(path)],resolved_sha256=[digest(path)])
    atomic_json(tmp_path/'result.json',obj)
    assert dt1_goal.result(tmp_path,ctx,['dt1'])[0]==[str(path)]
    assert obj['identity']['maximum_inward_yaw_deg']==25.
    for key in ('scene_sha256','state_sha256'):
        with pytest.raises(ValueError,match='Stale'):dt1_goal.result(tmp_path,dict(ctx,**{key:'new'}),['dt1'])
    obj['resolved_sha256']=[];atomic_json(tmp_path/'result.json',obj)
    with pytest.raises(ValueError,match='Incomplete'):dt1_goal.result(tmp_path,ctx,['dt1'])


def test_paired_capture_failure_keeps_busy_diagnostic_and_no_scene(monkeypatch,tmp_path):
    from drum_planner import camera_diagnostics
    class Child:
        returncode=1
        def poll(self):return self.returncode
    def popen(command,**kw):
        folder=Path(command[command.index('--folder')+1])
        atomic_json(folder/'error.json',dict(error='Camera bridge stopped'))
        (folder/'camera_stderr.log').write_text('uvc_open failed: [Return Code: -6]\n')
        return Child()
    monkeypatch.setattr(rgbd.subprocess,'Popen',popen)
    monkeypatch.setattr(camera_diagnostics,'camera_holders',lambda *a,**kw:['PID 123: /fake/other-camera'])
    folder=tmp_path/'camera'
    with pytest.raises(RuntimeError,match='is busy') as error:rgbd.capture(folder,DEFAULT_CALIBRATION)
    assert 'PID 123' in str(error.value) and 'No other process was stopped' in str(error.value)
    assert (folder/'depth_stderr.log').resolve()==folder/'camera_stderr.log'
    assert not (folder/'scene.json').exists()


def test_paired_capture_timeout_reaps_its_child_without_success(monkeypatch,tmp_path):
    class Child:
        returncode=None;terminated=False;waited=False
        def poll(self):return self.returncode
        def terminate(self):self.terminated=True;self.returncode=-15
        def wait(self,timeout):self.waited=True;return self.returncode
    child=Child();clock=iter(range(0,1000,20))
    monkeypatch.setattr(rgbd.subprocess,'Popen',lambda *a,**kw:child)
    monkeypatch.setattr(rgbd.time,'monotonic',lambda:next(clock))
    monkeypatch.setattr(rgbd.time,'sleep',lambda *a:None)
    with pytest.raises(TimeoutError,match='child timed out'):rgbd.capture(tmp_path/'camera',DEFAULT_CALIBRATION)
    assert child.terminated and child.waited and not (tmp_path/'camera/scene.json').exists()
