"""65-second UI soak: explicitly MOCK encoders and SYNTHETIC depth, NO devices."""
import hashlib,json,os,socket,sys,time
from pathlib import Path
os.environ['QT_QPA_PLATFORM']='offscreen'
ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import alignment_view as av
from drum_planner import hardware_check as hw
from drum_planner import depth
from drum_planner.common import atomic_json
D=Path(__file__).parent
candidates=list(Path('/tmp/pytest-of-jason').glob('pytest-*/test_alignment_gui_is_static_m0/run'))
source=max(candidates,key=lambda p:p.stat().st_mtime)
assert 'pytest-' in str(source)
real=socket.socket;can_attempts=[]
def guarded(family=socket.AF_INET,*a,**kw):
    if family==socket.AF_CAN:
        can_attempts.append(True);raise AssertionError('Offline UI attempted CAN')
    return real(family,*a,**kw)
socket.socket=guarded
def denied(*a,**kw):raise AssertionError('Offline UI attempted camera')
depth.DepthCamera=denied;hw.DepthCamera=denied
folder,cal,state,model,manifest=hw.load_snapshot(source)
packet,mesh=av.load_scene(folder,cal);coverage=json.loads((folder/'coverage.json').read_text())
def fingerprint():
    h=hashlib.sha256()
    for a in (packet.raw_image,packet.raw_points,mesh.vertices,mesh.faces):h.update(a.tobytes())
    return h.hexdigest()
v=av.AlignmentView(cal,D,av.MeasuredPose(folder,state,model),coverage)
v.window.setWindowTitle('OFFLINE UNIT TEST ONLY — mock encoders / synthetic depth')
v.banner.setText('OFFLINE UNIT TEST ONLY — MOCK ENCODERS / SYNTHETIC DEPTH — NO DEVICES')
v.set_scene(packet,mesh);before=fingerprint();start=time.monotonic();actions=set();error=None
try:
    idle_draws=len(v.render_draw_seconds)
    while time.monotonic()-start<65:
        elapsed=time.monotonic()-start
        if elapsed>5 and 'idle' not in actions:
            assert len(v.render_draw_seconds)==idle_draws
            actions.add('idle')
        for sec,name,callback in [(8,'points',lambda:v.mode.setCurrentIndex(1)),
                                  (10,'both',lambda:v.mode.setCurrentIndex(2)),
                                  (12,'surface',lambda:v.mode.setCurrentIndex(0)),
                                  (15,'hide',lambda:v.overlay_box.setChecked(False)),
                                  (17,'show',lambda:v.overlay_box.setChecked(True)),
                                  (20,'height',lambda:v.color.setCurrentIndex(1)),
                                  (22,'reset',v.reset_view),(24,'focus',v.focus_arm),
                                  (26,'play_blocked',v.toggle_play),(28,'save',v.save_view)]:
            if elapsed>=sec and name not in actions:callback();actions.add(name)
        assert not v.clock.playing and v.rendered==1 and v.worker.submissions==0
        assert v.animation_draws==0 and v.packet is packet
        v.poll();time.sleep(.01)
    assert fingerprint()==before and not can_attempts
    v.window.grab().save(str(D/'offline_alignment_window.png'))
except BaseException as exc:error=str(exc);raise
finally:
    report=dict(passed=error is None,error=error,duration_s=time.monotonic()-start,
                MOCK_ENCODERS=True,SYNTHETIC_DEPTH=True,physical_hardware_test=False,
                can_attempts=can_attempts,actions=sorted(actions),visualization=v.report())
    v.close();atomic_json(D/'offline_alignment_soak.json',report)
    print(json.dumps(report,indent=2))
