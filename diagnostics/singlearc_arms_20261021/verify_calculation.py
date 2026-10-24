"""Full Calculate with recorded depth and virtual encoders, real planning/checks.

Run inside experiment.runtime.Job with allow_can=False. No physical device is
opened. The generated acquisition is test data; revoke hardware eligibility
before exiting, including on failure. UI playback must use --simulate only.
"""
import copy,json,sys,threading,time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from drum_planner.experiment.config import no_can
no_can()
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,JOINTS,LEFT_JOINTS,atomic_json
from drum_planner.baseline import work,tasks
from drum_planner.baseline.feedback import normalized_pose
from drum_planner import current_plan,depth
from safe_zone import encoder

from drum_planner.experiment.config import saved_inputs
from drum_planner.baseline.arms import closed_gripper
SIDE=sys.argv[2]
STATE,SCENE=saved_inputs(SIDE)


def main():
    folder=Path(sys.argv[1]).resolve();folder.mkdir(parents=True,exist_ok=True)
    state=json.loads(STATE.read_text());scene=json.loads(SCENE.read_text())
    # Preserve the saved seven-joint poses; virtual motor-8 values use each
    # custom CLOSED target. No physical encoders, CAN, camera, or commands.
    counts={}
    for side in ('left','right'):
        for i in range(1,9):
            raw=encoder.joint_to_motor(side,i,state[side+'_rad'][i-1]) if i<=7 else -closed_gripper(side)
            counts[side,i]=round((raw+12.57)/25.14*65535)
        state[side+'_rad']=[encoder.encoder_to_joint(side,i,counts[side,i]/65535*25.14-12.57) for i in range(1,8)]
    state['left_gripper_opening_m']=0.
    with np.load(SCENE.parent/'raw_depth.npz') as f:arrays=f['depth'];scales=f['scales_m']
    class Bus:
        def __init__(self,side):self.side=side;self.queue=[]
        def send(self,frame):
            assert frame in work.ReadOnlyObserver.sample.__globals__['ALLOWED']
            motor=encoder.FRAME.unpack(frame)[0]&255
            payload=counts[self.side,motor].to_bytes(2,'big')+b'\x80\x00'*2+b'\x00\xe6'
            self.queue.append(encoder.FRAME.pack(encoder.EFF|(2<<24)|(motor<<8)|0xfd,8,payload));return 16
        def recv(self,n):
            if not self.queue:raise BlockingIOError()
            return self.queue.pop(0)
        def close(self):pass
    def initialize(observer,**kw):
        observer.sockets={s:Bus(s) for s in ('left','right')};observer.locks=[];observer.tx_count=0
    def ready(sockets,*args):
        time.sleep(.0005)
        return [s for s in sockets if s._socket.queue],[],[]
    class Camera:
        def __init__(self,folder,cal):
            self.frames=[];self.error=None;self.process=None;self.meta=copy.deepcopy(scene['camera'])
            depth.check_metadata(self.meta,cal)
            self.stop=threading.Event();self.thread=threading.Thread(target=self.deliver);self.thread.start()
        def deliver(self):
            for i in range(5):
                if self.stop.wait(.1):return
                h=dict(scene['frames'][i],received_monotonic=time.monotonic(),value_scale_m=float(scales[i]))
                self.frames.append((h,arrays[i]))
        def close(self):self.stop.set();self.thread.join(2)
    q=dict(zip(JOINTS+LEFT_JOINTS,state['right_rad']+state['left_rad']))
    q['openarmx_left_finger_joint1']=state['left_gripper_opening_m']
    anchor=normalized_pose(q,time.monotonic());began=time.monotonic()
    try:
        with patch.object(encoder.Observer,'__init__',initialize),patch.object(encoder,'select',SimpleNamespace(select=ready)),patch.object(current_plan,'DepthCamera',Camera):
            work.calculate(folder,DEFAULT_CALIBRATION,anchor,moving_side=SIDE)
        result=json.loads((folder/'result.json').read_text());packet,curve=tasks.check_packet(result['prepared'])
        report=json.loads((folder/'query_report.json').read_text())
        batches=[json.loads(s) for s in (folder/'capture/encoder_batches.jsonl').read_text().splitlines()]
        assert not report['error'] and report['thread_stopped'] and report['observer']['closed']
        assert report['observer']['motion_commands']==0 and report['sample_batches']>100
        assert packet['proof']['all_valid'] and curve.duration>0
        assert (folder/'computation/result.json').exists() and not (folder/'computation/error.json').exists()
        tau=curve.duration*.4
        tasks.check_return(folder/'checked_return',result['prepared'],curve.at(tau),tau,1.)
        returned=json.loads((folder/'checked_return/return.json').read_text())
        assert returned['proof']['all_valid']
        assert ('right_snapshot' if SIDE=='left' else 'left_snapshot') in returned
        # Every query packet passed the real exact-query gate. Every fake reply
        # passed the real decoder, timestamps, disabled/fault checks and guard.
        atomic_json(folder/'verification.json',dict(passed=True,software_only=True,physical_can_opened=False,
            physical_camera_opened=False,physical_motor_commands=0,elapsed_s=time.monotonic()-began,
            moving_side=SIDE,source_state=str(STATE),source_scene=str(SCENE),sample_batches=report['sample_batches'],duration_outward_s=curve.duration,
            maximum_batch_s=max(b['end']-b['start'] for b in batches),
            maximum_batch_start_gap_s=max(b['start']-a['start'] for a,b in zip(batches,batches[1:])),
            collision_zone=packet['proof'],recovery_proof=returned['proof'],plan=result['plan'],process_isolation=True,hardware_eligibility_revoked=True))
        print('FULL SOFTWARE CALCULATE PASSED: real planner, timing, dense checks and uninterrupted query monitor',flush=True)
    finally:
        # Never leave simulated encoders represented as physical authorization.
        for p in [folder/'plan/result/plan.json',folder/'checked/prepared.json']:
            if p.exists():
                obj=json.loads(p.read_text());obj.update(software_test_only=True,baseline_hardware_session=False,
                    actual_current_robot_state_verified=False,motion_authorized=False)
                atomic_json(p,obj)

if __name__=='__main__':main()
