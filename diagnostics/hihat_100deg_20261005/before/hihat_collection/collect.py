"""Finite hi-hat capture; reuse record3/center safety only for ten mixed examples."""
import argparse
from datetime import datetime,timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import time
import uuid

from camera_playback.hihat import DEFAULT_ESP_PORT
from camera_playback.recording_only import refresh_feedback
from x5_collection.collect import Collector,preflight,dump
from x5_collection.hybrid import CollectionSession
from .control import HiHatSession
from .plan import make_plan,CLOSURE_INTERVAL_SECONDS

ROOT=Path(__file__).resolve().parents[1]


def notify(directory,stage,text):
    spec=importlib.util.spec_from_file_location('hihat_ntfy',Path.home()/'.local/bin/codex-ntfy.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    delivered=module.post_ntfy(text,'X5 hi-hat collection: motion warning','warning,robot')
    dump(directory/f'notification_{stage}.json',dict(delivered=bool(delivered),message=text,t=time.monotonic()))
    if not delivered:raise RuntimeError('ntfy delivery failed; no motion allowed')


class HiHatCollector(Collector):
    def __init__(self,prepared,directory,plan):
        super().__init__(prepared,directory,plan['mixed'])
        self.collection_plan=plan;self.hihat=None;self.examples=[]

    def notify_motion(self):
        notify(self.directory,'hihat','Hi-hat data collection starts in 10 seconds. Normal 110-degree close / open cycles only; arm remains disabled initially. A separate warning precedes the arm stage. Keep clear.')

    def check_audio(self):
        super().check_audio()
        if self.hihat is not None and not self.recovering:self.hihat.check()

    def poll(self,check_running=True):
        if self.recovering and not self.near_center_recovery:
            # Recognize the bounded center neighborhood DURING the return,
            # not only at its starting pose. Keep the existing measured-center
            # relax gate; a small J7 settling overshoot is not a midair disable.
            q=self.positions()
            if (max(abs(q[i]-self.g.center[i]) for i in range(6))<=math.radians(.2)
                    and abs(q[6]-self.g.center[6])<=math.radians(1.)):
                self.near_center_recovery=True
        q=super().poll(check_running)
        if self.hihat is not None and not self.recovering:self.hihat.check()
        return q

    def wait_until(self,deadline,arm=False):
        while time.monotonic()<deadline:
            self.cancellation();self.check_audio()
            if arm:self.hybrid_tick()
            time.sleep(.003)

    def save_example(self,row):
        self.examples.append(row)
        dump(self.directory/'examples.json',self.examples)
        self.event('CAPTURED HIHAT EXAMPLE',**row)

    def background(self,first,last):
        for i in range(first,last+1):
            begin=time.monotonic();self.wait_until(begin+6.)
            self.save_example(dict(block=30+i,kind='background',closures=0,start=begin,end=time.monotonic()))

    def before_arm(self):
        # New serial session only after mandatory notification; no firmware changes.
        self.hihat=HiHatSession(DEFAULT_ESP_PORT,self.directory)
        deadline=time.monotonic()+5.
        while self.hihat.check()['state']!='idle':
            self.check_audio();self.cancellation()
            if time.monotonic()>deadline:raise RuntimeError('Hi-hat readiness timeout')
            time.sleep(.01)
        self.background(1,5)
        for row in self.collection_plan['clean']:
            begin=time.monotonic();closure_at=begin+2.
            self.hihat.block(row['block'],closure_at,row['closures'])
            self.wait_until(closure_at+(row['closures']-1)*CLOSURE_INTERVAL_SECONDS+4.)
            if self.hihat.check()['completed']!=row['block']:raise RuntimeError('Hi-hat block incomplete')
            self.save_example(dict(row,start=begin,end=time.monotonic(),close_due=closure_at))
        self.background(6,10)
        notify(self.directory,'arm','Arm moves in 10 seconds: center + close gripper, record3, ten hybrid ride hits at J7 10-12 degrees mixed with normal hi-hat closures, then verified CENTER and relax. Keep clear.')
        self.wait_until(time.monotonic()+10.)

    def collect_blocks(self):
        self.bus.mit_center_return=None
        self.hybrid=CollectionSession(self.bus,self.trajectory.last[6],self.lower_j7,self.upper_j7,self.tuning,self.directory)
        refresh_feedback(self.bus,.1)
        self.wait_hybrid(lambda s:s.ready)
        for row in self.collection_plan['mixed']:
            begin=time.monotonic();close_at=begin+2.
            self.hihat.block(row['block'],close_at,1)
            self.wait_until(close_at+row['ride_command_offset_s'],arm=True)
            requested_at=time.monotonic()
            request=self.hybrid.block(row['depth_deg'],1,row['block'])
            deadline=time.monotonic()+4.
            while True:
                self.check_audio();s=self.hybrid_tick()
                if s.request_id>=request and s.ready and not s.swing and s.count==1:break
                if time.monotonic()>deadline:raise RuntimeError('Mixed ride block timeout')
                time.sleep(.002)
            self.wait_until(close_at+4.,arm=True)
            if self.hihat.check()['completed']!=row['block']:raise RuntimeError('Mixed hi-hat incomplete')
            item=dict(row,start=begin,end=time.monotonic(),close_due=close_at,
                      requested_at=requested_at,last_release=s.released_at,last_return=s.returned_at,
                      last_peak_deg=math.degrees(s.peak_drop))
            self.blocks.append(item);dump(self.directory/'blocks.json',self.blocks)
            self.save_example(item)

    def after_arm(self):
        if self.hihat is not None:
            self.hihat.stop()
            result=json.loads((self.directory/'hihat_result.json').read_text())
            if result['error'] or not result['release_ack']:raise RuntimeError(str(result))
            self.event('HIHAT OPEN RETURN AND RELEASE ACK',**result)

    def center_and_relax(self):
        # Accessory returns independently while the parent services arm
        # feedback. Do not leave it awaiting a heartbeat throughout recovery.
        if self.hihat is not None:self.hihat.request_stop()
        super().center_and_relax()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true',help='Offline preparation only; no devices opened')
    args=parser.parse_args()
    plan=make_plan();prepared=preflight()
    if args.check:
        print(json.dumps(dict(plan=plan,recording=str(prepared[1].source),arm_corridor_deg=12,
                             hihat_close_deg=110,training=False),indent=2));return 0
    if os.environ.get('PLAYBACK_OFFLINE_ONLY')=='1' or os.environ.get('STRIKE_LAB_OFFLINE_ONLY')=='1':
        raise RuntimeError('Hardware forbidden by offline environment')
    directory=ROOT/'diagnostics/hihat_collection'/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True)
    dump(directory/'plan.json',dict(plan=plan,recording=str(prepared[1].source),
        record3_sha256=hashlib.sha256(prepared[1].source.read_bytes()).hexdigest(),training=False,camera=False))
    print('Session:',directory,flush=True)
    result=HiHatCollector(prepared,directory,plan).run_collection()
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
