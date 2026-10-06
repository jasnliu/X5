"""Record3 setup and adjustable goals. CAN access is forbidden, including mocks."""
from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np
from camera_playback.mit_strike import Command, Sample
from centering.motors import RIGHT_GRIPPER_CLOSED, SPEED, RIGHT_PLAYBACK_SPEED
from strike_lab.backends import HardwareBackend, SimBackend
from strike_lab.cli import parse, options_for, request_for
from strike_lab.config import ROOT, RECORDING, Limits, Rules, StrikeGoal, center_pose
from strike_lab.engine import Engine, validate_curve
from strike_lab.methods import METHODS
from strike_lab.preparation import load_record3
from strike_lab.scoring import score, summarize
from strike_lab.storage import Store, load_trials, rescore_session
from test_strike_lab import FakeBus, FakePlayback, MemoryStore, engine


class ClockStop:
    def __init__(self):self.time=1.;self.stopped=False
    def is_set(self):return self.stopped
    def wait(self,seconds):self.time+=seconds;return self.stopped


class TracedBus(FakeBus):
    def __init__(self):
        super().__init__([]);self.goals=[];self.fingers=[]
    def center(self,target,gripper_target,confirm_enabled=False):
        self.fingers.append(gripper_target)
        yield from super().center(target,gripper_target,confirm_enabled)
    def set_positions(self,target):
        self.goals.append(list(map(float,target)))
        super().set_positions(target)


def fake_hardware(playback=None):
    bus=TracedBus();stop=ClockStop();channel=Mock()
    channel.sample=Sample(.6,0,.16,2,stop.time);channel.temperature=25.
    with patch.dict(os.environ,{'STRIKE_LAB_OFFLINE_ONLY':'0'}):
        backend=HardwareBackend(Limits(),notifier=None,bus_factory=lambda *_:bus,
                                channel_factory=lambda _:channel,playback=playback or FakePlayback())
    backend.now=lambda:stop.time
    return backend,bus,stop


class PreparationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=RECORDING.read_bytes()
        cls.playback=load_record3(threading.Event(),lambda _:None)

    def test_real_recording_load_is_read_only_and_preserves_exact_endpoint(self):
        raw=json.loads(self.original)
        self.assertEqual(RECORDING.read_bytes(),self.original)
        self.assertEqual(self.playback.sample_count,192)
        np.testing.assert_allclose(self.playback.last_joints,
                                  [-.60553124,.21194553,.24416892,1.73296635,.20476169,-.3051632,.60016068],atol=1e-8)
        np.testing.assert_array_equal(self.playback.joints_at(self.playback.duration_s)[0],self.playback.last_joints)
        self.assertGreater(self.playback.duration_s,self.playback.original_duration_s)

    def test_setup_order_fixed_gripper_then_recording_then_mit(self):
        # Include explicit "open" recording finger data: it must never be sent.
        playback=FakePlayback();playback.gripper_openings=[0.,1.,0.,1.]
        b,bus,stop=fake_hardware(playback);phases=[]
        def arm(helper,ch):
            self.assertEqual(bus.goals[-1],playback.last_joints)
            self.assertEqual(bus.states['right',8][0],RIGHT_GRIPPER_CLOSED)
            helper.controller.bias=.16;bus.events.append('MIT')
        with patch('camera_playback.mit_strike.Joint7Worker._prepare',arm):
            anchor,_=b.prepare(stop,lambda m:phases.append(m['phase']))
        self.assertEqual(bus.fingers,[RIGHT_GRIPPER_CLOSED])
        self.assertEqual(bus.goals[0],center_pose())
        self.assertEqual(bus.goals[1],playback.first_joints)
        self.assertTrue(all(len(q)==7 for q in bus.goals))
        self.assertEqual(bus.events[-3:],[ 'center arm',('speed',SPEED),'MIT'])
        self.assertIn(('speed',RIGHT_PLAYBACK_SPEED),bus.events)
        self.assertIn('HOLDING RECORD3 END',phases)
        self.assertEqual(anchor,playback.last_joints[6])
        original=list(bus.goals)
        b.send(Command(anchor-math.radians(3),0,40,1.8,.16))
        self.assertEqual(bus.goals,original)
        self.assertTrue(all(bus.states['left',i][1]==0 for i in range(1,9)))
        b.close()

    def test_stop_during_playback_never_hands_off_to_mit(self):
        b,bus,stop=fake_hardware()
        def publish(m):
            if m['phase'].startswith('PLAYING RECORD3'):stop.stopped=True
        with patch('camera_playback.mit_strike.Joint7Worker._prepare') as arm:
            with self.assertRaisesRegex(InterruptedError,'playback'):b.prepare(stop,publish)
            arm.assert_not_called()
        b.close();self.assertFalse(bus.active)

    def test_preflight_failure_before_notification_or_bus(self):
        with patch.dict(os.environ,{'STRIKE_LAB_OFFLINE_ONLY':'0'}):
            ctor=Mock();notify=Mock();b=HardwareBackend(Limits(),notifier=notify,bus_factory=ctor)
        with patch('strike_lab.preparation.load_record3',side_effect=ValueError('bad recording')):
            with self.assertRaisesRegex(ValueError,'bad recording'):b.prepare(ClockStop(),lambda _:None)
        ctor.assert_not_called();notify.assert_not_called()

    def test_opening_gripper_during_setup_is_not_ignored(self):
        b,bus,stop=fake_hardware()
        def publish(m):
            if m['phase']=='CENTERING RIGHT ARM':bus.states['right',8]=(0.,2,0.)
        with self.assertRaisesRegex(RuntimeError,'Gripper did not remain closed'):b.prepare(stop,publish)
        b.close()

    def test_simulation_replays_real_record3_and_all_methods_hold_same_other_joints(self):
        status=[];backend=SimBackend(playback=self.playback)
        store=MemoryStore();e=Engine(backend,store,threading.Event(),status.append)
        self.assertEqual(e.anchor,float(self.playback.last_joints[6]))
        self.assertTrue(any('RECORD3 PLAYBACK' in m['phase'] for m in status))
        self.assertTrue(all(m['gripper_deg']==7. for m in status if 'gripper_deg' in m))
        self.assertFalse(any('load stick' in m['phase'].lower() for m in status))
        held=backend.positions[:6]
        for name in METHODS:
            e.trial(name)
            self.assertEqual(backend.positions[:6],held)
            self.assertEqual(e.anchor,float(self.playback.last_joints[6]))
        backend.close()


class AdjustableGoalTests(unittest.TestCase):
    def test_goals_validate_limits_and_joint_range(self):
        for bad in (True,'3',0,.49,-1,math.nan,math.inf,31):
            with self.assertRaises(ValueError):StrikeGoal(bad)
        with self.assertRaisesRegex(ValueError,'motion limit'):StrikeGoal(12).validate(Rules(),Limits())
        with self.assertRaisesRegex(ValueError,'joint limit'):StrikeGoal(3).validate(Rules(),Limits(),-1.39)
        StrikeGoal(3).validate(Rules(),Limits(),.6)
        self.assertAlmostEqual(StrikeGoal(1).zone_deg,.9)
        self.assertAlmostEqual(StrikeGoal(1).acceptance(Rules()),.05)
        self.assertAlmostEqual(StrikeGoal(10).acceptance(Rules()),.2)

    def test_all_methods_use_selected_goal_not_ten_degrees(self):
        for name in METHODS:
            e,store=engine();e.set_goal(3.)
            with self.subTest(method=name):
                r=e.trial(name)['score']
                self.assertEqual(r['target_deg'],3.)
                self.assertAlmostEqual(r['zone_deg'],2.7)
                self.assertAlmostEqual(r['peak_depth_deg'],3.,delta=.25)
                self.assertEqual(store.trials[-1]['metadata']['target_deg'],3.)
                self.assertLess(abs(store.trace[-1]['depth_deg']),.15)

    def test_powered_curve_peak_exact_at_selected_goal(self):
        for degrees in (.5,1.,3.,7.,11.):
            for name in ('powered','cosine','torque','impedance','optimized'):
                m=METHODS[name].create(.6,.16,target=math.radians(degrees))
                validate_curve(m,Limits())
                bottom=m.curve.down if hasattr(m.curve,'down') else m.curve.duration/2
                self.assertAlmostEqual(m.curve.at(bottom)[0],math.radians(degrees))

    def test_changed_goal_reuses_anchor_not_preparation_or_cached_reference(self):
        e,s=engine();anchor=e.anchor
        with patch.object(e.backend,'prepare',side_effect=AssertionError('must not replay')):
            with patch('strike_lab.engine.validate_curve',wraps=validate_curve) as validate:
                e.trial('powered');e.set_goal(3.);e.trial('powered');e.trial('powered')
                self.assertEqual(validate.call_count,2)
        self.assertEqual(e.anchor,anchor)
        self.assertEqual([t['metadata']['target_deg'] for t in s.trials],[10.,3.,3.])
        self.assertEqual(len(summarize(s.trials)['groups']),2)
        with self.assertRaises(ValueError):e.set_goal(50)
        self.assertEqual(e.goal.degrees,3.)

    def test_cli_goal_passes_through_all_request_kinds(self):
        for flags in ([],['--campaign']):
            a,c=parse(['--degrees','3',*flags])
            self.assertEqual(options_for(a,c)['degrees'],3.)
            self.assertEqual(request_for(a,c)['degrees'],3.)
        for value in ('nan','inf','0','-1','12'):
            with self.assertRaises(SystemExit):parse(['--degrees',value])

    def test_archive_and_rescore_keep_per_trial_goal_and_source_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=Store(tmp,'simulation',Rules(),Limits(),goal=StrikeGoal(3.))
            e=Engine(SimBackend(),store,threading.Event(),goal=StrikeGoal(3.))
            trial=e.trial('powered');e.set_goal(7);e.trial('powered')
            store.begin('powered',METHODS['powered'].defaults,'manual',1.4,.16,goal=StrikeGoal(2.))
            report=store.report();store.close()
            manifest=json.loads((store.path/'session.json').read_text())['source_manifest']
            self.assertEqual(manifest['recordings/record3.json'],hashlib.sha256(RECORDING.read_bytes()).hexdigest())
            self.assertEqual((store.path/'source/recordings/record3.json').read_bytes(),RECORDING.read_bytes())
            trials=load_trials(store.path)
            self.assertEqual([t['score']['target_deg'] for t in trials],[3.,7.,2.])
            self.assertIn('| Goal ° | Zone >° |',report.with_suffix('.md').read_text())
            revised=rescore_session(store.path,Rules(),Limits())
            rescored=json.loads((revised/(trial['id']+'.json')).read_text())
            for key in ('target_deg','zone_deg','lower_zone_ms','passed','peak_depth_deg'):
                self.assertEqual(rescored[key],trial['score'][key])
            groups=json.loads((revised/'summary.json').read_text())['groups']
            self.assertEqual({g['target_deg'] for g in groups},{3.,7.,2.})

    def test_worker_rejects_bad_goal_then_runs_new_goal_without_repreparing(self):
        import time
        from strike_lab.runtime import Session
        with tempfile.TemporaryDirectory() as tmp:
            session=Session(dict(output=tmp,hardware=False,realtime=False))
            session.request('prepare');phase='prepare';messages=[]
            deadline=time.monotonic()+20
            try:
                while session.process.is_alive() and time.monotonic()<deadline:
                    for m in session.poll():
                        messages.append(m)
                        if m['phase']=='READY' and phase=='prepare':
                            session.request('run',methods=['powered'],degrees=12.);phase='reject'
                        elif m['phase']=='REQUEST REJECTED' and phase=='reject':
                            session.request('run',methods=['powered'],degrees=3.);phase='run'
                        elif m['phase']=='BATCH COMPLETE':
                            session.request('finish');phase='finish'
                    time.sleep(.01)
                messages+=session.poll()
                self.assertEqual(phase,'finish')
                self.assertFalse(any(m['phase']=='FAULT' for m in messages),messages)
                self.assertFalse(session.process.is_alive())
            finally:session.close()
            scores=list(Path(tmp).glob('*/*/score.json'));self.assertEqual(len(scores),1)
            self.assertEqual(json.loads(scores[0].read_text())['target_deg'],3.)
            events=[json.loads(line) for line in next(Path(tmp).glob('*/events.jsonl')).read_text().splitlines()]
            self.assertEqual(sum(e['kind']=='prepared_reference' for e in events),1)

    def test_legacy_rescore_defaults_to_ten_when_metadata_has_no_goal(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=Store(tmp,'simulation',Rules(),Limits());e=Engine(SimBackend(),store,threading.Event())
            trial=e.trial('powered');store.close()
            p=store.path/trial['id']/'metadata.json';data=json.loads(p.read_text())
            del data['target_deg'];del data['zone_deg'];p.write_text(json.dumps(data))
            revised=rescore_session(store.path,Rules(),Limits())
            self.assertEqual(json.loads((revised/(trial['id']+'.json')).read_text())['target_deg'],10.)


if __name__=='__main__':unittest.main()
