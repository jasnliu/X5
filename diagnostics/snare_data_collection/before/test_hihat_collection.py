"""Offline plan checks and real serial-worker integration through a fake PTY."""
import json
import math
import os
from pathlib import Path
import pty
import select
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

from hihat_collection.plan import make_plan,cycle_commands,CLOSE_DEGREES
from camera_playback.hihat import MOTOR2_TARGET_DEGREES
from hihat_collection.control import HiHatSession
from hihat_collection.collect import HiHatCollector
from x5_collection.collect import Collector
from smooth_playback.runner import Runner


class HiHatCollectionTests(unittest.TestCase):
    def test_collection_matches_current_100_degree_swing_target(self):
        self.assertEqual(CLOSE_DEGREES, 100)
        self.assertEqual(CLOSE_DEGREES, MOTOR2_TARGET_DEGREES)

    def test_finite_balanced_plan(self):
        p=make_plan()
        self.assertEqual(len(p['clean']),20)
        self.assertEqual(sum(r['closures'] for r in p['clean']),24)
        self.assertEqual(len(p['mixed']),10)
        self.assertEqual(p['background_count'],10)
        self.assertEqual(len(set(p['ride_negative_ids'])),20)
        for depth in (10,10.5,11,11.5,12):
            self.assertEqual(sum(r['depth_deg']==depth for r in p['mixed']),2)

    def test_commands_preserve_swing_timing(self):
        self.assertEqual(cycle_commands(10.,1),[(10.,b'C'),(10.6,b'O')])
        self.assertEqual(cycle_commands(10.,2),[(10.,b'C'),(10.6,b'O'),(11.2,b'C'),(11.799999999999999,b'O')])
        for n in (0,3,30):
            with self.assertRaises(ValueError):cycle_commands(1.,n)

    def exercise_worker(self,lose_parent=False,async_stop=False):
        master,slave=pty.openpty();port=os.ttyname(slave)
        observed=[];stop=threading.Event()
        def firmware():
            while not stop.is_set():
                if not select.select([master],[],[],.02)[0]:continue
                for command in os.read(master,256):
                    observed.append((time.monotonic(),chr(command)))
                    if command==ord('S'):os.write(master,b'STOP -- all motors released.\r\n')
        thread=threading.Thread(target=firmware);thread.start()
        try:
            with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ,{'PLAYBACK_OFFLINE_ONLY':'0'}):
                session=HiHatSession(port,Path(tmp))
                try:
                    until=time.monotonic()+5.
                    while session.check()['state']!='idle':
                        self.assertLess(time.monotonic(),until);time.sleep(.01)
                    due=time.monotonic()+.15
                    session.block(1,due,2 if not lose_parent else 1)
                    if lose_parent:
                        session.heartbeat.value=time.monotonic()-5
                        session.process.join(4.)
                    else:
                        while session.check()['completed']!=1:
                            self.assertLess(time.monotonic(),due+3.);time.sleep(.01)
                        if async_stop:
                            session.request_stop();session.request_stop()
                            # Simulate the parent spending 2+ seconds on arm
                            # centering without checking the accessory again.
                            session.process.join(3.)
                        else:session.stop()
                    result=json.loads((Path(tmp)/'hihat_result.json').read_text())
                    self.assertTrue(result['release_ack'])
                    if lose_parent:self.assertIn('heartbeat',result['error'])
                    else:
                        self.assertIsNone(result['error'])
                        self.assertEqual(result['closures_commanded'],2)
                        beats=[(t,c) for t,c in observed if c in 'CO']
                        self.assertEqual([c for t,c in beats],['C','O','C','O','O'])
                        self.assertAlmostEqual(beats[1][0]-beats[0][0],.6,delta=.08)
                        self.assertAlmostEqual(beats[2][0]-beats[0][0],1.2,delta=.08)
                    self.assertNotIn('K',[c for _,c in observed])
                    self.assertFalse(session.process.is_alive())
                finally:
                    if session.process.is_alive():session.stop()
        finally:
            stop.set();thread.join();os.close(master);os.close(slave)

    def test_real_worker_finite_cycles_and_normal_shutdown_fake_serial(self):
        self.exercise_worker()

    def test_parent_failure_still_opens_before_releasing_fake_serial(self):
        self.exercise_worker(True)

    def test_async_stop_does_not_need_parent_heartbeat_during_arm_return(self):
        self.exercise_worker(async_stop=True)

    def test_accessory_stop_is_requested_before_arm_recovery(self):
        collector=object.__new__(HiHatCollector);calls=[]
        collector.hihat=SimpleNamespace(request_stop=lambda:calls.append('hihat-stop'))
        with patch.object(Collector,'center_and_relax',side_effect=lambda:calls.append('arm-center')):
            collector.center_and_relax()
        self.assertEqual(calls,['hihat-stop','arm-center'])

    def make_poll_collector(self,recovering=True):
        c=object.__new__(HiHatCollector)
        c.recovering=recovering;c.near_center_recovery=False;c.startup_recovery=False
        c.g=SimpleNamespace(center=np.array([0.,0.,0.,0.,0.,0.,1.4]))
        c.hihat=None
        return c

    def test_return_can_enter_bounded_center_region_during_motion(self):
        c=self.make_poll_collector()
        for j7,expected in ((.7,False),(1.40344,True)):
            q=c.g.center.copy();q[6]=j7;c.positions=lambda:q
            with patch.object(Runner,'poll',return_value=q) as base:
                np.testing.assert_array_equal(c.poll(),q)
                base.assert_called_once_with(True)
            self.assertEqual(c.near_center_recovery,expected)
        # The same small center overshoot must NOT change strike-time checks.
        c=self.make_poll_collector(False);c.positions=lambda:q
        with patch.object(Runner,'poll',return_value=q):c.poll()
        self.assertFalse(c.near_center_recovery)

    def test_near_center_still_rejects_departure_and_feedback_failures(self):
        c=self.make_poll_collector();c.near_center_recovery=True
        for joint,degrees in ((0,.21),(6,1.01)):
            q=c.g.center.copy();q[joint]+=math.radians(degrees);c.positions=lambda:q
            with patch.object(Runner,'poll',return_value=q):
                with self.assertRaisesRegex(RuntimeError,'bounded region'):c.poll()
        with patch.object(Runner,'poll',side_effect=RuntimeError('Fresh feedback unavailable')):
            with self.assertRaisesRegex(RuntimeError,'Fresh feedback'):c.poll()


if __name__=='__main__':unittest.main()
