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

from hihat_collection.plan import make_plan,cycle_commands,CLOSE_DEGREES,COUNTS
from camera_playback.hihat import MOTOR2_TARGET_DEGREES
from hihat_collection.control import HiHatSession
from hihat_collection.collect import HiHatCollector
from x5_collection.collect import Collector
from smooth_playback.runner import Runner


class HiHatCollectionTests(unittest.TestCase):
    def test_collection_angle_is_local_90_not_legacy_100(self):
        self.assertEqual(CLOSE_DEGREES, 90)
        self.assertEqual(MOTOR2_TARGET_DEGREES, 100)

    def test_exact_snare_plan(self):
        from collections import Counter
        p=make_plan()
        self.assertEqual(Counter(r['kind'] for r in p),COUNTS)
        self.assertEqual(sum(r['snare_hits'] for r in p),45)
        self.assertEqual(sum(r['snare_hits']>0 for r in p),40)
        self.assertEqual(sum(r['snare_hits']==2 for r in p),5)
        self.assertEqual(sum(r['split']=='test' for r in p),12)
        self.assertEqual(set(r['snare_degrees'] for r in p if r['snare_hits']),{11.})
        self.assertEqual(set(r['ride_depth_deg'] for r in p if r['ride_hits']),{10.,10.5,11.,11.5,12.})
        self.assertEqual([r['block'] for r in p],list(range(1,61)))
        self.assertEqual(p,make_plan())

    def test_commands_preserve_timing_use_calibrated_edge(self):
        self.assertEqual(cycle_commands(10.,1),[(10.,b'B'),(10.6,b'O')])
        self.assertEqual(cycle_commands(10.,2),[(10.,b'B'),(10.6,b'O'),(11.2,b'B'),(11.799999999999999,b'O')])
        for n in (0,3,30):
            with self.assertRaises(ValueError):cycle_commands(1.,n)

    def exercise_worker(self,lose_parent=False,async_stop=False):
        master,slave=pty.openpty();port=os.ttyname(slave)
        observed=[];stop=threading.Event()
        def firmware():
            buffer=bytearray()
            while not stop.is_set():
                if not select.select([master],[],[],.02)[0]:continue
                for command in os.read(master,256):
                    buffer.append(command)
                    if buffer.endswith(b'A90\n'):
                        os.write(master,b'ANGLE degrees=90 counts=134\r\n')
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
                        beats=[(t,c) for t,c in observed if c in 'BO']
                        self.assertEqual([c for t,c in beats],['B','O','B','O','O'])
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

    def test_accessory_stop_precedes_left_center_and_right_relax(self):
        collector=object.__new__(HiHatCollector);calls=[]
        collector.hihat=SimpleNamespace(request_stop=lambda:calls.append('hihat-stop'))
        collector.hybrid=None;collector.left_ready=True
        collector.bus=SimpleNamespace(cancel_snare=lambda:calls.append('cancel-snare'))
        collector.left_move=lambda *args:calls.append('left-center')
        with patch.object(Collector,'center_and_relax',side_effect=lambda:calls.append('right-center-relax')):
            collector.center_and_relax()
        self.assertEqual(calls,['hihat-stop','cancel-snare','left-center','right-center-relax'])

    def make_poll_collector(self,recovering=True):
        c=object.__new__(HiHatCollector)
        c.recovering=recovering;c.near_center_recovery=False;c.startup_recovery=False
        c.g=SimpleNamespace(center=np.array([0.,0.,0.,0.,0.,0.,1.4]))
        c.hihat=None;c.left_ready=False
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
