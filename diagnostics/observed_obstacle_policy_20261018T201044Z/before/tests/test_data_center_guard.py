import math
from collections import deque
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from hihat_collection.transport import settled_center,DataMotors
from camera_playback.left_hold import LEFT_CENTER
from centering.motors import packet

class CenterGuardTests(unittest.TestCase):
    def fixture(self):
        now=10.
        states={('left',i):(float(LEFT_CENTER[i-1]) if i<8 else 0.,2,now) for i in range(1,9)}
        history=deque([(9.35,LEFT_CENTER.copy()),(10.,LEFT_CENTER.copy())])
        return states,history,now

    def test_accepts_only_fresh_stationary_center(self):
        states,history,now=self.fixture()
        self.assertEqual(settled_center(states,history,LEFT_CENTER,'left',now)['max_error_deg'],0)
        for failure in ('offcenter','stale','short','moving','stopped'):
            s,h,n=self.fixture()
            if failure=='offcenter':s['left',1]=(math.radians(1),2,n)
            if failure=='stale':s['left',1]=(0.,2,n-.31)
            if failure=='short':h.popleft()
            if failure=='moving':h[0][1][0]=math.radians(.15)
            if failure=='stopped':s['left',6]=(0.,0,n)
            with self.subTest(failure=failure),self.assertRaises(RuntimeError):
                settled_center(s,h,LEFT_CENTER,'left',n)

    def test_offcenter_left_disable_blocked_at_frame_level(self):
        b=DataMotors.__new__(DataMotors);b.states,b.left_history,now=self.fixture();b.left_disabled_at=None
        b.states['left',1]=(math.radians(15),2,now)
        with patch('hihat_collection.transport.time.monotonic',return_value=now):
            with self.assertRaisesRegex(RuntimeError,'not settled'):b.guard_left_disable(packet(4,6))

    def test_snare_depth_is_fixed_in_implementation(self):
        import inspect
        source=inspect.getsource(DataMotors.start_snare)
        self.assertIn('SNARE_DEGREES',source)
        self.assertNotIn('_prepare(',source)
        self.assertNotIn('packet(4',source)

    def test_setup_holds_live_right_pose_before_synchronized_center(self):
        import inspect
        from hihat_collection.collect import HiHatCollector
        source=inspect.getsource(HiHatCollector.run_collection)
        self.assertIn('self.bus.center(right_live,RIGHT_GRIPPER_CLOSED',source)
        self.assertLess(source.index('self.bus.center(right_live'),source.index("self.move(self.g.center"))

    def test_original_ride_session_sets_owner_key_for_safe_stop(self):
        import inspect
        from x5_collection.hybrid import CollectionSession
        self.assertIn("self.session_attr = 'right_joint7_session'",inspect.getsource(CollectionSession.__init__))

if __name__=='__main__':unittest.main()
