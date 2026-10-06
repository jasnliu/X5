"""Offline contract checks; an audit hook forbids physical CAN socket creation."""
import json
import math
from pathlib import Path
import socket
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from .trajectory import build,transition,SOURCE
from . import runner
from .hardware import AuditedMotors
from centering.motors import packet


def audit(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.PF_CAN:
        raise RuntimeError('Test attempted physical CAN creation')
sys.addaudithook(audit)


class Clock:
    def __init__(self):self.t=100.
    def monotonic(self):return self.t
    def sleep(self,n):self.t+=max(.0001,n)


class FakeBus:
    def __init__(self,path,gripper):
        self.states={(s,i):(0.,0,100.) for s in ('left','right') for i in range(1,9)}
        self.active=False;self.center_permission=False;self.gripper=gripper;self.relax_count=0;self.callback=None
        self.gains_original={};self.gains_restored=True
    def fresh(self):return True
    def poll(self):pass
    def center(self,*a,**kw):
        self.active=True
        for i in range(1,9 if self.gripper else 8):self.states['right',i]=(0.,2,100.)
        if self.gripper:self.states['right',8]=(runner.RIGHT_GRIPPER_CLOSED,2,100.)
        yield
    def set_positions(self,q):
        for i,v in enumerate(q,1):self.states['right',i]=(float(v),2,100.)
        if self.callback:self.callback()
    def set_right_arm_speed(self,s):pass
    def apply_position_gain_factor(self,factor):self.gains_original={1:80.};self.gains_restored=False
    def restore_position_gains(self):self.gains_restored=True
    def relax(self):
        assert self.center_permission
        self.relax_count+=1;self.active=False
        for i in range(1,9):self.states['right',i]=(self.states['right',i][0],0,100.)
    def close(self):pass


class TrajectoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.g,cls.methods=build()
    def test_exact_endpoints(self):
        samples=json.loads(SOURCE.read_text())['samples']
        for t in self.methods.values():
            np.testing.assert_array_equal(t.at(0),samples[0]['positions_rad'])
            np.testing.assert_array_equal(t.at(t.duration),samples[-1]['positions_rad'])
    def test_limits_corridor_and_dynamics(self):
        for name,t in self.methods.items():
            self.assertLessEqual(t.metadata['max_joint_deviation_deg'],1.5)
            self.assertLessEqual(t.metadata['max_tcp_deviation_mm'],10)
            if name in ('smooth','retimed'):
                self.assertLessEqual(t.metadata['peak_velocity_rad_s'],.55)
                self.assertLessEqual(t.metadata['peak_acceleration_rad_s2'],.8)
                self.assertLessEqual(t.metadata['peak_piecewise_jerk_rad_s3'],6)
    def test_smooth_boundary_derivatives(self):
        t=self.methods['smooth']
        for p in t.curves:
            for at in (0,t.source_duration):
                self.assertAlmostEqual(float(p.derivative(1)(at)),0,places=8)
                self.assertAlmostEqual(float(p.derivative(2)(at)),0,places=8)
    def test_matched_timing(self):
        self.assertEqual(self.methods['matched'].duration,self.methods['smooth'].duration)
        self.assertEqual(self.methods['matched'].metadata['command_hz'],100)
    def test_paced_duration_and_dynamics(self):
        original=json.loads(SOURCE.read_text())['duration_s']
        for name in ('paced','paced_200hz','paced_soft','paced_soft200'):
            t=self.methods[name]
            self.assertLessEqual(t.duration,original*1.25)
            for key,limit in [('peak_velocity_rad_s',.72),('peak_acceleration_rad_s2',1.5),('peak_piecewise_jerk_rad_s3',40.)]:
                self.assertLessEqual(t.metadata[key],limit)
            for p in t.curves:
                for at in (0,t.source_duration):
                    self.assertAlmostEqual(float(p.derivative(1)(at)),0,places=7)
                    self.assertAlmostEqual(float(p.derivative(2)(at)),0,places=7)
    def test_legacy_baseline_exact(self):
        from camera_playback.trajectory import load_playback_trajectory
        g=self.g
        old=load_playback_trajectory(SOURCE,g.model,g.zone,g.lower,g.upper,g.center,.4,playback_speed=.8)
        for at in np.linspace(0,old.duration_s,501):
            np.testing.assert_allclose(self.methods['baseline'].at(at),old.joints_at(at)[0],atol=1e-13)
    def test_transition_endpoint(self):
        a=np.zeros(7);b=np.arange(7)/10
        d,at=transition(a,b)
        np.testing.assert_allclose(at(0),a);np.testing.assert_allclose(at(d),b)
        self.assertGreater(d,1)


class WorkflowTests(unittest.TestCase):
    def make(self,path):
        g=SimpleNamespace(center=np.zeros(7),check=lambda *a,**k:None,check_line=lambda *a,**k:None,
                          check_relaxed_start=lambda q:np.array(q))
        t=SimpleNamespace(name='fake',first=np.zeros(7),last=np.ones(7)*.05,duration=1.,
                          metadata={'command_hz':100,'firmware_speed_rad_s':.8},at=lambda s:np.ones(7)*.05*min(s,1.))
        return runner.Runner(g,t,Path(path),video=False)
    def test_complete_sequence(self):
        with tempfile.TemporaryDirectory() as d,patch.object(runner,'time',Clock()),patch.object(runner,'notify_motion'),patch.object(runner,'AuditedMotors',FakeBus):
            r=self.make(d);s=r.run()
            self.assertTrue(s['success']);self.assertTrue(s['relaxed_verified'])
            phases=[e['phase'] for e in r.events]
            for a,b in [('CENTER','PLAYBACK'),('PLAYBACK','ENDPOINT VERIFIED'),('ENDPOINT VERIFIED','RECENTER'),('RECENTER SETTLED','RELAX')]:
                self.assertLess(phases.index(a),phases.index(b))
            self.assertEqual(r.bus.relax_count,1)
    def test_notification_failure_blocks_bus(self):
        with tempfile.TemporaryDirectory() as d,patch.object(runner,'time',Clock()),patch.object(runner,'notify_motion',side_effect=RuntimeError('delivery failed')),patch.object(runner,'AuditedMotors') as bus:
            s=self.make(d).run();self.assertFalse(s['success']);bus.assert_not_called()
    def test_cancel_returns_center(self):
        with tempfile.TemporaryDirectory() as d,patch.object(runner,'time',Clock()),patch.object(runner,'notify_motion'):
            r=self.make(d)
            def bus_factory(*a):
                b=FakeBus(*a)
                def stop():
                    if r.phase=='PLAYBACK':r.cancel=True
                b.callback=stop;return b
            with patch.object(runner,'AuditedMotors',bus_factory):s=r.run()
            self.assertFalse(s['success']);self.assertTrue(s['relaxed_verified'])
            self.assertEqual(r.bus.relax_count,1)
    def test_failed_center_does_not_relax(self):
        with tempfile.TemporaryDirectory() as d,patch.object(runner,'time',Clock()),patch.object(runner,'notify_motion'),patch.object(runner,'AuditedMotors',FakeBus):
            r=self.make(d)
            def fail_move(*a,**k):raise RuntimeError('blocked center')
            r.move=fail_move;s=r.run()
            self.assertFalse(s['relaxed_verified']);self.assertEqual(r.bus.relax_count,0)
    def test_transport_refuses_disable_away_from_center(self):
        b=object.__new__(AuditedMotors);b.center_permission=False;b.states={('right',1):(0.,2,0.)}
        with self.assertRaisesRegex(RuntimeError,'REFUSED disable'):b._send('right',packet(4,1))
    def test_relax_requires_permission(self):
        b=object.__new__(AuditedMotors);b.center_permission=False
        with self.assertRaisesRegex(RuntimeError,'Center verification'):b.relax()
    def test_relax_refuses_unrestored_gains(self):
        b=object.__new__(AuditedMotors);b.center_permission=True;b.gains_restored=False
        with self.assertRaisesRegex(RuntimeError,'gains must be verified'):b.relax()
    def test_gain_increase_refused(self):
        from centering.motors import parameter
        b=object.__new__(AuditedMotors);b.gains_original={1:80.}
        with self.assertRaisesRegex(RuntimeError,'envelope'):b._send('right',parameter(1,0x701e,81.))
    def test_original_gains_restored_at_center(self):
        with tempfile.TemporaryDirectory() as d,patch.object(runner,'time',Clock()),patch.object(runner,'notify_motion'),patch.object(runner,'AuditedMotors',FakeBus):
            r=self.make(d);r.trajectory.metadata['position_gain_factor']=.5;s=r.run()
            self.assertTrue(s['success']);self.assertTrue(s['original_gains_restored'])
            phases=[e['phase'] for e in r.events]
            self.assertLess(phases.index('RECENTER SETTLED'),phases.index('RESTORING ORIGINAL GAINS'))
            self.assertLess(phases.index('RESTORING ORIGINAL GAINS'),phases.index('RELAX'))

if __name__=='__main__':unittest.main()
