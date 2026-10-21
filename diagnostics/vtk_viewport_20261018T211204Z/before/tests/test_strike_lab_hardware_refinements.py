"""Regression coverage for improvements discovered on the unloaded real arm."""
import math
import unittest
from unittest.mock import patch
from camera_playback.mit_strike import Sample
from strike_lab.config import TARGET
from strike_lab.curves import Catch
from strike_lab.methods import METHODS
from strike_lab.jobs import validate_jobs,run_jobs
from test_strike_lab import engine
from strike_lab.cli import parse, options_for
from strike_lab.config import ROOT


class HardwareRefinementTests(unittest.TestCase):
    def test_calibrated_hardware_defaults_do_not_change_simulation(self):
        sim,config=parse([])
        self.assertEqual(sim.method,'powered');self.assertEqual(config,{})
        real,config=parse(['--hardware','--no-motion-notification'])
        self.assertEqual(real.method,'gravity')
        self.assertEqual(real.config,ROOT/'config/experiment_hardware_tuned.json')
        self.assertEqual(set(config['parameters']),set(METHODS))
        self.assertTrue(all(p['load_torque']>0 for p in config['parameters'].values()))
        options=options_for(real,config)
        self.assertFalse(options['motion_notification'])
        self.assertEqual(len(options['configuration']['sha256']),64)

    def test_fixed_motion_load_does_not_inherit_hold_integrator(self):
        for mid in ('powered','cosine','torque','impedance','optimized'):
            a=METHODS[mid].create(1.4,.2,{'load_torque':.7})
            b=METHODS[mid].create(1.4,1.2,{'load_torque':.7})
            for m in (a,b):m.start(1.,[])
            s=Sample(1.35,-.5,.7,2,1.1)
            self.assertEqual(a.update(s,1.1),b.update(s,1.1),mid)

    def test_baseline_uses_fixed_load_after_readiness_history(self):
        m=METHODS['baseline'].create(1.4,1.2,{'load_torque':.7})
        history=[Sample(1.4,0,.7,2,i*.01) for i in range(7)]
        m.start(.06,history)
        self.assertEqual(m.c.bias,.7)

    def test_sampled_catch_hits_exact_reference_and_preserves_join(self):
        for shape in (4.,8.,12.,16.):
            c=Catch.to_target(.14,1.6,40.,.22,shape)
            self.assertAlmostEqual(c.low,TARGET,delta=1e-7)
            before=c.at(c.catch_time-1e-7);after=c.at(c.catch_time+1e-7)
            for i in range(3):self.assertAlmostEqual(before[i],after[i],delta=.001)
            for i in range(1001):
                x,v,a=c.at(c.duration*i/1000)
                self.assertTrue(-1e-8<=x<=TARGET+1e-8)

    def test_curve_preflight_cached_only_for_identical_parameters(self):
        from strike_lab.engine import validate_curve
        e,_=engine()
        with patch('strike_lab.engine.validate_curve',wraps=validate_curve) as validate:
            e.trial('powered');e.trial('powered');e.trial('powered',{'down_time':.24})
            self.assertEqual(validate.call_count,2)

    def test_finite_jobs_execute_and_preserve_schedule(self):
        e,s=engine()
        jobs=[dict(method='powered',repetitions=2,intervals=[.6,.8],stage='endurance')]
        run_jobs(e,jobs)
        self.assertEqual(len(s.trials),2)
        self.assertTrue(all(t['metadata']['schedule'] for t in s.trials))
        self.assertTrue(all(t['score']['passed'] for t in s.trials))

    def test_jobs_reject_bad_budgets_and_unknown_parameters(self):
        for j in ([],[dict(method='powered',repetitions=-1)],
                  [dict(method='powered',parameters={'target':9.1})],
                  [dict(method='powered',intervals=[math.nan])]):
            with self.assertRaises(ValueError):validate_jobs(j)


if __name__=='__main__':unittest.main()
