"""Offline-only experiment tests; actual AF_CAN construction is forbidden."""
from dataclasses import asdict, replace
import csv
import json
import math
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch


def forbid_physical_can(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:
        raise AssertionError('Real CAN sockets are forbidden in software tests')


sys.addaudithook(forbid_physical_can)

from camera_playback.mit_strike import Command, Sample
from strike_lab.backends import SimBackend, HardwareBackend
from strike_lab.campaign import Plan, neighbors, choose, run_campaign
from strike_lab.cli import parse, request_for
from strike_lab.config import Rules, Limits, Plant, TARGET, StrikeGoal, center_pose
from strike_lab.curves import Dip, CosineDip, Quintic
from strike_lab.engine import Engine, StableWindow, validate_curve
from strike_lab.methods import METHODS
from strike_lab.scoring import score, summarize, zone_measurements
from strike_lab.storage import Store, load_trials, rescore_session


class MemoryStore:
    def __init__(self):self.trials=[];self.trace=[]
    def check(self):pass
    def begin(self,method,parameters,stage,anchor,bias,parent=None,schedule=None,goal=StrikeGoal()):
        from strike_lab.scoring import identity
        self.trials.append(dict(id=str(len(self.trials)),metadata=dict(method=method,parameters=parameters,
            parameters_id=identity(parameters),stage=stage,anchor_rad=anchor,backend='simulation',schedule=schedule,target_deg=goal.degrees,zone_deg=goal.zone_deg)))
        return self.trials[-1]['id']
    def rows(self,trial,rows):self.trace.extend(rows)
    def finish(self,trial,score):self.trials[-1]['score']=score


def engine(plant=Plant(),limits=Limits(),backend=None):
    store=MemoryStore();backend=backend or SimBackend(plant)
    return Engine(backend,store,threading.Event(),limits=limits),store


class MethodTests(unittest.TestCase):
    def test_all_eight_families_and_nine_modes(self):
        self.assertEqual(len(METHODS),9)
        self.assertEqual(len({d.family for d in METHODS.values()}),8)

    def test_target_not_a_tuning_parameter(self):
        for d in METHODS.values():
            with self.subTest(method=d.id):
                with self.assertRaises(ValueError):d.parameters({'target':9.01})
                with self.assertRaises(ValueError):d.parameters({'target_deg':9.01})

    def test_all_methods_three_repeated_nominal_synthetic_strikes(self):
        for name in METHODS:
            e,store=engine()
            with self.subTest(method=name):
                for _ in range(3):
                    result=e.trial(name)['score']
                    self.assertTrue(result['passed'],result)
                    self.assertEqual(result['target_deg'],10.)
                    self.assertGreater(result['lower_zone_ms'],0)
                self.assertEqual(len(store.trials),3)

    def test_all_modes_work_at_a_different_fixed_anchor(self):
        for name in METHODS:
            backend=SimBackend();backend.anchor=.7
            e,_=engine(backend=backend)
            with self.subTest(method=name):
                self.assertTrue(e.trial(name)['score']['passed'])

    def test_synthetic_mismatch_is_not_hidden_as_success(self):
        # Physics mismatch is allowed to fail scoring. It must not fabricate a
        # nominal trajectory/teleport the plant into its target.
        e,_=engine(Plant(inertia=.045))
        try:r=e.trial('torque')['score']
        except RuntimeError:r=e.store.trials[-1]['score']
        self.assertFalse(r['passed'])

    def test_fixed_curve_peaks_and_continuous_bottom(self):
        for curve in (Dip(),Dip(.25,.17,.7,.55,1.1),CosineDip()):
            t=curve.down if isinstance(curve,Dip) else curve.duration/2
            before=curve.at(t-1e-7);at=curve.at(t);after=curve.at(t+1e-7)
            self.assertAlmostEqual(at[0],TARGET)
            self.assertAlmostEqual(at[1],0)
            self.assertLess(at[2],0)
            for i in range(3):self.assertAlmostEqual(before[i],after[i],delta=.001)
            self.assertEqual(curve.at(curve.duration),(0.,0.,0.))

    def test_trajectory_preflight_rejects_too_aggressive_reference(self):
        m=METHODS['cosine'].create(1.4,.16,{'duration':.30})
        with self.assertRaises(ValueError):validate_curve(m,replace(Limits(),acceleration=20))

    def test_parameter_validation(self):
        for d in METHODS.values():
            k=next(iter(d.defaults))
            for v in (math.nan,math.inf,'1',True):
                with self.assertRaises(ValueError):d.parameters({k:v})

    def test_hardware_is_opt_in_and_offline_lock_blocks_it(self):
        args,_=parse([]);self.assertFalse(args.hardware)
        with patch.dict(os.environ,{'STRIKE_LAB_OFFLINE_ONLY':'1'}), self.assertRaisesRegex(RuntimeError,'forbidden'):
            HardwareBackend(Limits())

    def test_other_programs_not_invoked_by_launcher(self):
        source=Path('launch_experiment.py').read_text()
        self.assertNotIn('camera_playback.app',source)
        self.assertNotIn('audio_bridge',source)
        self.assertNotIn('ESP32',source)

    def test_fast_mode_cannot_run_hardware(self):
        with self.assertRaises(SystemExit):parse(['--hardware','--headless','--fast'])
        with self.assertRaises(SystemExit):parse(['--fast'])


class ScoringTests(unittest.TestCase):
    def setUp(self):
        e,self.store=engine();e.trial('powered');self.rows=self.store.trace

    def test_exact_zone_interpolation(self):
        total,entries,exits,bracket=zone_measurements([0,1,2,3],[8,10,10,8])
        self.assertEqual(total,2);self.assertEqual(entries,[.5]);self.assertEqual(exits,[2.5])
        self.assertEqual(bracket,2)

    def test_equal_boundary_is_not_contact_and_reentries_count(self):
        total,entries,exits,_=zone_measurements([0,1,2,3,4,5],[9,10,9,10,9,9])
        self.assertEqual(total,4);self.assertEqual(len(entries),2);self.assertEqual(len(exits),2)
        self.assertEqual(zone_measurements([0,1,2],[8,9,8])[0],0)

    def test_nine_degree_dab_cannot_win(self):
        rows=[dict(r,depth_deg=r['depth_deg']*.905) for r in self.rows]
        s=score(rows)
        self.assertFalse(s['passed']);self.assertIn('depth_outside_acceptance',s['reasons'])

    def test_overshoot_counts_as_failure(self):
        rows=[dict(r,depth_deg=r['depth_deg']*1.05) for r in self.rows]
        self.assertIn('depth_outside_acceptance',score(rows)['reasons'])

    def test_feedback_gap_cannot_reduce_score_silently(self):
        rows=self.rows[:80]+self.rows[100:]
        self.assertIn('feedback_gap',score(rows)['reasons'])

    def test_duplicate_encoder_samples_do_not_add_fake_time(self):
        s=score(self.rows)
        copies=[r for row in self.rows for r in (row,dict(row))]
        other=score(copies)
        self.assertEqual(s['samples'],other['samples']);self.assertEqual(s['lower_zone_ms'],other['lower_zone_ms'])

    def test_failed_return_stays_failed(self):
        self.assertIn('return_not_settled',score(self.rows[:150])['reasons'])

    def test_stop_remains_a_failed_attempt(self):
        self.assertFalse(score(self.rows,abort='operator stop')['passed'])

    def test_velocity_claim_cannot_fake_entry_speed(self):
        # Stretch position timing: even unchanged raw velocity claims cannot
        # disguise an unrealistically slow crossing.
        rows=[dict(r,sample_at=r['sample_at']*10) for r in self.rows]
        self.assertIn('insufficient_entry_speed',score(rows)['reasons'])

    def test_staging_near_threshold_rejected(self):
        rows=[]
        for i in range(451):
            t=i*.002
            d=(8.8*t/.2 if t<.2 else 8.8 if t<.45 else
               8.8+(t-.45)/.05*1.2 if t<.5 else max(0,10-(t-.5)/.2*10))
            rows.append(dict(self.rows[0],sample_at=t,depth_deg=d))
        self.assertIn('prezone_pause',score(rows)['reasons'])

    def test_parameter_and_backend_groups_never_pooled(self):
        t=self.store.trials[0]
        real=dict(t,metadata=dict(t['metadata'],backend='hardware'))
        result=summarize([t,real])
        self.assertEqual(len(result['groups']),2)
        self.assertTrue(all(c['winner'] is None for c in result['comparisons']))

    def test_no_winner_for_unmatched_depths(self):
        a=self.store.trials[0]
        b=dict(a,metadata=dict(a['metadata'],method='other'),score=dict(a['score'],peak_depth_deg=10.19))
        c=summarize([a,b])['comparisons'][0]
        self.assertFalse(c['depth_matched']);self.assertIsNone(c['winner'])

    def test_any_failure_disqualifies_config_not_just_bad_trial(self):
        good=self.store.trials[0]
        bad=dict(good,score=dict(good['score'],passed=False,reasons=['test failure']))
        report=summarize([good,bad])
        self.assertEqual(report['groups'][0]['failures'],1);self.assertEqual(report['comparisons'],[])

    def test_failed_measured_strokes_are_not_omitted_from_statistics(self):
        good=self.store.trials[0]
        bad=dict(good,score=dict(good['score'],passed=False,reasons=['depth_outside_acceptance'],
                               lower_zone_ms=200.,peak_depth_deg=10.5))
        group=summarize([good,bad])['groups'][0]
        self.assertAlmostEqual(group['lower_zone_ms_median'],(good['score']['lower_zone_ms']+200)/2)
        self.assertEqual(group['lower_zone_ms_measured_attempts'],2)
        self.assertEqual(group['depth_max_deg'],10.5)
        self.assertEqual(group['failure_reasons'],{'depth_outside_acceptance':1})

    def test_rules_require_narrow_acceptance_and_real_settle_window(self):
        for kw in ({'acceptance_deg':1},{'settle_seconds':.001},{'minimum_entry_speed':math.nan}):
            with self.assertRaises(ValueError):Rules(**kw)

    def test_physical_winner_requires_completed_comparative_campaign(self):
        template=self.store.trials[0];trials=[]
        for mid in ('powered','other'):
            for stage in ('validate','endurance'):
                for _ in range(100):
                    trials.append(dict(template,metadata=dict(template['metadata'],method=mid,stage=stage,backend='hardware'),
                                       score=dict(template['score'],lower_zone_ms=50. if mid=='powered' else 80.)))
        self.assertTrue(all(c['winner'] is None for c in summarize(trials)['comparisons']))
        done=[{'stage':'all','methods':['powered','other']}]
        winners=[c['winner'] for c in summarize(trials,completed_campaigns=done)['comparisons'] if c['winner']]
        self.assertEqual(winners,['powered'])


class GuardTests(unittest.TestCase):
    def test_fault_relaxes_before_archiving_score(self):
        e,store=engine();events=[]
        original=e.backend.relax
        def relax():events.append('relax');original()
        e.backend.relax=relax
        finish=store.finish
        def archive(*args):events.append('archive');finish(*args)
        store.finish=archive
        original_receive=e.backend.receive;count=[0]
        def receive():
            s=original_receive();count[0]+=1
            return replace(s,position=e.anchor-math.radians(13)) if count[0]>10 else s
        e.backend.receive=receive
        with self.assertRaisesRegex(RuntimeError,'corridor'):e.trial('powered')
        self.assertLess(events.index('relax'),events.index('archive'))
        self.assertFalse(store.trials[-1]['score']['passed'])

    def test_old_feedback_and_wrong_operating_state_rejected(self):
        e,_=engine();s=e.backend.receive();now=e.backend.now()
        for bad in (replace(s,at=now-.1),replace(s,state=0),replace(s,position=math.nan)):
            with self.assertRaises(RuntimeError):e.check(bad,now,True)

    def test_effective_torque_and_slew_are_shared_not_just_feedforward(self):
        e,_=engine();s=e.backend.receive();now=e.backend.now()
        old=e.last_torque
        c,tau,limited=e.transmit(Command(e.anchor-.1,-2,80,4,2),s,now)
        self.assertTrue(limited);self.assertLessEqual(abs(tau),e.limits.torque)
        self.assertAlmostEqual(c.kp*(c.position-s.position)+c.kd*(c.velocity-s.velocity)+c.torque,tau)
        self.assertLessEqual(abs(tau-old),e.limits.torque_slew*max(.000001,now-(e.last_send-.01))+.001)

    def test_stability_uses_positions_not_noisy_velocity(self):
        w=StableWindow(1.4,Rules())
        for i in range(30):w.add(Sample(1.4,.2*(-1)**i,.16,2,i*.002))
        self.assertTrue(w.ready)
        w.add(Sample(1.41,0,.16,2,.060));self.assertFalse(w.ready)

    def test_curve_rejection_preserved_and_not_executed(self):
        e,store=engine(limits=replace(Limits(),acceleration=5))
        e.trial('powered')
        self.assertEqual(store.trace,[])
        self.assertFalse(store.trials[0]['score']['passed'])
        self.assertTrue(e.backend.enabled)


class ArchiveAndCampaignTests(unittest.TestCase):
    def test_archive_preserves_source_parameters_trace_score_and_rescores(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=Store(tmp,'simulation',Rules(),Limits())
            e=Engine(SimBackend(),store,threading.Event())
            e.trial('powered');e.trial('gravity');report=store.report();store.close()
            self.assertTrue(report.exists());self.assertTrue((store.path/'source/experiment.sh').exists())
            trials=load_trials(store.path);self.assertEqual(len(trials),2)
            raw=list(store.path.glob('*/trace.csv'));before={p:p.read_bytes() for p in raw}
            revised=rescore_session(store.path,replace(Rules(),acceptance_deg=.25),Limits())
            self.assertTrue((revised/'summary.json').exists())
            self.assertTrue(all(p.read_bytes()==b for p,b in before.items()))
            self.assertEqual(len(list(store.path.glob('*/score.json'))),2)

    def test_unique_sessions_do_not_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=Store(tmp,'simulation',Rules(),Limits());a.close()
            b=Store(tmp,'simulation',Rules(),Limits());b.close()
            self.assertNotEqual(a.path,b.path)

    def test_neighbors_keep_exact_ten_degree_contract(self):
        for mid in METHODS:
            choices=neighbors(mid)
            self.assertGreaterEqual(len(choices),7)
            self.assertTrue(all('target' not in c and 'depth' not in c for c in choices))
            for c in choices:METHODS[mid].parameters(c)

    def test_short_campaign_all_phases_and_saved_scheduling(self):
        e,store=engine()
        plan=Plan(screen_repeats=1,refine_repeats=1,validate_repeats=2,refine_rounds=1,endurance_seconds=1)
        out=run_campaign(e,['powered','cosine'],plan)
        self.assertEqual(set(out),{'powered','cosine'})
        stages={t['metadata']['stage'] for t in store.trials}
        self.assertTrue({'screen','refine_1','validate','endurance'}<=stages)
        self.assertTrue(all(t['metadata']['schedule'] is not None for t in store.trials if t['metadata']['stage']=='endurance'))

    def test_campaign_integer_budgets(self):
        with self.assertRaises(ValueError):Plan(screen_repeats=1.5)



class FakeStop:
    def is_set(self):return False
    def wait(self,seconds):return False


class FakeBus:
    def __init__(self,events):
        self.events=events;events.append('construct bus');self.active=False
        self.states={(side,i):(0.,0,0.) for side in ('left','right') for i in range(1,9)}
        sock=Mock();sock.getsockname.return_value=('fake-can',)
        self.sockets={'right':sock,'left':Mock()};self.right_joint7_session=None
    def fresh(self):return True
    def poll(self):pass
    def center(self,target,gripper_target,confirm_enabled=False):
        self.events.append('close gripper');self.active=True
        for i,q in enumerate(target+[gripper_target],1):self.states['right',i]=(q,2,0.)
        yield
    def set_positions(self,target):
        self.events.append('center arm')
        for i,q in enumerate(target,1):self.states['right',i]=(float(q),2,0.)
    def set_right_arm_speed(self,speed):self.events.append(('speed',speed))
    def relax(self):
        self.events.append('relax');self.active=False
        for i in range(1,9):q,_,t=self.states['right',i];self.states['right',i]=(q,0,t)


class FakePlayback:
    first_joints=[0.]*6+[1.4]
    last_joints=[-.1,.1,.2,.3,.2,-.1,.6]
    duration_s=.04
    def joints_at(self,elapsed):
        u=min(1.,elapsed/self.duration_s)
        return [a+(b-a)*u for a,b in zip(self.first_joints,self.last_joints)],u>=1.


class HardwareAdapterTests(unittest.TestCase):
    def test_early_can_reply_does_not_accelerate_command_clock(self):
        # A fake clock and receiver: no adapter, socket, or wall-clock wait.
        with patch.dict(os.environ,{'STRIKE_LAB_OFFLINE_ONLY':'0'}):
            b=HardwareBackend(Limits())
        clock=[0.];b.now=lambda:clock[0]
        b.channel=Mock()
        def early_reply(timeout):clock[0]+=min(timeout,.0002)
        b.channel.wait.side_effect=early_reply
        b.advance(.002,FakeStop())
        self.assertAlmostEqual(clock[0],.002)
        self.assertGreaterEqual(b.channel.receive.call_count,10)

    def test_notification_precedes_connection_and_only_j7_changes_after_recording(self):
        from strike_lab.backends import HardwareBackend
        events=[];bus=FakeBus(events);events.clear()
        channel=Mock();channel.sample=Sample(1.4,0,.16,2,0.)
        channel.temperature=25.
        def constructor(*args):events.append('construct bus');return bus
        def arm(helper,ch):events.append('arm J7');helper.controller.bias=.16
        with patch.dict(os.environ,{'STRIKE_LAB_OFFLINE_ONLY':'0'}), patch('camera_playback.mit_strike.Joint7Worker._prepare',arm):
            b=HardwareBackend(Limits(),notifier=lambda:events.append('notify'),
                              bus_factory=constructor,channel_factory=lambda _:channel,playback=FakePlayback())
            clock=iter(i*.01 for i in range(10000));b.now=lambda:next(clock)
            anchor,bias=b.prepare(FakeStop(),lambda _:None)
            self.assertEqual(events[:4],['notify','construct bus','close gripper','center arm'])
            self.assertEqual(anchor,.6);self.assertEqual(bias,.16)
            initial=list(events)
            b.send(Command(1.3,-.2,40,1.8,.16))
            self.assertEqual(events,initial)
            from safe_zone.encoder import FRAME
            cid,_,_=FRAME.unpack(channel.send.call_args.args[0])
            self.assertEqual(cid&255,7)
            bus.states['right',2]=(.2,2,0.);b.last_poll=0
            with self.assertRaisesRegex(RuntimeError,'motor 2 moved'):b.receive()
            b.close();self.assertIn('relax',events)

    def test_notification_failure_prevents_bus_construction(self):
        ctor=Mock()
        with patch.dict(os.environ,{'STRIKE_LAB_OFFLINE_ONLY':'0'}):
            b=HardwareBackend(Limits(),notifier=Mock(side_effect=RuntimeError('notify failed')),bus_factory=ctor,playback=FakePlayback())
            with self.assertRaisesRegex(RuntimeError,'notify failed'):b.prepare(FakeStop(),lambda _:None)
        ctor.assert_not_called()

    def test_spawned_simulator_prepares_executes_and_closes(self):
        import time
        from strike_lab.runtime import Session
        with tempfile.TemporaryDirectory() as tmp:
            session=Session(dict(output=tmp,hardware=False,realtime=False,exit_after_batch=True))
            session.request('prepare');started=False;complete=False;faults=[];deadline=time.monotonic()+30
            try:
                while session.process.is_alive() and time.monotonic()<deadline:
                    for msg in session.poll():
                        if msg['phase']=='READY' and not started:
                            session.request('run',methods=['powered'],repetitions=2);started=True
                        if msg['phase']=='BATCH COMPLETE':complete=True
                        if msg['phase']=='FAULT':faults.append(msg)
                    time.sleep(.01)
                for msg in session.poll():
                    complete|=msg['phase']=='BATCH COMPLETE'
                    if msg['phase']=='FAULT':faults.append(msg)
                self.assertFalse(faults,faults);self.assertTrue(complete)
            finally:session.close()
            self.assertFalse(session.process.is_alive())
            self.assertEqual(len(list(Path(tmp).glob('*/*/score.json'))),2)


class IntegrityTests(unittest.TestCase):
    def test_worker_terminal_interrupt_is_a_clean_stop_not_a_fault(self):
        import signal,time
        from strike_lab.runtime import Session
        with tempfile.TemporaryDirectory() as tmp:
            s=Session(dict(output=tmp,hardware=False,realtime=True));s.request('prepare')
            messages=[];sent=False;deadline=time.monotonic()+30
            try:
                while s.process.is_alive() and time.monotonic()<deadline:
                    batch=s.poll();messages+=batch
                    if not sent and any(m['phase']=='READY' for m in batch):
                        os.kill(s.process.pid,signal.SIGINT);sent=True
                    time.sleep(.01)
                messages+=s.poll()
                self.assertTrue(sent);self.assertFalse(s.process.is_alive())
                self.assertEqual(s.process.exitcode,0)
                self.assertFalse(any(m['phase']=='FAULT' for m in messages),messages)
                self.assertTrue(s.relaxed.is_set())
            finally:s.close()

    def test_finish_unpauses_and_prevents_new_requests(self):
        from strike_lab.app import App
        app=App.__new__(App);app.session=Mock();app.session.process.is_alive.return_value=True
        app.paused=True;app.pause_button=Mock();app.status=Mock();app.refresh=Mock()
        app.finish()
        self.assertFalse(app.paused);self.assertTrue(app.stopping)
        app.session.pause.assert_called_once_with(False)
        app.session.request.assert_called_once_with('finish')

    def test_incomplete_attempt_is_kept_in_loaded_reports_and_rescores(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=Store(tmp,'simulation',Rules(),Limits())
            store.begin('powered',METHODS['powered'].defaults,'manual',1.4,.16)
            report=store.report();store.close()
            groups=json.loads(report.read_text())['groups']
            self.assertEqual(groups[0]['failures'],1)
            self.assertEqual(len(load_trials(store.path)),1)
            revised=rescore_session(store.path,Rules(),Limits())
            groups=json.loads((revised/'summary.json').read_text())['groups']
            self.assertEqual(groups[0]['failures'],1)

    def test_rescore_preserves_timing_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=Store(tmp,'simulation',Rules(),Limits())
            e=Engine(SimBackend(),store,threading.Event())
            trial=e.trial('powered',stage='endurance',schedule={'requested_start':e.backend.now()-.1})
            self.assertIn('missed_start_interval',trial['score']['reasons'])
            store.close()
            revised=rescore_session(store.path,Rules(),Limits())
            rescored=json.loads((revised/(trial['id']+'.json')).read_text())
            self.assertFalse(rescored['passed']);self.assertIn('missed_start_interval',rescored['reasons'])

    def test_method_snapshot_is_immutable_after_trial(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=Store(tmp,'simulation',Rules(),Limits())
            source=store.path/'source/strike_lab/methods/powered.py'
            before=source.read_bytes()
            e=Engine(SimBackend(),store,threading.Event());e.trial('powered',{'down_time':.24})
            store.close();self.assertEqual(source.read_bytes(),before)
            meta=json.loads(next(store.path.glob('*/metadata.json')).read_text())
            self.assertEqual(meta['parameters']['down_time'],.24)
            self.assertEqual(meta['source_id'],store.source_id)

    def test_fallback_is_only_for_started_unrelaxed_dead_hardware_worker(self):
        from strike_lab.runtime import Session
        import queue,time
        from types import SimpleNamespace
        def facade(hardware,started,relaxed):
            s=Session.__new__(Session);s.hardware=hardware;s.motion_started=threading.Event();s.relaxed=threading.Event()
            if started:s.motion_started.set()
            if relaxed:s.relaxed.set()
            s.process=Mock();s.process.is_alive.return_value=False;s.process.exitcode=-9
            s.events=queue.Queue();s.heartbeat=SimpleNamespace(value=time.monotonic())
            s.fallback_attempted=False;s.saw_closed=False
            return s
        with patch('strike_lab.backends.emergency_disable_right') as disable:
            for args in ((False,True,False),(True,False,False),(True,True,True)):
                facade(*args).poll()
            disable.assert_not_called()
            s=facade(True,True,False);messages=s.poll();s.poll()
            disable.assert_called_once();self.assertTrue(s.relaxed.is_set())
            self.assertTrue(any(m['phase']=='FAULT' for m in messages))

    def test_offline_lock_also_blocks_emergency_can_transport(self):
        from strike_lab.backends import emergency_disable_right
        with patch.dict(os.environ,{'STRIKE_LAB_OFFLINE_ONLY':'1'}),patch('socket.socket') as sock:
            with self.assertRaisesRegex(RuntimeError,'forbidden'):emergency_disable_right()
            sock.assert_not_called()

    def test_stop_event_aborts_and_retains_the_attempt(self):
        e,store=engine();receive=e.backend.receive;calls=[0]
        def stopped():
            calls[0]+=1
            if calls[0]==20:e.stop.set()
            return receive()
        e.backend.receive=stopped
        with self.assertRaises(InterruptedError):e.trial('powered')
        self.assertFalse(e.backend.enabled)
        self.assertFalse(store.trials[-1]['score']['passed'])
        self.assertTrue(store.trace)

if __name__=='__main__':unittest.main()
