"""No physical hardware: finite plans, safety gates, and PTY accessory timing."""
from collections import Counter
import json
import os
from pathlib import Path
import pty
import select
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from bass_collection.plan import COUNTS, PULSE, make_plan, schedule
from bass_collection.control import BassSession
from bass_collection.transport import BassDataMotors, damaging_fault
from hihat_collection.transport import DataMotors


class BassCollectionTests(unittest.TestCase):
    def test_approved_plan(self):
        rows=make_plan()
        self.assertEqual(len(rows),60)
        self.assertEqual([r['block'] for r in rows],list(range(1,61)))
        self.assertEqual(Counter(r['kind'] for r in rows),COUNTS)
        self.assertEqual(Counter(r['split'] for r in rows),dict(train=36,validation=12,test=12))
        self.assertEqual(sum(r['bass_hits']==1 for r in rows),6)
        self.assertEqual(sum(r['bass_hits'] for r in rows),222)
        self.assertEqual({r['bpm'] for r in rows},{80,100,120})
        for row in rows:
            self.assertEqual(row['bass_pulse'],PULSE)
            acc,arm=schedule(row,10.)
            kicks=[t for t,c in acc if c=='K']
            self.assertEqual(len(kicks),row['bass_hits'])
            for a,b in zip(kicks,kicks[1:]): self.assertAlmostEqual(b-a,60./row['bpm'])
            self.assertEqual(Counter(c for _,c in arm),{k:n for k,n in [('ride',row['ride_hits']),('snare',row['snare_hits'])] if n})
            self.assertLess(max(t for t,_ in acc),16.)
        for split in ('train','validation','test'):
            self.assertEqual({r['kind'] for r in rows if r['split']==split},set(COUNTS))

    def test_no_normal_emergency_bypass(self):
        for msg in ('audio dropout','Snare failed to settle after strike','Cannot safely center','feedback lost','User cancellation'):
            self.assertFalse(damaging_fault(msg))
            with self.assertRaisesRegex(RuntimeError,'observed damaging'):
                object.__new__(BassDataMotors).emergency_stop(msg)
        for msg in ('STALL: left motor 3','right motor 7: too hot','left motor 6: fault'):
            self.assertTrue(damaging_fault(msg))
        bus=object.__new__(BassDataMotors)
        with patch.object(DataMotors,'guard_left_disable',return_value='guarded') as gate:
            self.assertEqual(bus.guard_left_disable(b'frame'),'guarded')
            gate.assert_called_once_with(b'frame')

    def test_idle_hold_matches_main_beat_without_position_differencing(self):
        from types import SimpleNamespace
        from bass_collection.transport import DriveVelocityHold
        from camera_playback.mit_center_return import MitCenterReturn
        from camera_playback.snare_beat import PATH_BIAS_GAIN,PATH_KP,PATH_KD
        old=MitCenterReturn(-.02,-.018,.2,10.)
        channel=SimpleNamespace(sample=SimpleNamespace(velocity=.015))
        actual=DriveVelocityHold(old,channel)
        expected=MitCenterReturn(-.02,-.018,.2,10.,bias_gain=PATH_BIAS_GAIN,kp=PATH_KP,kd=PATH_KD)
        for i in range(1,20):
            now=10.+i*.002
            self.assertEqual(actual.update(-.019,now),expected.update(-.019,now,measured_velocity=.015))

    def exercise_worker(self,lose_parent=False):
        master,slave=pty.openpty();port=os.ttyname(slave)
        observed=[];stop=threading.Event()
        def firmware():
            angle=False;opened=1;closed=False
            while not stop.is_set():
                if not select.select([master],[],[],.02)[0]:continue
                for c in os.read(master,256):
                    if angle:
                        if c==10:
                            os.write(master,b'ANGLE degrees=90 counts=134\r\n');angle=False
                        continue
                    if c==ord('A'): angle=True;continue
                    observed.append((time.monotonic(),chr(c)))
                    if c==ord('S'): os.write(master,b'STOP -- all motors released.\r\n')
                    if c==ord('B'): closed=True;opened=0
                    if c==ord('O'): closed=False;opened=1
                    if c==ord('Q'):
                        value=f'HIHAT v=2 min=90 max=115 angle=90 counts=134 pos={134 if closed else -13} target={134 if closed else 0} released={0 if closed else 1} opened={opened} fault=0\r\n'
                        os.write(master,value.encode())
        thread=threading.Thread(target=firmware);thread.start()
        try:
            with tempfile.TemporaryDirectory() as tmp,patch.dict(os.environ,{'PLAYBACK_OFFLINE_ONLY':'0'}):
                session=BassSession(port,Path(tmp))
                try:
                    deadline=time.monotonic()+6.
                    while session.check()['state']!='idle':
                        self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                    start=time.monotonic()+.15
                    row=next(r for r in make_plan() if r['kind']=='bass_hihat' and r['bpm']==120)
                    events,_=schedule(row,start)
                    session.block(row['block'],events)
                    if lose_parent:
                        session.heartbeat.value=time.monotonic()-5.
                        session.process.join(4.)
                    else:
                        while session.check()['completed']!=row['block']:
                            self.assertLess(time.monotonic(),start+5.);time.sleep(.01)
                        session.stop()
                    result=json.loads((Path(tmp)/'hihat_result.json').read_text())
                    self.assertTrue(result['release_ack'])
                    self.assertTrue(result['encoder_position_verified'])
                    self.assertEqual(result['bass_pulse'],PULSE)
                    if lose_parent:
                        self.assertIn('heartbeat',result['error'])
                    else:
                        self.assertIsNone(result['error'])
                        self.assertEqual(result['kicks_commanded'],4)
                        self.assertEqual(result['closures_commanded'],2)
                        kicks=[t for t,c in observed if c=='K']
                        for a,b in zip(kicks,kicks[1:]):self.assertAlmostEqual(b-a,.5,delta=.04)
                    self.assertFalse(session.process.is_alive())
                    last=[c for _,c in observed if c in 'KBO'][-1]
                    self.assertEqual(last,'O')
                finally:
                    if session.process.is_alive():session.stop()
        finally:
            stop.set();thread.join();os.close(master);os.close(slave)

    def test_serial_exact_fixed_k_and_finite_quarters(self): self.exercise_worker()
    def test_parent_loss_cancels_future_kicks_and_opens(self): self.exercise_worker(True)


class BassExportTests(unittest.TestCase):
    def fixture(self, root):
        import wave
        import numpy as np
        from bass_collection.plan import make_plan
        session=root/'session';session.mkdir()
        values={
            'result.json':dict(success=True,relaxed_verified=True,hardware_emergency=False),
            'query_final_disabled.json':dict(all_16_disabled_fault_free=True),
            'hihat_result.json':dict(error=None,shutdown_error=None,release_ack=True,encoder_position_verified=True,bass_pulse=PULSE),
            'audio_result.json':dict(errors=[]),
            'both_center_before_relax.json':{s:dict(max_error_deg=.02,span_deg=.03,settled_s=.64) for s in ('left','right')},
            'examples.json':[make_plan()[0]],
        }
        for name,value in values.items():(session/name).write_text(json.dumps(value))
        with wave.open(str(session/'raw.wav'),'wb') as wav:
            wav.setparams((1,2,16000,0,'NONE','not compressed'))
            wav.writeframes(np.arange(16000,dtype='<i2').tobytes())
        review=dict(method='Test-only acoustic fixture',examples=[dict(block=1,accepted=True,
            clip_start_seconds=0.,clip_end_seconds=1.,events=[dict(label='bass_hit',raw_seconds=.25)])])
        return session,review,root/'output'

    def test_numeric_pair_and_labels_no_overwrite(self):
        import csv
        from bass_collection.export import export
        with tempfile.TemporaryDirectory() as tmp:
            session,review,out=self.fixture(Path(tmp))
            records=export(session,review,out)
            self.assertEqual(records[0]['id'],'r1')
            self.assertEqual(records[0]['hits'],[.25])
            self.assertEqual(records[0]['annotation_scope'],'bass_only')
            self.assertEqual(records[0]['exhaustively_labeled_instruments'],['bass'])
            self.assertFalse(records[0]['accompaniment_labels_complete'])
            with (out/'timestamps/t1.csv').open() as src:
                self.assertEqual(list(csv.DictReader(src)),[dict(recording_file='r1.wav',hit_time_seconds='0.250000',label='bass_hit')])
            self.assertTrue((out/'recordings/r1.wav').is_file())
            with self.assertRaisesRegex(ValueError,'already exported'):export(session,review,out)

    def test_unreviewed_accompaniment_labels_refused(self):
        from bass_collection.export import export
        with tempfile.TemporaryDirectory() as tmp:
            session,review,out=self.fixture(Path(tmp))
            review['examples'][0]['events'].append(dict(label='snare_hit',raw_seconds=.5))
            with self.assertRaisesRegex(ValueError,'Unknown label'):export(session,review,out)
            self.assertFalse(out.exists())

    def test_missing_hit_and_unreviewed_refused(self):
        from bass_collection.export import export
        with tempfile.TemporaryDirectory() as tmp:
            session,review,out=self.fixture(Path(tmp))
            review['examples'][0]['accepted']=False
            with self.assertRaisesRegex(ValueError,'Uncertain audio'):export(session,review,out)
            review['examples'][0]['accepted']=True;review['examples'][0]['events']=[]
            with self.assertRaisesRegex(ValueError,'bass_hit'):export(session,review,out)
            self.assertFalse(out.exists())

    def test_uncentered_and_emergency_refused(self):
        from bass_collection.export import export
        with tempfile.TemporaryDirectory() as tmp:
            session,review,out=self.fixture(Path(tmp))
            f=session/'both_center_before_relax.json';centers=json.loads(f.read_text());centers['left']['max_error_deg']=1.;f.write_text(json.dumps(centers))
            with self.assertRaisesRegex(ValueError,'center proof'):export(session,review,out)
            f=session/'result.json';result=json.loads(f.read_text());result['hardware_emergency']=True;f.write_text(json.dumps(result))
            with self.assertRaisesRegex(ValueError,'centered shutdown'):export(session,review,out)


class ReverseSafetyTests(unittest.TestCase):
    def exercise(self, progress, fail=False):
        import numpy as np
        from types import SimpleNamespace
        from bass_collection.collect import BassCollector
        from x5_collection.collect import Collector
        from camera_playback.left_hold import LEFT_CENTER
        c=object.__new__(BassCollector);calls=[]
        c.left_ready=True;c.left_recording_started=True;c.left_reverse_verified=False
        c.left_recording_progress=progress;c.hihat=None;c.hybrid=None
        c.bus=SimpleNamespace(cancel_snare=lambda:calls.append('cancel'))
        c.left_recording=SimpleNamespace(side='left',duration_s=4.,first_joints=np.zeros(7),
            joints_at=lambda t:(np.full(7,t),t>=4.))
        c.event=lambda *args,**kwargs:None
        c.left_move=lambda q,label:calls.append((label,q.copy()))
        c.left_settle=lambda q,label:(calls.append(label) or dict(verified=True))
        def stream(duration,at):
            calls.append(('reverse_duration',duration))
            for t in (0.,duration/2,duration):
                calls.append(('reverse_target',at(t)[0].copy()))
                if fail: raise RuntimeError('Reverse failed')
        c.left_stream=stream
        with tempfile.TemporaryDirectory() as d:
            c.directory=Path(d)
            with patch.object(Collector,'center_and_relax',side_effect=lambda *_:calls.append('right-center-dual-relax')):
                if fail:
                    with self.assertRaisesRegex(RuntimeError,'Reverse failed'):c.center_and_relax()
                    self.assertFalse(c.left_reverse_verified)
                    self.assertNotIn('right-center-dual-relax',calls)
                    self.assertFalse(any(isinstance(x,tuple) and x[0]=='LEFT CENTER AFTER REVERSE' for x in calls))
                    return
                c.center_and_relax()
                proof=json.loads((c.directory/'left_reverse_before_center.json').read_text())
                self.assertEqual(proof['reverse_from_seconds'],progress)
        self.assertEqual(calls[0],'cancel')
        self.assertEqual(calls[1][0],'LEFT REVERSE START')
        np.testing.assert_allclose(calls[1][1],np.full(7,progress))
        self.assertEqual(calls[2],('reverse_duration',progress))
        self.assertEqual(calls[-3],'LEFT RECORDING START VERIFIED')
        self.assertEqual(calls[-2][0],'LEFT CENTER AFTER REVERSE')
        np.testing.assert_allclose(calls[-2][1],LEFT_CENTER)
        self.assertEqual(calls[-1],'right-center-dual-relax')
        self.assertTrue(c.left_reverse_verified)
        self.assertEqual(c.left_recording_progress,0.)

    def test_completed_recording_reverses_before_center_and_relax(self): self.exercise(4.)
    def test_interrupted_recording_reverses_only_played_prefix(self): self.exercise(1.2)
    def test_failed_reverse_never_falls_back_to_direct_center(self): self.exercise(4.,True)

if __name__=='__main__': unittest.main()
