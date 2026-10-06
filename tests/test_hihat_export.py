import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import wave
from hihat_collection.export import export


class HiHatExportTests(unittest.TestCase):
    def fixture(self,p):
        s=p/'session';s.mkdir();r=p/'ride';(r/'recordings').mkdir(parents=True)
        for name,data in {
            'result.json':dict(success=True,relaxed_verified=True),
            'query_final_disabled.json':dict(all_16_disabled_fault_free=True),
            'hihat_result.json':dict(release_ack=True,error=None),
            'audio_result.json':dict(errors=[]),
            'examples.json':[dict(block=1,closures=1,kind='mixed',depth_deg=11,end=100.)],
        }.items():(s/name).write_text(json.dumps(data))
        for f in (s/'raw.wav',r/'recordings/r1.wav'):
            with wave.open(str(f),'wb') as w:
                w.setparams((1,2,16000,0,'NONE','not compressed'));w.writeframes(b'\x01\x00'*160000)
        (r/'manifest.json').write_text(json.dumps(dict(records=[dict(id='r1',wav='recordings/r1.wav',group='ride-session',wav_sha256=hashlib.sha256((r/'recordings/r1.wav').read_bytes()).hexdigest())])))
        review=dict(method='synthetic unit test',examples=[dict(block=1,accepted=True,clip_start_seconds=1.,clip_end_seconds=6.,onsets_raw_seconds=[3.]),dict(ride_id='r1',accepted=True)])
        return s,r,review

    def test_mixed_positive_and_ride_negative_have_correct_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);s,r,review=self.fixture(p);out=p/'out'
            records=export(s,review,out,r)
            with (out/'timestamp/t1.csv').open() as f:rows=list(csv.DictReader(f))
            self.assertEqual(rows,[dict(recording_file='r1.wav',hit_time_seconds='2.000000',label='hihat_close')])
            with (out/'timestamp/t2.csv').open() as f:self.assertEqual(list(csv.DictReader(f)),[])
            self.assertEqual((out/'recording/r2.wav').read_bytes(),(r/'recordings/r1.wav').read_bytes())
            self.assertEqual(records[1]['group'],'ride-session')
            with self.assertRaises(ValueError):export(s,review,out,r)

    def test_missing_closure_and_unsafe_shutdown_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);s,r,review=self.fixture(p)
            review['examples'][0]['onsets_raw_seconds']=[]
            with self.assertRaisesRegex(ValueError,'mixed closure'):export(s,review,p/'out',r)
            (s/'result.json').write_text('{"success": false, "relaxed_verified": false}')
            with self.assertRaisesRegex(ValueError,'relaxation'):export(s,review,p/'out',r)

    def recovered_fixture(self,s):
        for name,data in {
            'result.json':dict(success=False,relaxed_verified=False,error='Joint limit violation'),
            'post_capture_recovery.json':dict(capture_completed_before_shutdown_issue=True,issues=['Kept original failure; separately verified recovery']),
            'recovery_verified.json':dict(all_16_disabled=True,center=dict(t=103.,max_error_deg=.19,span_deg=.11,settled_s=.64)),
            'hihat_result.json':dict(release_ack=True,error='RuntimeError: Collection parent heartbeat lost',open_command_at=101.,shutdown_error=None),
        }.items():(s/name).write_text(json.dumps(data))

    def test_completed_capture_can_export_after_separate_verified_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);s,r,review=self.fixture(p);self.recovered_fixture(s)
            original=(s/'result.json').read_bytes()
            self.assertEqual(len(export(s,review,p/'out',r)),2)
            self.assertEqual((s/'result.json').read_bytes(),original)
            self.assertTrue(json.loads((p/'out/manifest.json').read_text())['shutdown_notes'])

    def test_recovery_does_not_hide_in_capture_error_or_inadequate_center(self):
        for filename,field,value in [
            ('hihat_result.json','open_command_at',99.),
            ('hihat_result.json','shutdown_error','Release failed'),
            ('query_final_disabled.json','all_16_disabled_fault_free',False),
            ('audio_result.json','errors',['overflow']),
            ('post_capture_recovery.json','capture_completed_before_shutdown_issue',False),
            ('recovery_verified.json','all_16_disabled',False),
            ('center','max_error_deg',.21),('center','span_deg',.13),
            ('center','settled_s',.5),('center','t',99.),
        ]:
            with self.subTest(field=field),tempfile.TemporaryDirectory() as tmp:
                p=Path(tmp);s,r,review=self.fixture(p);self.recovered_fixture(s)
                path=s/('recovery_verified.json' if filename=='center' else filename)
                data=json.loads(path.read_text())
                (data['center'] if filename=='center' else data)[field]=value
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):export(s,review,p/'out',r)
                self.assertFalse((p/'out').exists())


if __name__=='__main__':unittest.main()
