"""Snare exports are reviewed, append-only and require BOTH center proofs."""
import csv
import json
from pathlib import Path
import tempfile
import unittest
import wave
from hihat_collection.export import export


class CollectionExportTests(unittest.TestCase):
    def fixture(self,p,name='session'):
        s=p/name;s.mkdir()
        proof=dict(max_error_deg=.1,span_deg=.05,settled_s=.65)
        for name,data in {
            'result.json':dict(success=True,relaxed_verified=True),
            'both_center_before_relax.json':dict(left=proof,right=proof),
            'query_final_disabled.json':dict(all_16_disabled_fault_free=True),
            'hihat_result.json':dict(release_ack=True,error=None),
            'audio_result.json':dict(errors=[]),
            'examples.json':[dict(block=1,snare_hits=1,kind='snare_ride',snare_degrees=11,ride_depth_deg=10,split='development'),
                             dict(block=2,snare_hits=0,kind='ride_only',split='development')],
        }.items():(s/name).write_text(json.dumps(data))
        with wave.open(str(s/'raw.wav'),'wb') as w:
            w.setparams((1,2,16000,0,'NONE','not compressed'));w.writeframes(b'\x01\x00'*160000)
        review=dict(method='synthetic unit test',examples=[
            dict(block=1,accepted=True,clip_start_seconds=1.,clip_end_seconds=6.,onsets_raw_seconds=[3.]),
            dict(block=2,accepted=True,clip_start_seconds=6.,clip_end_seconds=9.,onsets_raw_seconds=[])])
        return s,review

    def test_labels_negatives_append_and_no_duplicates(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);s,review=self.fixture(p);out=p/'out'
            records=export(s,review,out)
            with (out/'timestamps/t1.csv').open() as f:
                self.assertEqual(list(csv.DictReader(f)),[dict(recording_file='r1.wav',hit_time_seconds='2.000000',label='snare_hit')])
            with (out/'timestamps/t2.csv').open() as f:self.assertEqual(list(csv.DictReader(f)),[])
            original=(out/'recordings/r1.wav').read_bytes()
            with self.assertRaisesRegex(ValueError,'already exported'):export(s,review,out)
            s2,review2=self.fixture(p,'session2');records=export(s2,review2,out)
            self.assertEqual([r['id'] for r in records],['r3','r4'])
            self.assertEqual((out/'recordings/r1.wav').read_bytes(),original)

    def test_missing_snare_and_incomplete_review_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);s,review=self.fixture(p)
            review['examples'][0]['onsets_raw_seconds']=[]
            with self.assertRaisesRegex(ValueError,'onset count'):export(s,review,p/'out')
            review['examples'].pop()
            with self.assertRaisesRegex(ValueError,'every captured'):export(s,review,p/'out')

    def test_both_center_and_fault_free_capture_required(self):
        cases=[('result.json','success',False),('query_final_disabled.json','all_16_disabled_fault_free',False),
               ('hihat_result.json','release_ack',False),('audio_result.json','errors',['overflow'])]
        for filename,key,value in cases:
            with self.subTest(key=key),tempfile.TemporaryDirectory() as tmp:
                p=Path(tmp);s,review=self.fixture(p);f=s/filename;d=json.loads(f.read_text());d[key]=value;f.write_text(json.dumps(d))
                with self.assertRaises(ValueError):export(s,review,p/'out')
        for side in ('left','right'):
            with self.subTest(side=side),tempfile.TemporaryDirectory() as tmp:
                p=Path(tmp);s,review=self.fixture(p);f=s/'both_center_before_relax.json';d=json.loads(f.read_text());d[side]['max_error_deg']=.21;f.write_text(json.dumps(d))
                with self.assertRaisesRegex(ValueError,'Both arms'):export(s,review,p/'out')

    def test_preexisting_file_never_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);s,review=self.fixture(p);out=p/'out';(out/'recordings').mkdir(parents=True)
            f=out/'recordings/r1.wav';f.write_bytes(b'keep')
            with self.assertRaisesRegex(ValueError,'overwrite'):export(s,review,out)
            self.assertEqual(f.read_bytes(),b'keep')

    def test_archived_ride_negative_keeps_provenance_and_empty_labels(self):
        import hashlib
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);session,review=self.fixture(p);ride=p/'ride';(ride/'recordings').mkdir(parents=True)
            src=ride/'recordings/r1.wav';src.write_bytes((session/'raw.wav').read_bytes())
            digest=hashlib.sha256(src.read_bytes()).hexdigest()
            (ride/'manifest.json').write_text(json.dumps(dict(records=[dict(id='r1',wav='recordings/r1.wav',group='old-ride-session',wav_sha256=digest,commanded_depth_deg=11.5)])))
            review['examples'][1]['source_ride_id']='r1'
            rows=export(session,review,p/'out',ride)
            self.assertEqual(rows[1]['group'],'old-ride-session')
            self.assertEqual(rows[1]['commanded_ride_degrees'],11.5)
            self.assertEqual(rows[1]['wav_sha256'],digest)
            with (p/'out/timestamps/t2.csv').open() as f:self.assertEqual(list(csv.DictReader(f)),[])
            with self.assertRaisesRegex(ValueError,'already exported'):export(session,review,p/'out',ride)
            review['examples'][0]['source_ride_id']='r1'
            with self.assertRaisesRegex(ValueError,'development ride-only'):export(session,review,p/'bad',ride)

if __name__=='__main__':unittest.main()
