import csv
import json
from pathlib import Path
import tempfile
import unittest
import wave

from x5_collection.export import export


class ExportTests(unittest.TestCase):
    def fixture(self, path):
        session = path/'session'; session.mkdir()
        for name, obj in {
            'result.json': {'relaxed_verified': True},
            'query_final_disabled.json': {'all_16_disabled_fault_free': True},
            'audio_result.json': {'errors': []},
            'blocks.json': [{'block': 1, 'depth_deg': 11, 'kind': 'double', 'strikes': 2}],
        }.items():
            (session/name).write_text(json.dumps(obj))
        (session/'strikes.jsonl').write_text('')
        with wave.open(str(session/'raw.wav'), 'wb') as w:
            w.setparams((1, 2, 16000, 0, 'NONE', 'not compressed'))
            w.writeframes(b'\x01\x00'*160000)
        review = dict(method='synthetic test annotations', examples=[dict(block=1, accepted=True,
                            onsets_raw_seconds=[3., 3.2])])
        return session, review

    def test_st7_wav_csv_style_and_separate_double_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory); s, review = self.fixture(path)
            records = export(s, review, path/'output')
            record = records[0]
            with (path/'output'/record['csv']).open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(set(rows[0]), {'recording_file', 'hit_time_seconds', 'label'})
            self.assertEqual([float(r['hit_time_seconds']) for r in rows], [2., 2.2])
            self.assertTrue(all(r['label'] == 'cymbal_hit' for r in rows))
            with wave.open(str(path/'output'/record['wav'])) as w:
                self.assertEqual((w.getframerate(), w.getnchannels(), w.getsampwidth()), (16000, 1, 2))
            self.assertFalse(record['human_verified'])
            number = record['id'].removeprefix('r')
            self.assertEqual(Path(record['wav']).name, f'r{number}.wav')
            self.assertEqual(Path(record['csv']).name, f't{number}.csv')
            self.assertEqual(record['hits'], [float(r['hit_time_seconds']) for r in rows])
            with self.assertRaisesRegex(ValueError, 'overwrite'):
                export(s, review, path/'output')

    def test_variable_clip_margins_and_rounding_match_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory); s, review = self.fixture(path)
            review['examples'][0].update(onsets_raw_seconds=[3.0000625, 3.2186875],
                                         lead_in_seconds=1.75, tail_seconds=4.)
            records = export(s, review, path/'output')
            with (path/'output'/records[0]['csv']).open() as stream:
                hits = [float(r['hit_time_seconds']) for r in csv.DictReader(stream)]
            self.assertEqual(hits, records[0]['hits'])
            self.assertAlmostEqual(hits[0], 1.75)
            self.assertAlmostEqual(records[0]['duration'], 5.968625)

    def test_refuses_labeling_before_verified_relax(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory); s, review = self.fixture(path)
            (s/'result.json').write_text('{"relaxed_verified": false}')
            with self.assertRaisesRegex(ValueError, 'disabled feedback'):
                export(s, review, path/'output')

    def test_incomplete_double_is_not_exported_as_mislabeled_training_data(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory); s, review = self.fixture(path)
            review['examples'][0]['onsets_raw_seconds'] = [3.]
            with self.assertRaisesRegex(ValueError, 'Ambiguous'):
                export(s, review, path/'output')


if __name__ == '__main__':
    unittest.main()
