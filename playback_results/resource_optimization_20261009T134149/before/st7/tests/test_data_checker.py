"""Read-only dataset validation and GUI regression tests (standard library only)."""
import hashlib
from pathlib import Path
import struct
import tempfile
import time
import tkinter as tk
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import wave

from data_checker import DataCheckerApp, DatasetError, index_folder, load_audio, load_timestamps, pair_folders


class DataCheckerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.wav = self.base / 'recordings'
        self.csv = self.base / 'timestamps'
        self.wav.mkdir()
        self.csv.mkdir()

    def pair(self, number, width=2, channels=1, rate=8000):
        path = self.wav / f'r{number}.wav'
        # Alternating signed PCM extrema (unsigned for 8-bit).
        sample = bytes([0, 255]) if width == 1 else (
            (-(1 << (8 * width - 1))).to_bytes(width, 'little', signed=True)
            + ((1 << (8 * width - 1)) - 1).to_bytes(width, 'little', signed=True))
        with wave.open(str(path), 'wb') as out:
            out.setnchannels(channels)
            out.setsampwidth(width)
            out.setframerate(rate)
            out.writeframes(sample * channels * (rate // 2))
        (self.csv / f't{number}.csv').write_text(
            f'recording_file,hit_time_seconds,label\nr{number}.wav,0.250000,cymbal_hit\n')
        return path

    def test_numeric_order_and_gaps(self):
        for n in (10, 2, 1):
            self.pair(n)
        self.assertEqual([p.number for p in pair_folders(self.wav, self.csv)], [1, 2, 10])
        self.assertEqual([p.csv.name for p in pair_folders(self.wav, self.csv)], ['t1.csv', 't2.csv', 't10.csv'])

    def test_reject_invalid_names_without_renaming(self):
        bad = self.wav / 'recording.wav'
        bad.write_bytes(b'unchanged')
        with self.assertRaisesRegex(DatasetError, 'Invalid filename'):
            index_folder(self.wav, 'r', 'wav')
        self.assertEqual(bad.read_bytes(), b'unchanged')

    def test_duplicate_numbers(self):
        self.pair(1)
        (self.wav / 'r01.wav').write_bytes(b'')
        with self.assertRaisesRegex(DatasetError, 'Duplicate number 1'):
            pair_folders(self.wav, self.csv)

    def test_missing_partners(self):
        self.pair(1)
        (self.csv / 't2.csv').write_text('')
        (self.wav / 'r3.wav').write_bytes(b'')
        with self.assertRaises(DatasetError) as error:
            pair_folders(self.wav, self.csv)
        self.assertIn('r3.wav', str(error.exception))
        self.assertIn('t2.csv', str(error.exception))

    def test_empty_missing_folders_and_unrelated_files(self):
        with self.assertRaisesRegex(DatasetError, 'No r'):
            index_folder(self.wav, 'r', 'wav')
        with self.assertRaisesRegex(DatasetError, 'Cannot read folder'):
            index_folder(self.base / 'absent', 'r', 'wav')
        self.pair(1)
        (self.wav / 'notes.txt').write_text('ignored')
        self.assertEqual(len(pair_folders(self.wav, self.csv)), 1)

    def test_audio_formats_and_channels(self):
        for width in (1, 2, 3, 4):
            with self.subTest(width=width):
                path = self.pair(width, width=width, channels=2, rate=16000)
                audio = load_audio(path)
                self.assertEqual(audio.duration, 1.0)
                self.assertEqual(audio.channels, 2)
                self.assertEqual(audio.sample_width, width)
                self.assertEqual(max(audio.peaks), 1.0)

    def test_empty_corrupt_truncated_audio(self):
        path = self.pair(1)
        path.write_bytes(path.read_bytes()[:-20])
        with self.assertRaisesRegex(DatasetError, 'truncated'):
            load_audio(path)
        path.write_bytes(b'not wav')
        with self.assertRaises(DatasetError):
            load_audio(path)
        with wave.open(str(path), 'wb') as out:
            out.setnchannels(1)
            out.setsampwidth(2)
            out.setframerate(8000)
            out.writeframes(b'')
        with self.assertRaisesRegex(DatasetError, 'empty'):
            load_audio(path)

    def test_timestamp_validation(self):
        self.pair(1)
        path = self.csv / 't1.csv'
        marks, warnings = load_timestamps(path, 'r1.wav', 1.0)
        self.assertEqual(marks, [0.25])
        self.assertEqual(warnings, [])
        path.write_text('recording_file,hit_time_seconds,label\n')
        self.assertEqual(load_timestamps(path, 'r1.wav', 1.0), ([], []))
        path.write_text('recording_file,hit_time_seconds,label\n'
                        'r9.wav,0.75,wrong\nr1.wav,0.25,cymbal_hit\nr1.wav,0.25,cymbal_hit\n'
                        'r1.wav,nan,cymbal_hit\nr1.wav,inf,cymbal_hit\nr1.wav,-1,cymbal_hit\n'
                        'r1.wav,2,cymbal_hit\nr1.wav,nope,cymbal_hit\nr1.wav\n')
        marks, warnings = load_timestamps(path, 'r1.wav', 1.0)
        self.assertEqual(marks, [0.75, 0.25, 0.25])
        self.assertEqual(len(warnings), 10)
        path.write_text('wrong,header\n')
        with self.assertRaisesRegex(DatasetError, 'requires columns'):
            load_timestamps(path, 'r1.wav', 1.0)

    def test_gui_navigation_markers_readonly_and_playback(self):
        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(f'GUI display unavailable: {exc}')
        app = DataCheckerApp(root)
        self.addCleanup(app.close)
        for n in (10, 2, 1):
            self.pair(n)
        files = list(self.wav.iterdir()) + list(self.csv.iterdir())
        before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        with patch('data_checker.messagebox.showerror') as error:
            app.set_folder('wav', self.wav)
            self.assertIsNone(app.audio)
            app.set_folder('csv', self.csv)
            root.update()
            self.assertEqual(app.pairs[app.index].number, 1)
            self.assertEqual(len(app.wave_canvas.find_withtag('hit_marker')), 1)
            self.assertEqual(app.marks, [0.25])
            self.assertTrue(app.back_button.instate(['disabled']))
            self.assertTrue(app.play_button.instate(['!disabled']))
            app.next_button.invoke()
            self.assertEqual(app.pairs[app.index].number, 2)
            app.next_button.invoke()
            self.assertEqual(app.pairs[app.index].number, 10)
            self.assertTrue(app.next_button.instate(['disabled']))
            app.back_button.invoke()
            self.assertEqual(app.pairs[app.index].number, 2)
            # A click/drag over a marker must seek, not alter the timestamp.
            event = SimpleNamespace(x=app.wave_canvas.winfo_width() / 2)
            app._begin_seek(event)
            app._end_seek(event)
            self.assertAlmostEqual(app.position, 0.5, places=2)
            self.assertEqual(app.marks, [0.25])
            app.root.event_generate('<Delete>')
            self.assertEqual(app.marks, [0.25])
            # Mock the output process, not the UI; assert actual PCM and parameters.
            with patch('data_checker.shutil.which', return_value='/usr/bin/paplay'), \
                 patch('data_checker.subprocess.Popen') as popen:
                process = popen.return_value
                process.communicate.return_value = (None, b'')
                process.returncode = 0
                process.poll.return_value = 0
                app.play_button.invoke()
                for _ in range(30):
                    root.update()
                    if app.process is None:
                        break
                    time.sleep(0.01)
                self.assertIn('--format=s16le', popen.call_args.args[0])
                self.assertIn('--rate=8000', popen.call_args.args[0])
                self.assertGreater(len(process.communicate.call_args.kwargs['input']), 0)
                self.assertIsNone(app.process)
            error.assert_not_called()
        self.assertEqual(before, {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in files})

    def test_gui_errors_and_stale_playback_event(self):
        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        app = DataCheckerApp(root)
        self.addCleanup(app.close)
        self.pair(1)
        self.pair(2)
        (self.csv / 't1.csv').write_text('wrong,header\n')
        app.set_folder('wav', self.wav)
        app.set_folder('csv', self.csv)
        self.assertIsNotNone(app.audio)  # Bad CSV does not prevent audio review.
        self.assertIn('requires columns', app.details.get('1.0', 'end'))
        app.navigate(1)
        app.events.put((-1, 1, 'old playback failure'))
        with patch('data_checker.messagebox.showerror') as error:
            root.after(80, root.quit)
            root.mainloop()
            error.assert_not_called()
        self.assertEqual(app.pairs[app.index].number, 2)
        self.assertEqual(app.marks, [0.25])


if __name__ == '__main__':
    unittest.main()
