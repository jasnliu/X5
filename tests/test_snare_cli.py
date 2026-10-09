"""Snare depth argument contracts; never run a physical controller."""
import contextlib
import io
import unittest
from unittest.mock import patch

from snare_lab.cli import main, parse, parser


class SnareCliTests(unittest.TestCase):
    def test_default_is_positive_ten_and_simulation(self):
        args = parse([])
        self.assertEqual(args.degrees, 10.)
        self.assertFalse(args.hardware)

    def test_numeric_shorthand_is_positive(self):
        for option, expected in (('--10', 10.), ('--11', 11.), ('--11.5', 11.5),
                                 ('--12', 12.), ('--20', 20.), ('--40', 40.),
                                 ('--0.5', .5), ('--.5', .5)):
            for mode in ([], ['--simulate'], ['--hardware']):
                with self.subTest(option=option, mode=mode):
                    self.assertEqual(parse(mode + [option]).degrees, expected)
                    self.assertEqual(parse([option] + mode).degrees, expected)

    def test_existing_degrees_option_is_preserved(self):
        for argv in (['--degrees', '11.5'], ['--degrees=11.5']):
            self.assertEqual(parse(argv).degrees, 11.5)

    def test_process_arguments_and_recording_are_preserved(self):
        with patch('sys.argv', ['snare.sh', '--11.5', '--recording', 'custom.json']):
            args = parse()
        self.assertEqual(args.degrees, 11.5)
        self.assertEqual(str(args.recording), 'custom.json')

    def test_invalid_and_ambiguous_depths_fail(self):
        cases = [
            ['--0'], ['--0.1'], ['---11'], ['--11.5.2'],
            ['--degrees', '-11'], ['--degrees', 'nan'], ['--degrees', 'inf'],
            ['--10', '--11'], ['--11', '--11'], ['--degrees', '10', '--11'],
            ['--11', '--degrees=10'], ['--degrees=10', '--degrees=11'],
            ['--hardware', '--simulate'], ['--hardware', '--fast'],
            ['--', '--11'],
        ]
        for argv in cases:
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as exc:
                    parse(argv)
                self.assertEqual(exc.exception.code, 2)

    def test_help_documents_shorthand(self):
        self.assertIn('--11.5', parser().format_help())

    def test_main_passes_depth_to_each_runner_without_running_it(self):
        for mode, method in (('--simulate', 'run_simulate'), ('--hardware', 'run_hardware')):
            with self.subTest(mode=mode), patch('snare_lab.runner.' + method, return_value=0) as run:
                self.assertEqual(main([mode, '--11.5']), 0)
                self.assertEqual(run.call_args.args[0].degrees, 11.5)

    def test_target_increases_only_left_joint_six(self):
        import math
        import numpy as np
        from types import SimpleNamespace
        from snare_lab.runner import dipped_target
        anchor = np.array([0., .1, .2, .3, .4, .5, .6])
        recording = SimpleNamespace(last_joints=anchor)
        target = dipped_target(recording, parse(['--11.5']).degrees)
        delta = np.zeros(7)
        delta[5] = math.radians(11.5)
        np.testing.assert_allclose(target - anchor, delta)
        np.testing.assert_array_equal(recording.last_joints, anchor)


if __name__ == '__main__':
    unittest.main()
