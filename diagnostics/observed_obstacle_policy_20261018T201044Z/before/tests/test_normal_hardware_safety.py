"""Normal hardware stop/close contracts, with no physical transport."""
import unittest
from unittest.mock import Mock

from camera_playback.app import App, CENTER_RELAX_PHASE
from camera_playback.playback_transport import PlaybackMotors


class NormalHardwareSafetyTests(unittest.TestCase):
    def app(self):
        a = App.__new__(App)
        a.single_run_mode = True
        a.bus = Mock(spec=PlaybackMotors, active=True)
        a.root = Mock()
        a.center_relax = Mock()
        a.close_requested = False
        return a

    def test_relax_request_away_from_center_routes_to_center_not_disable(self):
        a = self.app()
        a.bus.center_evidence.side_effect = RuntimeError('not centered')
        a.relax()
        a.center_relax.assert_called_once()
        a.bus.relax.assert_not_called()

    def test_major_stall_and_thermal_faults_are_explicit_exceptions(self):
        for message in ('STALL: motor 4', 'right motor 7: too hot'):
            a = self.app()
            a.relax = Mock()
            a.fail(message)
            a.relax.assert_called_once_with('MAJOR FAULT: '+message)

    def test_close_waits_for_center_and_disabled_confirmation(self):
        a = self.app()
        a.request_safe_close()
        a.center_relax.assert_called_once()
        self.assertTrue(a.close_requested)
        a.phase = CENTER_RELAX_PHASE
        a._safe_close_tick()
        a.root.quit.assert_not_called()
        a.bus.active = False
        a.phase = 'RELAXING'
        a._safe_close_tick()
        a.root.quit.assert_not_called()
        a.phase = 'RELAXED'
        a._safe_close_tick()
        a.root.quit.assert_called_once()


if __name__ == '__main__':
    unittest.main()
