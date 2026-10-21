"""Visualization-only tests; never instantiate a hardware controller."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
from camera_playback.app import App
from camera_playback.dual_recording import left_geometry
from camera_playback.zone_markers import left_zone_markers
from safe_zone.geometry import Model, Zone, LEFT_TCP
from smooth_playback.trajectory import ROOT


class BeatZoneMarkerTests(unittest.TestCase):
    @staticmethod
    def marker(index, kind, color, scale=.01):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns = 'right_goal_motion'
        marker.id, marker.type, marker.action = index, kind, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def test_exact_left_validation_hull_and_separate_namespace(self):
        g = left_geometry(Model(ROOT/'model/openarmx.urdf'))
        markers = left_zone_markers(self.marker, g.zone)
        mesh, edges, label = markers
        self.assertEqual({m.ns for m in markers}, {'left_beat_zone'})
        self.assertEqual({m.id for m in markers}, {0, 1, 2})
        self.assertTrue(all(m.header.frame_id == 'world' for m in markers))
        expected = []
        for triangle in g.zone.hull.simplices:
            ps = [tuple(g.zone.hull.points[i]) for i in triangle]
            expected.extend(ps+ps[::-1])
        self.assertEqual([(p.x,p.y,p.z) for p in mesh.points], expected)
        self.assertEqual(len(edges.points), len(expected))
        self.assertEqual(mesh.type, Marker.TRIANGLE_LIST)
        self.assertLess(mesh.color.a, .3)
        self.assertGreater(mesh.color.g, mesh.color.b)
        self.assertIn('LEFT zone1', label.text)
        self.assertEqual(g.zone.tcp, LEFT_TCP)

    def test_empty_zone_deletes_markers(self):
        markers = left_zone_markers(self.marker, Zone('0'*64, LEFT_TCP))
        self.assertTrue(all(m.action == Marker.DELETE for m in markers))

    def test_app_keeps_right_markers_and_reuses_static_left_mesh(self):
        app = App.__new__(App)
        app.target_point = None
        app.marker = self.marker
        stamp = Mock(return_value=Time(sec=1))
        app.node = SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=stamp)))
        app.dual = SimpleNamespace(g=left_geometry(Model(ROOT/'model/openarmx.urdf')))
        first = app.extra_markers()
        second = app.extra_markers()
        self.assertEqual([(m.ns,m.id) for m in first[:2]], [('right_goal_motion',5),('right_goal_motion',6)])
        self.assertEqual(len(first), 5)
        self.assertIs(first[2], second[2])
        stamp.return_value = Time(sec=2)
        self.assertEqual(app.extra_markers()[2].header.stamp.sec, 2)
        del app.dual
        self.assertEqual(len(app.extra_markers()), 2)


if __name__ == '__main__':
    unittest.main()
