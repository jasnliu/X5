"""Display the beat controller's existing left TCP envelope; no motion logic."""
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def left_zone_markers(marker_factory, zone):
    """Use the very same hull as validation, in its world-frame coordinates."""
    zone.rebuild()
    mesh = marker_factory(0, Marker.TRIANGLE_LIST, (.15, 1., .35, .16), 1.)
    edges = marker_factory(1, Marker.LINE_LIST, (.25, 1., .4, 1.), .002)
    label = marker_factory(2, Marker.TEXT_VIEW_FACING, (.4, 1., .5, 1.), .03)
    markers = [mesh, edges, label]
    for marker in markers:
        marker.ns = 'left_beat_zone'
    if zone.hull is None:
        for marker in markers:
            marker.action = Marker.DELETE
        return markers
    hull = zone.hull
    for triangle in hull.simplices:
        points = [Point(x=float(hull.points[i, 0]), y=float(hull.points[i, 1]),
                        z=float(hull.points[i, 2])) for i in triangle]
        # Both windings keep the translucent boundary visible from either side.
        mesh.points.extend(points + points[::-1])
        edges.points.extend([points[0], points[1], points[1], points[2], points[2], points[0]])
    center = np.mean(hull.points[hull.vertices], axis=0)
    label.pose.position = Point(x=float(center[0]), y=float(center[1]),
                                z=float(np.max(hull.points[:, 2]) + .04))
    label.text = 'LEFT zone1 — estimated TCP envelope'
    return markers
