"""Model-based forward kinematics and explicitly non-certified convex envelopes."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.transform import Rotation

FRAME = 'world'
LEFT_TCP = 'openarmx_left_hand_tcp'
RIGHT_TCP = 'openarmx_right_hand_tcp'
# Backward-compatible name used by the left-arm motion programs.
TCP = LEFT_TCP
SCHEMAS = {
    LEFT_TCP: 'openarmx-left-envelope-v1',
    RIGHT_TCP: 'openarmx-right-envelope-v1',
}
MAX_POINTS = 50000
# Classification only: never alter the recorded hull or its RViz markers.
MEMBERSHIP_BUFFER_M = 0.005

class Model:
    def __init__(self, path):
        data = Path(path).read_bytes()
        self.digest = hashlib.sha256(data).hexdigest()
        root = ET.fromstring(data)
        if root.find('ros2_control') is not None:
            raise ValueError('Hardware control tags are forbidden')
        self.joints = list(root.findall('joint'))
        self.names = [j.get('name') for j in self.joints if j.get('type') != 'fixed' and j.find('mimic') is None]

    def transforms(self, values):
        frames = {FRAME: np.eye(4)}
        pending = self.joints.copy()
        while pending:
            progress = False
            for j in pending[:]:
                parent = j.find('parent').get('link')
                if parent not in frames:
                    continue
                origin = j.find('origin')
                t = np.eye(4)
                if origin is not None:
                    t[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                    t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
                q = values.get(j.get('name'), 0.)
                mimic = j.find('mimic')
                if mimic is not None:
                    q = values.get(mimic.get('joint'), 0.) * float(mimic.get('multiplier', 1)) + float(mimic.get('offset', 0))
                motion = np.eye(4)
                if j.get('type') != 'fixed':
                    axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                    if j.get('type') == 'prismatic':
                        motion[:3, 3] = axis * q
                    else:
                        motion[:3, :3] = Rotation.from_rotvec(axis / np.linalg.norm(axis) * q).as_matrix()
                frames[j.find('child').get('link')] = frames[parent] @ t @ motion
                pending.remove(j)
                progress = True
            if not progress:
                raise ValueError('Disconnected URDF')
        return frames

class Zone:
    def __init__(self, model_hash, tcp=TCP):
        if tcp not in SCHEMAS:
            raise ValueError('Unsupported TCP frame')
        self.model_hash = model_hash
        self.tcp = tcp
        self.points = []
        self.hull = None
        self._dirty = True

    def add(self, point):
        p = np.asarray(point, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all() or np.max(np.abs(p)) > 5:
            raise ValueError('Invalid point')
        if len(self.points) >= MAX_POINTS:
            raise ValueError('Sample limit reached; save and start a new zone')
        if self.points and np.linalg.norm(p - self.points[-1]) < .005:
            return False
        self.points.append(p.tolist())
        self._dirty = True
        return True

    def rebuild(self):
        if not self._dirty: return
        self._dirty = False
        self.hull = None
        p = np.asarray(self.points)
        if len(p) < 4 or np.linalg.matrix_rank(p - p[0], tol=1e-5) < 3:
            return
        try:
            h = ConvexHull(p)  # No jitter: never invent depth for flat traces.
            if h.volume > 1e-9:
                self.hull = h
        except QhullError:
            pass

    def contains(self, p, buffer_m=0.):
        """Test original hull with optional outward tolerance per face, in meters.

        This offsets the acceptance planes only, not the stored/drawn geometry.
        At edges/corners this is a plane-offset envelope, not an exact rounded
        Euclidean-distance shell. The default remains the original strict test.
        """
        if not np.isfinite(buffer_m) or buffer_m < 0:
            raise ValueError('Buffer must be finite and nonnegative')
        p = np.asarray(p, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            return False
        if self.hull is None:
            return False
        normals = self.hull.equations[:, :3]
        distance = (normals @ p + self.hull.equations[:, 3]) / np.linalg.norm(normals, axis=1)
        return bool(np.all(distance <= buffer_m + 1e-9))

    def save(self, path):
        self.rebuild()
        h = self.hull
        obj = dict(schema=SCHEMAS[self.tcp], frame=FRAME, tcp=self.tcp, units='m',
                   model_sha256=self.model_hash, method='convex_hull', certified_safe=False,
                   points=self.points, vertices=[] if h is None else h.points[h.vertices].tolist(),
                   planes=[] if h is None else h.equations.tolist(),
                   warning='TCP estimate only; fills concavities; not a collision guard or whole-arm safe region.')
        path = Path(path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=path.parent, prefix='.zone-', suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(obj, f, indent=2, allow_nan=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, path)
        finally:
            if os.path.exists(tmp): os.unlink(tmp)

    @classmethod
    def load(cls, path, model_hash, tcp=TCP):
        if tcp not in SCHEMAS:
            raise ValueError('Unsupported TCP frame')
        if Path(path).stat().st_size > 15000000:
            raise ValueError('Zone file too large')
        obj = json.loads(Path(path).read_text())
        expected = dict(schema=SCHEMAS[tcp], frame=FRAME, tcp=tcp, units='m',
                        method='convex_hull', model_sha256=model_hash, certified_safe=False)
        if any(obj.get(k) != v for k, v in expected.items()):
            raise ValueError('Incompatible zone schema, frame, TCP, units or robot model')
        p = np.asarray(obj['points'], dtype=float)
        if p.size == 0: p = np.empty((0, 3))
        if p.ndim != 2 or p.shape[1] != 3 or len(p) > MAX_POINTS or not np.isfinite(p).all() or (p.size and np.max(np.abs(p)) > 5):
            raise ValueError('Invalid saved points')
        z = cls(model_hash, tcp)
        z.points = p.tolist()
        z.rebuild()  # Never trust imported hull/plane data.
        return z
