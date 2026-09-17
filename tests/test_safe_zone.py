import itertools
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from safe_zone.encoder import FRAME, EFF, PAYLOAD, request_frame, decode, Observer, RIGHT_JOINT5_ZERO_OFFSET, encoder_to_joint, joint_to_motor
from safe_zone.geometry import Model, Zone, TCP, LEFT_TCP, RIGHT_TCP, MEMBERSHIP_BUFFER_M

ROOT = Path(__file__).resolve().parents[1]

class ProtocolTests(unittest.TestCase):
    def test_right_joint5_uses_only_measured_zero_offset(self):
        self.assertAlmostEqual(np.degrees(RIGHT_JOINT5_ZERO_OFFSET), -8.5)
        self.assertAlmostEqual(np.degrees(encoder_to_joint('right', 5, np.radians(-9.5))), 1.)
        self.assertAlmostEqual(np.degrees(joint_to_motor('right', 5, np.radians(1.))), -9.5)
        for side, motor in (('left', 5), ('right', 4), ('right', 6)):
            self.assertAlmostEqual(encoder_to_joint(side, motor, .25), -.25)
            self.assertAlmostEqual(joint_to_motor(side, motor, -.25), .25)

    def test_sdk_query_bytes(self):
        for i in range(1,9):
            self.assertEqual(request_frame(i), FRAME.pack(EFF | 0x0200fd00 | i, 8, PAYLOAD))
        for i in (0,9):
            with self.assertRaises(ValueError): request_frame(i)

    def test_disabled_decode(self):
        for raw in (0, 20000, 32768, 65535):
            packet = FRAME.pack(EFF | 0x020003fd, 8, raw.to_bytes(2,'big') + bytes(6))
            motor, angle = decode(packet)
            self.assertEqual(motor, 3)
            self.assertAlmostEqual(angle, raw / 65535 * 25.14 - 12.57)

    def test_enabled_and_faults_rejected(self):
        for bit in range(16,24):
            with self.assertRaises(RuntimeError): decode(FRAME.pack(EFF | 0x020001fd | 1<<bit,8,bytes(8)))
        with self.assertRaises(RuntimeError): decode(FRAME.pack(EFF | 0x150001fd,8,bytes(8)))
        with self.assertRaises(RuntimeError): decode(bytes(4))
        with self.assertRaises(RuntimeError): decode(FRAME.pack(0x20000000,8,bytes(8)))

    def test_identical_buses_rejected_before_socket(self):
        with patch('socket.socket', side_effect=AssertionError('Must not open socket')):
            with self.assertRaises(ValueError): Observer('can0','can0')

class GeometryTests(unittest.TestCase):
    def setUp(self): self.z = Zone('test')

    def cube(self):
        for p in itertools.product((0.,1.),repeat=3): self.z.add(p)
        self.z.rebuild()

    def test_cube(self):
        self.cube()
        self.assertAlmostEqual(self.z.hull.volume,1.)
        self.assertTrue(self.z.contains([.5,.5,.5]))
        self.assertFalse(self.z.contains([2,0,0]))
        self.assertEqual(len(self.z.hull.vertices),8)

    def test_background_buffer_faces_and_resting_border(self):
        self.cube()
        self.assertEqual(MEMBERSHIP_BUFFER_M, .005)
        for axis in range(3):
            for side in (0., 1.):
                direction = -1 if side == 0 else 1
                for amount, accepted in ((0., True), (.001, True), (.005, True), (.00501, False)):
                    p = np.full(3, .5)
                    p[axis] = side + direction * amount
                    self.assertEqual(self.z.contains(p, MEMBERSHIP_BUFFER_M), accepted)
                    if amount > 0: self.assertFalse(self.z.contains(p))

    def test_buffer_sloping_face(self):
        for p in ([0,0,0], [1,0,0], [0,1,0], [0,0,1]): self.z.add(p)
        self.z.rebuild()
        center = np.full(3, 1/3)
        normal = np.ones(3) / np.sqrt(3)
        self.assertTrue(self.z.contains(center + normal*.005, MEMBERSHIP_BUFFER_M))
        self.assertFalse(self.z.contains(center + normal*.00501, MEMBERSHIP_BUFFER_M))

    def test_buffer_does_not_modify_hull_or_saved_geometry(self):
        self.cube()
        points = np.array(self.z.points)
        planes = self.z.hull.equations.copy()
        triangles = self.z.hull.simplices.copy()
        with tempfile.TemporaryDirectory() as tmp:
            before, after = Path(tmp)/'before.json', Path(tmp)/'after.json'
            self.z.save(before)
            self.assertTrue(self.z.contains([.5,.5,-.004], MEMBERSHIP_BUFFER_M))
            self.z.save(after)
            self.assertEqual(before.read_bytes(), after.read_bytes())
            loaded = Zone.load(before, 'test')
            self.assertTrue(loaded.contains([.5,.5,-.004], MEMBERSHIP_BUFFER_M))
            self.assertFalse(loaded.contains([.5,.5,-.006], MEMBERSHIP_BUFFER_M))
        np.testing.assert_array_equal(self.z.points, points)
        np.testing.assert_array_equal(self.z.hull.equations, planes)
        np.testing.assert_array_equal(self.z.hull.simplices, triangles)

    def test_buffer_does_not_validate_bad_input_or_empty_zone(self):
        self.assertFalse(self.z.contains([0,0,0], MEMBERSHIP_BUFFER_M))
        self.cube()
        for p in ([float('nan'),0,0], [float('inf'),0,0], [0,0]):
            self.assertFalse(self.z.contains(p, MEMBERSHIP_BUFFER_M))
        for b in (-.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError): self.z.contains([0,0,0], b)

    def test_flat_and_small(self):
        for p in ([0,0,0],[1,0,0],[1,1,0],[0,1,0]): self.z.add(p)
        self.z.rebuild()
        self.assertIsNone(self.z.hull)
        self.assertFalse(self.z.contains([.5,.5,0]))

    def test_spacing_and_invalid(self):
        self.assertTrue(self.z.add([0,0,0]))
        self.assertFalse(self.z.add([.001,0,0]))
        for p in ([float('nan'),0,0],[7,0,0],[0,0]):
            with self.assertRaises(ValueError): self.z.add(p)

    def test_round_trip_and_validation(self):
        self.cube()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'zone.json'
            self.z.save(path)
            z = Zone.load(path,'test')
            np.testing.assert_allclose(z.points,self.z.points)
            self.assertAlmostEqual(z.hull.volume,1)
            with self.assertRaises(ValueError): Zone.load(path,'other')
            data = json.loads(path.read_text())
            data['planes'] = [[999]]
            path.write_text(json.dumps(data))
            self.assertAlmostEqual(Zone.load(path,'test').hull.volume,1)
            data['points'][0][0] = float('nan')
            path.write_text(json.dumps(data))
            with self.assertRaises(ValueError): Zone.load(path,'test')

    def test_empty_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'empty.json'
            self.z.save(path)
            self.assertEqual(Zone.load(path,'test').points,[])

    def test_right_zone_schema_round_trip_and_side_mismatch_rejection(self):
        z = Zone('test', RIGHT_TCP)
        for p in itertools.product((0.,1.),repeat=3): z.add(p)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'right.json';z.save(path)
            data=json.loads(path.read_text())
            self.assertEqual(data['schema'],'openarmx-right-envelope-v1')
            self.assertEqual(data['tcp'],RIGHT_TCP)
            loaded=Zone.load(path,'test',RIGHT_TCP)
            self.assertEqual(loaded.tcp,RIGHT_TCP);self.assertAlmostEqual(loaded.hull.volume,1.)
            with self.assertRaises(ValueError):Zone.load(path,'test',LEFT_TCP)

    def test_fk_left_only(self):
        model = Model(ROOT/'model/openarmx.urdf')
        zero = model.transforms({})
        self.assertIn(TCP,zero)
        moved = model.transforms({'openarmx_left_joint2':.5})
        self.assertGreater(np.linalg.norm(zero[TCP][:3,3]-moved[TCP][:3,3]),.01)
        np.testing.assert_allclose(zero['openarmx_right_hand_tcp'],moved['openarmx_right_hand_tcp'])
        for t in moved.values():
            np.testing.assert_allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-12)

    def test_fk_right_only(self):
        model=Model(ROOT/'model/openarmx.urdf');zero=model.transforms({})
        moved=model.transforms({'openarmx_right_joint2':.5})
        self.assertGreater(np.linalg.norm(zero[RIGHT_TCP][:3,3]-moved[RIGHT_TCP][:3,3]),.01)
        np.testing.assert_allclose(zero[LEFT_TCP],moved[LEFT_TCP])

if __name__ == '__main__': unittest.main()
