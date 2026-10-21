"""Board observations and eye-to-hand math; this module never opens hardware."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / 'output/pdf/openarmx_charuco_6x4_40mm.json'
URDF = ROOT / 'model/openarmx.urdf'
HAND = 'openarmx_right_hand'
JOINTS = [f'openarmx_right_joint{i}' for i in range(1, 8)]
MIN_TRAIN = 15
MIN_VALIDATION = 5


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def transform(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.asarray(t).reshape(3)
    return T


def checked_transform(value):
    T = np.asarray(value, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=2e-5)
            or abs(np.linalg.det(T[:3, :3]) - 1) > 2e-5):
        raise ValueError('Invalid rigid transform')
    return T


def inverse(T):
    return transform(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


def distance(A, B):
    return (float(np.linalg.norm(A[:3, 3] - B[:3, 3])),
            float(np.degrees(Rotation.from_matrix(A[:3, :3].T @ B[:3, :3]).magnitude())))


def camera_parameters(metadata):
    c = metadata['color']
    if (c['width'], c['height']) != (1280, 720):
        raise ValueError('Expected uncropped 1280x720 RGB; no smaller-FOV fallback')
    if (c['intrinsic_width'], c['intrinsic_height']) != (c['width'], c['height']):
        raise ValueError('Intrinsics do not match the active image size')
    K = np.asarray(c['K'], dtype=float)
    D = np.asarray(c['D_opencv_order'], dtype=float)
    if (K.shape != (3, 3) or D.shape != (8,) or not np.isfinite(K).all()
            or not np.isfinite(D).all() or K[0, 0] <= 0 or K[1, 1] <= 0):
        raise ValueError('Invalid factory camera parameters')
    # SDK ObTypes.h: 0=none, 3=Brown-Conrady, 4=Brown-Conrady K6.
    # Modified/inverse Brown and fisheye are NOT interchangeable with OpenCV.
    if c['distortion_model'] == 0:
        D = np.zeros(8)
    elif c['distortion_model'] == 3:
        D = D[:5]
    elif c['distortion_model'] != 4:
        raise ValueError(f"Unsupported distortion model {c['distortion_model']}; refusing a guessed conversion")
    checked_transform(metadata['T_color_depth_m'])
    return K, D


class Board:
    def __init__(self, path=TARGET):
        self.spec = json.loads(Path(path).read_text())
        s = self.spec
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, s['dictionary']))
        args = (s['squares_x'], s['squares_y'], s['square_length_m'], s['marker_length_m'])
        if hasattr(cv2.aruco, 'CharucoBoard_create'):
            self.board = cv2.aruco.CharucoBoard_create(*args, self.dictionary)
            points, corners, ids = self.board.objPoints, self.board.chessboardCorners, self.board.ids
            self.parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.board = cv2.aruco.CharucoBoard(args[:2], *args[2:], self.dictionary)
            self.board.setLegacyPattern(s['legacy_pattern'])
            points, corners, ids = self.board.getObjPoints(), self.board.getChessboardCorners(), self.board.getIds()
            self.parameters = cv2.aruco.DetectorParameters()
        if (not np.allclose(points, s['marker_object_points_m'], atol=1e-7)
                or not np.allclose(corners, s['charuco_corner_points_m'], atol=1e-7)
                or not np.array_equal(np.asarray(ids).ravel(), s['marker_ids'])):
            raise ValueError('OpenCV board layout differs from the supplied PDF manifest')
        self.points = np.asarray(corners, dtype=np.float64)
        self.parameters.markerBorderBits = s['marker_border_bits']

    def detect(self, image, K, D):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if hasattr(cv2.aruco, 'detectMarkers'):
            markers, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        else:
            markers, ids, _ = cv2.aruco.ArucoDetector(self.dictionary, self.parameters).detectMarkers(gray)
        if ids is None or len(ids) < 4:
            raise ValueError('Show at least four board markers')
        if hasattr(cv2.aruco, 'interpolateCornersCharuco'):
            _, corners, corner_ids = cv2.aruco.interpolateCornersCharuco(
                markers, ids, gray, self.board, cameraMatrix=K, distCoeffs=D)
        else:
            params = cv2.aruco.CharucoParameters(); params.cameraMatrix=K; params.distCoeffs=D
            corners, corner_ids, _, _ = cv2.aruco.CharucoDetector(self.board, params).detectBoard(gray)
        if corner_ids is None or len(corner_ids) < 10:
            raise ValueError('Need at least 10 of the 15 checkerboard corners')
        ids = corner_ids.ravel(); uv = corners.reshape(-1, 2).astype(np.float64)
        obj = self.points[ids]
        if np.linalg.matrix_rank(obj[:, :2] - obj[0, :2]) < 2:
            raise ValueError('Detected corners are collinear')
        if np.min(np.ptp(uv, axis=0)) < 45:
            raise ValueError('Board is too small or too edge-on in the image')
        # IPPE handles planar targets and returns both pose hypotheses.
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, uv, K, D, flags=cv2.SOLVEPNP_IPPE)
        candidates = []
        for r, t in zip(rvecs, tvecs):
            T = transform(cv2.Rodrigues(r)[0], t)
            if np.min((T[:3, :3] @ obj.T + T[:3, 3:4])[2]) <= 0:
                continue
            projected = cv2.projectPoints(obj, r, t, K, D)[0].reshape(-1, 2)
            rms = float(np.sqrt(np.mean(np.sum((projected - uv)**2, axis=1))))
            candidates.append((rms, T))
        if not count or not candidates:
            raise ValueError('No physically visible board pose')
        candidates.sort(key=lambda x: x[0]); rms, T = candidates[0]
        if rms > 1.0:
            raise ValueError(f'Board reprojection error {rms:.2f}px exceeds 1px; improve lighting/flatness')
        if len(candidates) > 1:
            other_error, other = candidates[1]
            if other_error - rms < .15 and distance(T, other)[1] > 5:
                raise ValueError('Ambiguous flat-board pose; tilt the board slightly')
        return dict(T_color_board=T.tolist(), corner_ids=ids.tolist(), corners_px=uv.tolist(),
                    reprojection_rms_px=rms, corner_count=len(ids))


def novel_pose(A, samples):
    return all((d > .02 or a > 7) for d, a in
               (distance(A, np.asarray(s['T_base_hand'])) for s in samples))


def excitation(samples):
    A = [checked_transform(s['T_base_hand']) for s in samples]
    vectors = [Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).as_rotvec()
               for i, a in enumerate(A) for b in A[i+1:]]
    if not vectors:
        raise ValueError('Not enough different poses')
    # Rotation axes expressed relative to one reference, not mixed pair frames.
    ref = [Rotation.from_matrix(A[0][:3, :3].T @ a[:3, :3]).as_rotvec() for a in A[1:]]
    singular = np.linalg.svd(np.asarray(ref), compute_uv=False) / np.sqrt(len(ref))
    angles = np.degrees(np.linalg.norm(vectors, axis=1))
    if singular[1] < np.radians(8) or max(angles) < 25:
        raise ValueError('Insufficient rotation diversity: tilt/turn the wrist about at least two different axes')
    span = np.ptp([a[:3, 3] for a in A], axis=0)
    if np.linalg.norm(span) < .08:
        raise ValueError('Insufficient position diversity: spread poses across at least 8cm')
    return dict(rotation_excitation_deg=np.degrees(singular).tolist(),
                max_relative_rotation_deg=float(max(angles)), position_span_m=span.tolist())


def _pack(T):
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def _unpack(x):
    return transform(Rotation.from_rotvec(x[:3]).as_matrix(), x[3:6])


def metrics(samples, C, X, K, D, points):
    rows = []
    for s in samples:
        A, B = checked_transform(s['T_base_hand']), checked_transform(s['T_color_board'])
        pred = inverse(C) @ A @ X
        d, a = distance(pred, B)
        obj = points[np.asarray(s['corner_ids'], dtype=int)]
        uv = np.asarray(s['corners_px'])
        z = (pred[:3, :3] @ obj.T + pred[:3, 3:4])[2]
        if np.min(z) <= 0:
            raise ValueError('Solution predicts a board behind the camera')
        image = cv2.projectPoints(obj, Rotation.from_matrix(pred[:3, :3]).as_rotvec(), pred[:3, 3], K, D)[0].reshape(-1, 2)
        px = float(np.sqrt(np.mean(np.sum((image-uv)**2, axis=1))))
        rows.append(dict(sample=s['id'], translation_mm=d*1000, rotation_deg=a, reprojection_px=px))
    return aggregate_metrics(rows)


def aggregate_metrics(rows):
    """Aggregate per-pose residuals, including out-of-fold predictions."""
    if not rows:
        raise ValueError('No poses to evaluate')
    out = {'samples': rows}
    for key in ('translation_mm', 'rotation_deg', 'reprojection_px'):
        values = np.array([r[key] for r in rows])
        out[key] = dict(rms=float(np.sqrt(np.mean(values**2))), maximum=float(np.max(values)))
    return out


def passes(m):
    return (m['translation_mm']['rms'] <= 5 and m['translation_mm']['maximum'] <= 10
            and m['rotation_deg']['rms'] <= 1 and m['rotation_deg']['maximum'] <= 2
            and m['reprojection_px']['rms'] <= 4 and m['reprojection_px']['maximum'] <= 8)


def solve(training, K, D, board):
    if len(training) < MIN_TRAIN:
        raise ValueError(f'Collect at least {MIN_TRAIN} training poses first')
    coverage = excitation(training)
    A = [checked_transform(s['T_base_hand']) for s in training]
    B = [checked_transform(s['T_color_board']) for s in training]
    Ai = [inverse(a) for a in A]
    # Eye-to-hand: invert FK input; output is BASE <- CAMERA, not HAND <- CAMERA.
    R, t = cv2.calibrateHandEye([a[:3, :3] for a in Ai], [a[:3, 3] for a in Ai],
                              [b[:3, :3] for b in B], [b[:3, 3] for b in B],
                              method=cv2.CALIB_HAND_EYE_PARK)
    C = checked_transform(transform(R, t))
    mounts = [inverse(a) @ C @ b for a, b in zip(A, B)]
    X = transform(Rotation.from_matrix(np.array([x[:3, :3] for x in mounts])).mean().as_matrix(),
                  np.mean([x[:3, 3] for x in mounts], axis=0))

    def residual(p):
        c, x = _unpack(p[:6]), _unpack(p[6:])
        result = []
        for a, b in zip(A, B):
            lhs, rhs = a @ x, c @ b
            result.extend((lhs[:3, 3] - rhs[:3, 3]) / .005)
            result.extend(Rotation.from_matrix(lhs[:3, :3].T @ rhs[:3, :3]).as_rotvec() / np.radians(1))
        return result

    fit = least_squares(residual, np.r_[_pack(C), _pack(X)], loss='soft_l1', max_nfev=500)
    if not fit.success or not np.isfinite(fit.x).all():
        raise ValueError('Calibration optimization did not converge')
    C, X = _unpack(fit.x[:6]), _unpack(fit.x[6:])
    m = metrics(training, C, X, K, D, board.points)
    # No silently discarded outliers. All captured training poses are reported.
    return dict(T_base_color=C.tolist(), T_hand_board=X.tolist(), coverage=coverage,
                training_metrics=m, training_passed=passes(m), training_ids=[s['id'] for s in training],
                method='OpenCV Park eye-to-hand + joint rigid-transform soft-L1 refinement')
