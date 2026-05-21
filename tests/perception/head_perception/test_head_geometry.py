import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rammp.perception.head_perception import head_geometry as hg


def _random_points(n=50, seed=0):
    rng = np.random.default_rng(seed)
    return rng.uniform(-0.1, 0.1, size=(n, 3))


def test_make_transform_shape_and_content():
    R = Rotation.from_euler("xyz", [10, 20, 30], degrees=True).as_matrix()
    t = np.array([0.1, 0.2, 0.3])
    T = hg.make_transform(R, t)
    assert T.shape == (4, 4)
    np.testing.assert_allclose(T[:3, :3], R)
    np.testing.assert_allclose(T[:3, 3], t)
    np.testing.assert_allclose(T[3], [0, 0, 0, 1])


def test_kabsch_fixed_scale_recovers_known_transform():
    source = _random_points()
    R_true = Rotation.from_euler("xyz", [5, -15, 25], degrees=True).as_matrix()
    t_true = np.array([0.05, -0.02, 0.3])
    target = (R_true @ source.T).T + t_true
    R, t = hg.kabsch_fixed_scale(target, source)
    np.testing.assert_allclose(R, R_true, atol=1e-6)
    np.testing.assert_allclose(t, t_true, atol=1e-6)


def test_kabsch_with_rejection_ignores_outliers():
    source = _random_points(n=60)
    R_true = Rotation.from_euler("xyz", [3, 8, -4], degrees=True).as_matrix()
    t_true = np.array([0.0, 0.1, 0.25])
    target = (R_true @ source.T).T + t_true
    target[5] += np.array([0.5, 0.5, 0.5])  # gross outlier
    R, t, keep = hg.kabsch_with_rejection(target, source)
    assert keep[5] == False  # noqa: E712
    np.testing.assert_allclose(R, R_true, atol=1e-4)
    np.testing.assert_allclose(t, t_true, atol=1e-4)


def test_backproject_landmarks_basic():
    depth = np.full((20, 20), 500.0, dtype=np.float32)  # 0.5 m everywhere, in mm
    landmarks = np.array([[10.0, 10.0], [5.0, 8.0]])
    pts = hg.backproject_landmarks(landmarks, depth, fx=600, fy=600, cx=10, cy=10)
    assert pts.shape == (2, 3)
    np.testing.assert_allclose(pts[0], [0.0, 0.0, 0.5], atol=1e-9)
    assert pts[1, 2] == pytest.approx(0.5)


def test_backproject_landmarks_invalid_depth_is_nan():
    depth = np.zeros((20, 20), dtype=np.float32)  # 0 m -> out of range
    landmarks = np.array([[10.0, 10.0]])
    pts = hg.backproject_landmarks(landmarks, depth, fx=600, fy=600, cx=10, cy=10)
    assert np.all(np.isnan(pts[0]))


def test_backproject_landmarks_out_of_bounds_is_nan():
    depth = np.full((20, 20), 500.0, dtype=np.float32)
    landmarks = np.array([[999.0, 5.0]])
    pts = hg.backproject_landmarks(landmarks, depth, fx=600, fy=600, cx=10, cy=10)
    assert np.all(np.isnan(pts[0]))


def test_matrix_pose_roundtrip():
    R = Rotation.from_euler("xyz", [12, -34, 56], degrees=True).as_matrix()
    T = hg.make_transform(R, np.array([0.1, 0.2, 0.3]))
    pos, quat = hg.matrix_to_pose(T)
    T2 = hg.pose_to_matrix(pos, quat)
    np.testing.assert_allclose(T, T2, atol=1e-9)


def test_transform_smoother_averages_when_stable():
    smoother = hg.TransformSmoother(buffer_size=5, std_threshold_m=0.01)
    base = hg.make_transform(np.eye(3), np.array([0.0, 0.0, 0.5]))
    out = None
    for _ in range(5):
        out, noisy = smoother.update(base.copy())
        assert noisy is False
    np.testing.assert_allclose(out[:3, 3], [0.0, 0.0, 0.5], atol=1e-6)


def test_transform_smoother_rejects_large_jump():
    smoother = hg.TransformSmoother(jump_threshold_m=0.1)
    base = hg.make_transform(np.eye(3), np.array([0.0, 0.0, 0.5]))
    smoother.update(base.copy())
    jumped = hg.make_transform(np.eye(3), np.array([0.0, 0.0, 1.0]))
    out, noisy = smoother.update(jumped)
    assert noisy is True
    np.testing.assert_allclose(out[:3, 3], [0.0, 0.0, 0.5], atol=1e-6)


def test_build_calibration_outputs():
    rigid = np.array([[0.0, 0.0, 0.5], [0.02, 0.0, 0.5], [0.0, 0.02, 0.5]])
    ee_pose = hg.make_transform(np.eye(3), np.array([0.4, 0.0, 0.3]))
    base_to_camera = hg.make_transform(np.eye(3), np.array([0.1, 0.0, 0.2]))
    tool_to_tip = hg.make_transform(np.eye(3), np.array([0.0, 0.0, 0.1]))
    ref_pts, ref_frame, tip_tf = hg.build_calibration(
        rigid, ee_pose, base_to_camera, tool_to_tip
    )
    np.testing.assert_allclose(ref_pts, rigid)
    np.testing.assert_allclose(ref_frame[:3, 3], rigid.mean(axis=0))
    np.testing.assert_allclose(tip_tf[:3, 3], [0.3, 0.0, 0.2], atol=1e-9)


def test_backproject_landmarks_offcenter():
    depth = np.full((480, 640), 1000.0, dtype=np.float32)  # 1.0 m, in mm
    pts = hg.backproject_landmarks(
        np.array([[330.0, 250.0]]), depth, fx=500.0, fy=500.0, cx=320.0, cy=240.0
    )
    # X = (1.0 / 500) * (330 - 320) = 0.02 ; Y = (1.0 / 500) * (250 - 240) = 0.02
    np.testing.assert_allclose(pts[0], [0.02, 0.02, 1.0], atol=1e-9)


def test_head_frame_to_pose_roundtrips_yxz_euler():
    rotation = Rotation.from_euler("yxz", [15.0, -10.0, 5.0], degrees=True).as_matrix()
    head_frame = hg.make_transform(rotation, np.array([1.0, 2.0, 3.0]))
    pose = hg.head_frame_to_pose(head_frame)
    np.testing.assert_allclose(pose[:3], [1.0, 2.0, 3.0], atol=1e-9)
    np.testing.assert_allclose(pose[3:], [15.0, -10.0, 5.0], atol=1e-6)
