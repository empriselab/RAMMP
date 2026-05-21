"""Pure geometry helpers for MediaPipe-based head perception.

No ROS or MediaPipe imports — this module is independently unit-testable.
All 3D points/transforms are in meters.
"""

import numpy as np
from scipy.spatial.transform import Rotation


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a 3x3 rotation and 3-vector."""
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation).reshape(3)
    return transform


def orthonormalize(transform: np.ndarray) -> np.ndarray:
    """Return a copy of a 4x4 transform with its rotation block re-orthonormalized."""
    result = transform.copy()
    u, _, vt = np.linalg.svd(transform[:3, :3])
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1
        rotation = u @ vt
    result[:3, :3] = rotation
    return result


def matrix_to_pose(transform: np.ndarray):
    """Convert a 4x4 transform to ((x, y, z), (qx, qy, qz, qw))."""
    position = transform[:3, 3]
    quat = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return position, quat


def pose_to_matrix(position, orientation) -> np.ndarray:
    """Convert (position, quaternion) to a 4x4 transform."""
    return make_transform(
        Rotation.from_quat(orientation).as_matrix(), np.asarray(position)
    )


def kabsch_fixed_scale(target: np.ndarray, source: np.ndarray):
    """Best-fit rigid transform (rotation, translation) mapping source -> target.

    target, source: (N, 3) arrays of corresponding points. Returns (R 3x3, t 3,)
    such that target ~= R @ source + t.
    """
    assert target.shape == source.shape and target.shape[1] == 3
    target_centroid = target.mean(axis=0)
    source_centroid = source.mean(axis=0)
    rotation, _ = Rotation.align_vectors(
        target - target_centroid, source - source_centroid
    )
    rotation_matrix = rotation.as_matrix()
    translation = target_centroid - rotation_matrix @ source_centroid
    return rotation_matrix, translation


def kabsch_with_rejection(
    target: np.ndarray,
    source: np.ndarray,
    residual_threshold_m: float = 0.02,
    min_points: int = 20,
):
    """Kabsch fit with one outlier-rejection refit pass.

    Returns (R, t, keep_mask). Points whose residual exceeds the threshold are
    dropped and the fit is recomputed once, provided enough points remain.
    """
    rotation, translation = kabsch_fixed_scale(target, source)
    predicted = (rotation @ source.T).T + translation
    residuals = np.linalg.norm(target - predicted, axis=1)
    keep = residuals < residual_threshold_m
    if min_points <= int(keep.sum()) < len(keep):
        rotation, translation = kabsch_fixed_scale(target[keep], source[keep])
    return rotation, translation, keep


def backproject_landmarks(
    landmarks_px: np.ndarray,
    depth_image: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    min_depth_m: float = 0.05,
    max_depth_m: float = 1.5,
) -> np.ndarray:
    """Back-project 2D pixel landmarks to 3D camera-frame points using depth.

    landmarks_px: (N, 2) (x, y) pixel coordinates.
    depth_image: (H, W) depth in millimeters.
    Returns (N, 3) camera-frame points in meters; invalid entries are NaN.
    """
    landmarks_px = np.asarray(landmarks_px, dtype=np.float64)
    n = landmarks_px.shape[0]
    points = np.full((n, 3), np.nan, dtype=np.float64)
    height, width = depth_image.shape[:2]

    xs = np.round(landmarks_px[:, 0]).astype(int)
    ys = np.round(landmarks_px[:, 1]).astype(int)
    in_bounds = (xs >= 0) & (ys >= 0) & (xs < width) & (ys < height)

    xs_clamped = np.clip(xs, 0, width - 1)
    ys_clamped = np.clip(ys, 0, height - 1)
    depth_m = depth_image[ys_clamped, xs_clamped].astype(np.float64) / 1000.0

    valid = (
        in_bounds
        & np.isfinite(depth_m)
        & (depth_m >= min_depth_m)
        & (depth_m <= max_depth_m)
    )
    world_x = (depth_m / fx) * (xs - cx)
    world_y = (depth_m / fy) * (ys - cy)

    points[valid, 0] = world_x[valid]
    points[valid, 1] = world_y[valid]
    points[valid, 2] = depth_m[valid]
    return points


def head_frame_to_pose(head_frame: np.ndarray):
    """Convert a 4x4 head frame to (x, y, z, a, b, c) with 'yxz' Euler degrees.

    The Euler convention matches how bring_cup_to_mouth reconstructs the pose
    (Rotation.from_euler('yxz', ...)).
    """
    position = head_frame[:3, 3]
    euler = Rotation.from_matrix(head_frame[:3, :3]).as_euler("yxz", degrees=True)
    return (
        float(position[0]),
        float(position[1]),
        float(position[2]),
        float(euler[0]),
        float(euler[1]),
        float(euler[2]),
    )


def build_calibration(
    rigid_camera_points: np.ndarray,
    ee_pose_matrix: np.ndarray,
    base_to_camera: np.ndarray,
    tool_frame_to_tip_matrix: np.ndarray,
):
    """Compute the three calibration artifacts from a captured frame.

    rigid_camera_points: (N, 3) back-projected rigid landmarks (camera frame,
        meters); NaN entries allowed for landmarks without valid depth.
    ee_pose_matrix: 4x4 end-effector (tool wrist) pose in the base frame.
    base_to_camera: 4x4 transform, base frame -> camera frame.
    tool_frame_to_tip_matrix: 4x4 transform, wrist frame -> drink-tip frame.

    Returns (reference_points, reference_head_frame, tool_tip_transform):
      - reference_points: rigid_camera_points unchanged (saved as the reference).
      - reference_head_frame: 4x4, origin at the centroid of valid points,
        identity rotation, in the camera frame.
      - tool_tip_transform: 4x4 drink-tip pose in the camera frame.
    """
    centroid = np.nanmean(rigid_camera_points, axis=0)
    reference_head_frame = make_transform(np.eye(3), centroid)

    tool_tip_base = ee_pose_matrix @ tool_frame_to_tip_matrix
    camera_to_base = np.linalg.inv(base_to_camera)
    tool_tip_transform = camera_to_base @ tool_tip_base

    return rigid_camera_points, reference_head_frame, tool_tip_transform


class TransformSmoother:
    """Temporal smoother for the head-motion transform.

    Averages the last `buffer_size` transforms when the head is nearly still,
    and rejects single frames that jump implausibly far.
    """

    def __init__(
        self,
        buffer_size: int = 10,
        std_threshold_m: float = 0.005,
        jump_threshold_m: float = 0.10,
    ) -> None:
        self.buffer_size = buffer_size
        self.std_threshold_m = std_threshold_m
        self.jump_threshold_m = jump_threshold_m
        self._buffer: list[np.ndarray] = []
        self._last: np.ndarray | None = None

    def update(self, transform: np.ndarray):
        """Feed a raw 4x4 transform; return (smoothed_transform, is_noisy)."""
        if self._last is not None:
            jump = np.linalg.norm(transform[:3, 3] - self._last[:3, 3])
            if jump > self.jump_threshold_m:
                return self._last, True

        self._buffer.append(transform)
        if len(self._buffer) > self.buffer_size:
            self._buffer.pop(0)

        stacked = np.array(self._buffer)
        position_std = stacked[:, :3, 3].std(axis=0)
        if np.all(position_std < self.std_threshold_m):
            smoothed = orthonormalize(stacked.mean(axis=0))
        else:
            smoothed = transform

        self._last = smoothed
        return smoothed, False
