# MediaPipe Head Perception Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the DECA/FLAME head-perception stack with a lightweight MediaPipe Face Landmarker pipeline that runs on a Jetson, while producing the same outputs the rest of the system consumes.

**Architecture:** MediaPipe Face Landmarker detects 478 face landmarks + a `jawOpen` blendshape per RGB frame. A rigid subset of those landmarks (face minus lips/eyes/eyebrows/irises) is back-projected through the RealSense depth image into 3D camera-frame points. A single Kabsch fit between the current points and a one-time calibration reference yields `trans` — how much the head moved since calibration. The cup target is the calibrated tool-tip pose shifted by `trans`. This mirrors the existing relative-tracking design but swaps the 5023-vertex FLAME reconstruction for a tiny MediaPipe model. A new `--calibrate-head` mode records the reference.

**Tech Stack:** Python 3.10, MediaPipe Tasks (`mediapipe`), NumPy, SciPy, OpenCV, ROS 2 Humble (`rclpy`), pytest.

---

## Background & Context (read before starting)

**The existing pipeline being replaced** lives in `src/rammp/perception/head_perception/deca_perception.py` (`HeadPerception` class). It is consumed via `src/rammp/interfaces/perception_interface.py` (`run_head_perception`), which calls `HeadPerception.run_deca(...)` and repackages the result into a dict with keys `head_pose`, `face_keypoints`, `tool_tip_target_pose`, `camera_color_data`.

**Downstream consumers of that dict** (verified — these are the only ones):
- `src/rammp/actions/bring_cup_to_mouth.py:36-46` — uses `tool_tip_target_pose` (only its **translation**; the rotation is overwritten at line 49 with a hardcoded quaternion) and `head_pose` (passed to `sim.set_head_pose` — visualization only).
- `src/rammp/perception/gestures_perception/static_gesture_detectors.py` — `mouth_open` uses `face_keypoints` for a mouth-aspect-ratio threshold.

**The relative-tracking idea:** the system never detects an absolute mouth location. It is calibrated once (person + cup in a known-good pose), and each frame measures how far the head moved from that calibration and shifts the cup target by the same amount.

**Calibration files** (today, DECA-specific, in `src/rammp/perception/head_perception/config/drink/`): `head_points.npy`, `reference_neck_frame.npy`, `tool_tip_transform.npy`, plus shared `config/fixed_model_head_points.npy`. These are FLAME-vertex-specific and cannot be reused. The new pipeline stores its own files under a new directory and a new `--calibrate-head` mode regenerates them.

**Environment notes for the implementer:**
- The development machine has Python 3.12 with `numpy`, `scipy`, `opencv-python` (cv2 4.6) available, but **not** `rclpy`, `pybullet_helpers`, or the `rammp` package installed. The production runtime is a conda env on Python 3.10 + ROS 2 Humble.
- Therefore: **pure-math modules** (`head_geometry.py`, `landmark_indices.py`) and `mouth_open` get real pytest TDD that runs on the dev machine. **ROS-integrated modules** (`mediapipe_perception.py` wrapper, `perception_interface.py`, `calibrate_head.py`) cannot be executed on the dev machine — for those, the verification step is `python -m py_compile` plus careful review; runtime verification happens on the target hardware.
- `mediapipe` must be `pip install`-ed (Task 1) for `landmark_indices.py` tests to run.

**Camera data format** (`RealSenseInterface.get_camera_data()` returns a dict):
- `rgb_image`: OpenCV image, **BGR**, uint8, 1280×720.
- `depth_image`: float32, single channel, depth in **millimeters**.
- `camera_info`: ROS `CameraInfo` message; intrinsics in `.k` (9-element row-major): `fx=k[0]`, `fy=k[4]`, `cx=k[2]`, `cy=k[5]`.
- `base_to_camera`: 4×4 NumPy transform from `RealSenseInterface.get_base_to_camera_transform()`.

---

## File Structure

**Create:**
- `tests/conftest.py` — puts `src/` on `sys.path` for tests.
- `src/rammp/perception/head_perception/head_geometry.py` — pure geometry math (Kabsch, back-projection, transform/pose conversions, temporal smoother, calibration builder).
- `tests/perception/head_perception/test_head_geometry.py` — tests for the above.
- `src/rammp/perception/head_perception/landmark_indices.py` — the rigid MediaPipe landmark index subset + jaw blendshape name.
- `tests/perception/head_perception/test_landmark_indices.py` — tests for the above.
- `src/rammp/perception/head_perception/mediapipe_perception.py` — `MediaPipeHeadPerception` class (the new `HeadPerception`).
- `src/rammp/perception/head_perception/calibrate_head.py` — ROS node implementing `--calibrate-head` mode.
- `src/rammp/perception/head_perception/models/face_landmarker.task` — bundled MediaPipe model (binary, committed).
- `tests/perception/gestures_perception/test_mouth_open.py` — tests for the rewritten `mouth_open`.

**Modify:**
- `src/rammp/interfaces/perception_interface.py` — import + construct `MediaPipeHeadPerception` instead of `HeadPerception`; pass `jaw_open_score` through.
- `src/rammp/perception/gestures_perception/static_gesture_detectors.py` — `mouth_open` reads `jaw_open_score`.
- `pyproject.toml` — add `mediapipe`; remove DECA-only deps.
- `README.md` — replace DECA setup with MediaPipe setup; document `--calibrate-head`.

**Left untouched (legacy):** `deca_perception.py` and the `DECA/` directory remain in the repo but are no longer imported. Removing them is out of scope.

---

## Task 1: Environment setup and MediaPipe model asset

**Files:**
- Create: `src/rammp/perception/head_perception/models/face_landmarker.task` (downloaded binary)
- Modify: `pyproject.toml`

- [ ] **Step 1: Install MediaPipe**

Run: `pip install mediapipe`
Expected: installs successfully. Verify: `python -c "import mediapipe; from mediapipe.tasks.python import vision; print(mediapipe.__version__)"` prints a version (0.10.x or newer) with no error.

If the install fails on the dev machine, report status BLOCKED with the error — do not proceed.

- [ ] **Step 2: Download the Face Landmarker model**

```bash
mkdir -p src/rammp/perception/head_perception/models
curl -L -o src/rammp/perception/head_perception/models/face_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task
```

Verify the file is ~3–4 MB: `ls -l src/rammp/perception/head_perception/models/face_landmarker.task`

- [ ] **Step 3: Smoke-test the model loads**

Run:
```bash
python -c "
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
opts = vision.FaceLandmarkerOptions(
    base_options=python.BaseOptions(model_asset_path='src/rammp/perception/head_perception/models/face_landmarker.task'),
    output_face_blendshapes=True,
    output_facial_transformation_matrixes=True,
    num_faces=1,
    running_mode=vision.RunningMode.IMAGE,
)
det = vision.FaceLandmarker.create_from_options(opts)
print('FaceLandmarker created OK')
"
```
Expected: prints `FaceLandmarker created OK`.

- [ ] **Step 4: Update `pyproject.toml` dependencies**

In the `full` optional-dependency list: add `"mediapipe"`. Remove the DECA-only entries `"kornia"`, `"face-alignment"`, `"scikit-image"`, and `"yacs"`. Leave `"open3d"`, `"scikit-learn"`, `"opencv-python"`, `"numpy"` (still used by cup detection / general code).

- [ ] **Step 5: Commit**

```bash
git add src/rammp/perception/head_perception/models/face_landmarker.task pyproject.toml
git commit -m "Add MediaPipe dependency and Face Landmarker model"
```

---

## Task 2: Head geometry math module

**Files:**
- Create: `tests/conftest.py`
- Create: `src/rammp/perception/head_perception/head_geometry.py`
- Test: `tests/perception/head_perception/test_head_geometry.py`

This module is pure NumPy/SciPy — no ROS, no MediaPipe. It is the testable core of the new pipeline.

- [ ] **Step 1: Create the test path shim**

Create `tests/conftest.py`:
```python
"""Ensure the src/ layout package is importable in tests."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
```

- [ ] **Step 2: Write the failing tests**

Create `tests/perception/head_perception/test_head_geometry.py`:
```python
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
    # tip in base = ee_pose @ tool_to_tip -> translation (0.4, 0.0, 0.4)
    # tip in camera = inv(base_to_camera) @ that -> (0.3, 0.0, 0.2)
    np.testing.assert_allclose(tip_tf[:3, 3], [0.3, 0.0, 0.2], atol=1e-9)
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `pytest tests/perception/head_perception/test_head_geometry.py -v`
Expected: FAIL — `ModuleNotFoundError` or `AttributeError` (module not yet created).

- [ ] **Step 4: Implement `head_geometry.py`**

Create `src/rammp/perception/head_perception/head_geometry.py`:
```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pytest tests/perception/head_perception/test_head_geometry.py -v`
Expected: PASS — all 10 tests.

- [ ] **Step 6: Commit**

```bash
git add tests/conftest.py src/rammp/perception/head_perception/head_geometry.py tests/perception/head_perception/test_head_geometry.py
git commit -m "Add head geometry math module for MediaPipe head perception"
```

---

## Task 3: Rigid landmark index subset

**Files:**
- Create: `src/rammp/perception/head_perception/landmark_indices.py`
- Test: `tests/perception/head_perception/test_landmark_indices.py`

The MediaPipe face mesh has 468 base landmarks (+10 iris landmarks = 478 total). For head-pose tracking we want only the **rigid** part of the face — exclude lips, eyes, eyebrows, and irises, because those move with expression (e.g. when the person opens their mouth) and would corrupt the pose fit.

- [ ] **Step 1: Write the failing tests**

Create `tests/perception/head_perception/test_landmark_indices.py`:
```python
from rammp.perception.head_perception import landmark_indices as li


def test_rigid_indices_are_sorted_unique_in_range():
    idx = li.RIGID_LANDMARK_INDICES
    assert idx == sorted(idx)
    assert len(idx) == len(set(idx))
    assert all(0 <= i < 468 for i in idx)


def test_rigid_indices_substantial_count():
    # After removing lips/eyes/eyebrows/irises, a large rigid set should remain.
    assert len(li.RIGID_LANDMARK_INDICES) > 200


def test_rigid_indices_exclude_lip_landmarks():
    # Landmarks 13 and 14 are the inner upper/lower lip centers.
    assert 13 not in li.RIGID_LANDMARK_INDICES
    assert 14 not in li.RIGID_LANDMARK_INDICES


def test_jaw_blendshape_name_constant():
    assert li.JAW_OPEN_BLENDSHAPE == "jawOpen"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/perception/head_perception/test_landmark_indices.py -v`
Expected: FAIL — `ModuleNotFoundError`.

- [ ] **Step 3: Implement `landmark_indices.py`**

Create `src/rammp/perception/head_perception/landmark_indices.py`:
```python
"""Selection of MediaPipe face-mesh landmarks used for rigid head tracking.

The non-rigid regions (lips, eyes, eyebrows, irises) move with facial
expression and are excluded so that, e.g., opening the mouth does not shift
the estimated head pose.
"""

from mediapipe.python.solutions import face_mesh_connections as _fmc

# Name of the blendshape that scores how open the jaw/mouth is (0..1).
JAW_OPEN_BLENDSHAPE = "jawOpen"

# Number of base (non-iris) face-mesh landmarks.
_NUM_BASE_LANDMARKS = 468


def _indices_from_connections(connections) -> set[int]:
    """Flatten a set of (start, end) landmark-index pairs into a set of indices."""
    indices: set[int] = set()
    for start, end in connections:
        indices.add(start)
        indices.add(end)
    return indices


_NON_RIGID_INDICES: set[int] = (
    _indices_from_connections(_fmc.FACEMESH_LIPS)
    | _indices_from_connections(_fmc.FACEMESH_LEFT_EYE)
    | _indices_from_connections(_fmc.FACEMESH_RIGHT_EYE)
    | _indices_from_connections(_fmc.FACEMESH_LEFT_EYEBROW)
    | _indices_from_connections(_fmc.FACEMESH_RIGHT_EYEBROW)
    | _indices_from_connections(_fmc.FACEMESH_LEFT_IRIS)
    | _indices_from_connections(_fmc.FACEMESH_RIGHT_IRIS)
)

# Sorted list of rigid landmark indices used for head-pose estimation.
RIGID_LANDMARK_INDICES: list[int] = sorted(
    set(range(_NUM_BASE_LANDMARKS)) - _NON_RIGID_INDICES
)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/perception/head_perception/test_landmark_indices.py -v`
Expected: PASS — all 4 tests.

- [ ] **Step 5: Commit**

```bash
git add src/rammp/perception/head_perception/landmark_indices.py tests/perception/head_perception/test_landmark_indices.py
git commit -m "Add rigid MediaPipe landmark index selection"
```

---

## Task 4: MediaPipeHeadPerception class

**Files:**
- Create: `src/rammp/perception/head_perception/mediapipe_perception.py`

This is the new `HeadPerception` equivalent. It is thin glue: detect → back-project → call `head_geometry` → assemble the output dict. It cannot be unit-tested on the dev machine (needs a camera + a face); verification is `py_compile` + review + on-hardware testing.

- [ ] **Step 1: Implement `mediapipe_perception.py`**

Create `src/rammp/perception/head_perception/mediapipe_perception.py`:
```python
"""MediaPipe-based head perception — replacement for the DECA pipeline.

Detects face landmarks with MediaPipe Face Landmarker, back-projects a rigid
subset through the depth image, and tracks head motion relative to a one-time
calibration to produce a cup-tip target pose.
"""

import os

import cv2
import numpy as np
from mediapipe import Image, ImageFormat
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

from rammp.perception.head_perception import head_geometry as hg
from rammp.perception.head_perception.landmark_indices import (
    JAW_OPEN_BLENDSHAPE,
    RIGID_LANDMARK_INDICES,
)
from rammp.utils.timing import timer

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_MODEL_PATH = os.path.join(_THIS_DIR, "models", "face_landmarker.task")
_CONFIG_DIR = os.path.join(_THIS_DIR, "mediapipe_config")

# Minimum number of rigid landmarks with valid depth required to trust a frame.
_MIN_VALID_LANDMARKS = 60


class MediaPipeHeadPerception:
    """Head-pose / cup-target perception backed by MediaPipe Face Landmarker."""

    def __init__(self, model_path: str | None = None) -> None:
        model_path = model_path or _DEFAULT_MODEL_PATH
        options = mp_vision.FaceLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            output_face_blendshapes=True,
            output_facial_transformation_matrixes=False,
            num_faces=1,
            running_mode=mp_vision.RunningMode.IMAGE,
        )
        self._detector = mp_vision.FaceLandmarker.create_from_options(options)

        self.tool: str | None = None
        self._rigid_indices = np.array(RIGID_LANDMARK_INDICES, dtype=int)
        self._smoother = hg.TransformSmoother()

        self.reference_points: np.ndarray | None = None
        self.reference_head_frame: np.ndarray | None = None
        self.tool_tip_transform: np.ndarray | None = None

    def set_tool(self, tool: str) -> None:
        """Load the calibration files recorded for the given tool."""
        self.tool = tool
        self._smoother = hg.TransformSmoother()
        tool_dir = os.path.join(_CONFIG_DIR, tool)
        required = {
            "reference_points": "reference_landmarks_camera.npy",
            "reference_head_frame": "reference_head_frame.npy",
            "tool_tip_transform": "tool_tip_transform.npy",
        }
        missing = [
            name
            for name in required.values()
            if not os.path.exists(os.path.join(tool_dir, name))
        ]
        if missing:
            raise FileNotFoundError(
                f"Head-perception calibration for tool '{tool}' is missing "
                f"{missing} in {tool_dir}. Run the calibration first:\n"
                f"  python -m rammp.perception.head_perception.calibrate_head "
                f"--tool {tool}"
            )
        self.reference_points = np.load(
            os.path.join(tool_dir, required["reference_points"])
        )
        self.reference_head_frame = np.load(
            os.path.join(tool_dir, required["reference_head_frame"])
        )
        self.tool_tip_transform = np.load(
            os.path.join(tool_dir, required["tool_tip_transform"])
        )

    def detect_landmarks(self, bgr_image: np.ndarray):
        """Run MediaPipe on a BGR image.

        Returns (landmarks_px (478, 2), jaw_open_score float) or None if no
        face was detected.
        """
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        mp_image = Image(image_format=ImageFormat.SRGB, data=rgb_image)
        with timer("head/mediapipe_detect"):
            result = self._detector.detect(mp_image)
        if not result.face_landmarks:
            return None

        height, width = bgr_image.shape[:2]
        landmarks = result.face_landmarks[0]
        landmarks_px = np.array(
            [[lm.x * width, lm.y * height] for lm in landmarks], dtype=np.float64
        )

        jaw_open_score = 0.0
        if result.face_blendshapes:
            for category in result.face_blendshapes[0]:
                if category.category_name == JAW_OPEN_BLENDSHAPE:
                    jaw_open_score = float(category.score)
                    break

        return landmarks_px, jaw_open_score

    def rigid_landmark_points(self, bgr_image, depth_image, camera_info):
        """Back-project the rigid landmark subset to camera-frame 3D points.

        Returns (rigid_points (N, 3) with NaN for invalid, landmarks_px,
        jaw_open_score) or None if no face was detected.
        """
        detection = self.detect_landmarks(bgr_image)
        if detection is None:
            return None
        landmarks_px, jaw_open_score = detection

        rigid_px = landmarks_px[self._rigid_indices]
        rigid_points = hg.backproject_landmarks(
            rigid_px,
            depth_image,
            fx=camera_info.k[0],
            fy=camera_info.k[4],
            cx=camera_info.k[2],
            cy=camera_info.k[5],
        )
        return rigid_points, landmarks_px, jaw_open_score

    def run(self, rgb_image, camera_info, depth_image, base_to_camera):
        """Run one head-perception cycle.

        Returns a dict with keys head_pose, tool_tip_target_pose, landmarks2d,
        jaw_open_score, noisy_reading — or None if perception failed this frame.
        """
        if self.reference_points is None:
            raise RuntimeError(
                "MediaPipeHeadPerception.set_tool() must be called before run()."
            )
        if base_to_camera is None:
            return None

        result = self.rigid_landmark_points(rgb_image, depth_image, camera_info)
        if result is None:
            return None
        rigid_points, landmarks_px, jaw_open_score = result

        valid = ~np.isnan(rigid_points).any(axis=1) & ~np.isnan(
            self.reference_points
        ).any(axis=1)
        if int(valid.sum()) < _MIN_VALID_LANDMARKS:
            return None

        rotation, translation, _ = hg.kabsch_with_rejection(
            rigid_points[valid], self.reference_points[valid]
        )
        trans = hg.make_transform(rotation, translation)
        trans, noisy_reading = self._smoother.update(trans)

        tool_tip_target_camera = trans @ self.tool_tip_transform
        tool_tip_target_base = base_to_camera @ tool_tip_target_camera

        head_frame_camera = trans @ self.reference_head_frame
        head_frame_base = base_to_camera @ head_frame_camera

        return {
            "head_pose": hg.head_frame_to_pose(head_frame_base),
            "tool_tip_target_pose": tool_tip_target_base,
            "landmarks2d": landmarks_px,
            "jaw_open_score": jaw_open_score,
            "noisy_reading": noisy_reading,
        }
```

- [ ] **Step 2: Verify it compiles**

Run: `python -m py_compile src/rammp/perception/head_perception/mediapipe_perception.py`
Expected: no output, exit code 0.

- [ ] **Step 3: Commit**

```bash
git add src/rammp/perception/head_perception/mediapipe_perception.py
git commit -m "Add MediaPipeHeadPerception class"
```

---

## Task 5: Wire MediaPipe perception into PerceptionInterface

**Files:**
- Modify: `src/rammp/interfaces/perception_interface.py`

- [ ] **Step 1: Swap the import**

In `src/rammp/interfaces/perception_interface.py`, replace:
```python
from rammp.perception.head_perception.deca_perception import HeadPerception
```
with:
```python
from rammp.perception.head_perception.mediapipe_perception import MediaPipeHeadPerception
```

- [ ] **Step 2: Swap the constructor**

In `__init__`, replace:
```python
            self._head_perception = HeadPerception()
```
with:
```python
            self._head_perception = MediaPipeHeadPerception()
```

- [ ] **Step 3: Swap the run call and pass jaw_open_score through**

In `run_head_perception`, replace the `run_deca(...)` call block:
```python
        with timer("head/run_head_perception_total"):
            head_perception_data = self._head_perception.run_deca(
                camera_data["rgb_image"],
                camera_data["camera_info"],
                camera_data["depth_image"],
                base_to_camera,
                debug_print=False,
                visualize=False,
                filter_noisy_readings=False,
            )
```
with:
```python
        with timer("head/run_head_perception_total"):
            head_perception_data = self._head_perception.run(
                camera_data["rgb_image"],
                camera_data["camera_info"],
                camera_data["depth_image"],
                base_to_camera,
            )
```

And replace the result-repackaging block:
```python
        if head_perception_data is not None:
            head_perception_data = {
                "head_pose": head_perception_data["head_pose"],
                "face_keypoints": head_perception_data["landmarks2d"],
                "tool_tip_target_pose": head_perception_data["tool_tip_target_pose"],
                "camera_color_data": camera_data["rgb_image"],
            }
```
with:
```python
        if head_perception_data is not None:
            head_perception_data = {
                "head_pose": head_perception_data["head_pose"],
                "face_keypoints": head_perception_data["landmarks2d"],
                "tool_tip_target_pose": head_perception_data["tool_tip_target_pose"],
                "jaw_open_score": head_perception_data["jaw_open_score"],
                "camera_color_data": camera_data["rgb_image"],
            }
```

- [ ] **Step 4: Verify it compiles**

Run: `python -m py_compile src/rammp/interfaces/perception_interface.py`
Expected: no output, exit code 0.

- [ ] **Step 5: Commit**

```bash
git add src/rammp/interfaces/perception_interface.py
git commit -m "Wire MediaPipe head perception into PerceptionInterface"
```

---

## Task 6: Rewrite mouth_open to use the jaw blendshape

**Files:**
- Modify: `src/rammp/perception/gestures_perception/static_gesture_detectors.py`
- Test: `tests/perception/gestures_perception/test_mouth_open.py`

The old `mouth_open` computed a mouth-aspect-ratio from 68 landmarks. MediaPipe gives a `jawOpen` blendshape directly, already surfaced as `jaw_open_score` in the head-perception dict.

- [ ] **Step 1: Write the failing tests**

Create `tests/perception/gestures_perception/test_mouth_open.py`:
```python
import threading

from rammp.perception.gestures_perception.static_gesture_detectors import mouth_open


class _FakePerception:
    """Yields a preset sequence of head-perception dicts on each call."""

    def __init__(self, sequence):
        self._sequence = list(sequence)
        self._calls = 0

    def run_head_perception(self):
        if self._calls < len(self._sequence):
            value = self._sequence[self._calls]
        else:
            value = self._sequence[-1]
        self._calls += 1
        return value


def test_mouth_open_returns_true_when_jaw_open():
    perception = _FakePerception([{"jaw_open_score": 0.9}])
    assert mouth_open(perception, termination_event=None, timeout=5) is True


def test_mouth_open_skips_none_then_detects():
    perception = _FakePerception([None, None, {"jaw_open_score": 0.8}])
    assert mouth_open(perception, termination_event=None, timeout=5) is True


def test_mouth_open_returns_false_when_terminated():
    event = threading.Event()
    event.set()
    perception = _FakePerception([{"jaw_open_score": 0.0}])
    assert mouth_open(perception, termination_event=event, timeout=5) is False


def test_mouth_open_returns_false_on_timeout():
    perception = _FakePerception([{"jaw_open_score": 0.1}])
    assert mouth_open(perception, termination_event=None, timeout=0.2) is False
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/perception/gestures_perception/test_mouth_open.py -v`
Expected: FAIL — current `mouth_open` reads `face_keypoints` and raises `KeyError`/`TypeError`.

- [ ] **Step 3: Rewrite `mouth_open`**

Replace the entire body of `src/rammp/perception/gestures_perception/static_gesture_detectors.py` with:
```python
import time

function_name_to_label = {
    "mouth_open": "mouth open",
    "head_nod": "head nod",
}

# jawOpen blendshape score above which the mouth is considered open.
MOUTH_OPEN_THRESHOLD = 0.4


def mouth_open(perception_interface, termination_event, timeout):
    """Block until the user opens their mouth, the timeout elapses, or cancel.

    Returns True if a mouth-open gesture was detected, False otherwise.
    """
    start_time = time.time()
    while time.time() - start_time < timeout and (
        termination_event is None or not termination_event.is_set()
    ):
        head_perception_data = perception_interface.run_head_perception()
        if head_perception_data is None:
            continue
        if head_perception_data["jaw_open_score"] > MOUTH_OPEN_THRESHOLD:
            return True
    return False
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/perception/gestures_perception/test_mouth_open.py -v`
Expected: PASS — all 4 tests.

- [ ] **Step 5: Commit**

```bash
git add src/rammp/perception/gestures_perception/static_gesture_detectors.py tests/perception/gestures_perception/test_mouth_open.py
git commit -m "Rewrite mouth_open to use MediaPipe jawOpen blendshape"
```

---

## Task 7: Head calibration mode

**Files:**
- Create: `src/rammp/perception/head_perception/calibrate_head.py`

A standalone ROS node that records the three calibration files. It reuses `RealSenseInterface` (camera) and `ArmInterfaceClient` (end-effector pose). It cannot be run on the dev machine (needs ROS + hardware); verification is `py_compile` + review.

The operator workflow: position the user comfortably, physically hold the cup/tool tip at the user's mouth, then press Enter to capture.

- [ ] **Step 1: Implement `calibrate_head.py`**

Create `src/rammp/perception/head_perception/calibrate_head.py`:
```python
"""Head-perception calibration node.

Records the reference the runtime head perception tracks against:
positions the user comfortably, holds the drink tip at their mouth, and
captures one frame. Run with:

    python -m rammp.perception.head_perception.calibrate_head --tool drink
"""

import argparse
import os

import numpy as np
import rclpy
from rclpy.node import Node

import rammp.simulation.scene_description as scene_description_module
from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.interfaces.realsense_interface import RealSenseInterface
from rammp.perception.head_perception import head_geometry as hg
from rammp.perception.head_perception.mediapipe_perception import (
    MediaPipeHeadPerception,
    _CONFIG_DIR,
)


def _wait_for_camera(node: Node, realsense: RealSenseInterface) -> dict:
    node.get_logger().info("Waiting for camera data...")
    while realsense.get_camera_data()["rgb_image"] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    return realsense.get_camera_data()


def calibrate(tool: str, scene_config: str) -> None:
    rclpy.init()
    node = Node("head_calibration")

    realsense = RealSenseInterface(node)
    arm = ArmInterfaceClient(node=node)
    head_perception = MediaPipeHeadPerception()

    scene_config_path = os.path.join(
        os.path.dirname(scene_description_module.__file__),
        "configs",
        f"{scene_config}.yaml",
    )
    scene_description = scene_description_module.create_scene_description_from_config(
        scene_config_path
    )
    tool_frame_to_tip = scene_description.tool_frame_to_drink_tip.to_matrix()

    _wait_for_camera(node, realsense)

    input(
        f"\nPosition the user comfortably and hold the {tool} tip at their "
        f"mouth.\nPress Enter to capture the calibration frame..."
    )

    camera_data = realsense.get_camera_data()
    base_to_camera = realsense.get_base_to_camera_transform()
    if base_to_camera is None:
        node.get_logger().error("base->camera transform unavailable; aborting.")
        rclpy.shutdown()
        return

    arm_state = arm.get_state()
    ee_pose = arm_state["ee_pose"]
    if ee_pose is None:
        node.get_logger().error("End-effector pose unavailable; aborting.")
        rclpy.shutdown()
        return
    ee_pose_matrix = hg.pose_to_matrix(ee_pose.position, ee_pose.orientation)

    result = head_perception.rigid_landmark_points(
        camera_data["rgb_image"],
        camera_data["depth_image"],
        camera_data["camera_info"],
    )
    if result is None:
        node.get_logger().error("No face detected in the calibration frame.")
        rclpy.shutdown()
        return
    rigid_points, _landmarks_px, _jaw = result

    valid_count = int((~np.isnan(rigid_points).any(axis=1)).sum())
    node.get_logger().info(f"Captured {valid_count} valid rigid landmarks.")
    if valid_count < 60:
        node.get_logger().error(
            "Too few valid landmarks; improve lighting/framing and retry."
        )
        rclpy.shutdown()
        return

    reference_points, reference_head_frame, tool_tip_transform = hg.build_calibration(
        rigid_points, ee_pose_matrix, base_to_camera, tool_frame_to_tip
    )

    tool_dir = os.path.join(_CONFIG_DIR, tool)
    os.makedirs(tool_dir, exist_ok=True)
    np.save(os.path.join(tool_dir, "reference_landmarks_camera.npy"), reference_points)
    np.save(os.path.join(tool_dir, "reference_head_frame.npy"), reference_head_frame)
    np.save(os.path.join(tool_dir, "tool_tip_transform.npy"), tool_tip_transform)
    node.get_logger().info(f"Calibration saved to {tool_dir}")

    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Record head-perception calibration.")
    parser.add_argument("--tool", default="drink")
    parser.add_argument("--scene_config", default="wheelchair")
    args = parser.parse_args()
    calibrate(args.tool, args.scene_config)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify it compiles**

Run: `python -m py_compile src/rammp/perception/head_perception/calibrate_head.py`
Expected: no output, exit code 0.

- [ ] **Step 3: Commit**

```bash
git add src/rammp/perception/head_perception/calibrate_head.py
git commit -m "Add head-perception calibration mode"
```

---

## Task 8: Documentation update

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Replace the DECA setup section**

In `README.md`, remove section **"10. Download DECA model files (robot mode only)"** entirely (including the NumPy/chumpy compatibility note). The MediaPipe model is committed in the repo, so no download step is needed.

Also remove the now-unneeded PyTorch / PyTorch3D / chumpy install steps (sections "2. Install PyTorch", "3. Install PyTorch3D", "5. Install additional Python dependencies" — the `chumpy` line) since DECA is no longer used. Leave the ROS 2, workspace, Pinocchio, and `pip install -e ".[full]"` steps.

- [ ] **Step 2: Add a head-calibration section**

After the "Running" section in `README.md`, add:
```markdown
## Head-perception calibration

Head perception tracks the user's head relative to a one-time calibration.
Run this once per user / camera setup before using `bring_cup_to_mouth`:

    python -m rammp.perception.head_perception.calibrate_head --tool drink

Position the user comfortably, hold the drink tip at their mouth, and press
Enter to capture. This writes the reference files to
`src/rammp/perception/head_perception/mediapipe_config/drink/`.
```

- [ ] **Step 3: Update the Architecture section**

In `README.md`, in the "Key Components" table, update the head-perception row (if present) or the perception description so it refers to MediaPipe Face Landmarker rather than DECA. Specifically, ensure no remaining text claims DECA/FLAME is used for head perception.

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "Update docs for MediaPipe head perception and calibration"
```

---

## Notes for the executor

- **Out of scope (separate future plan):** the cup-detection (`drink_perception.py`) performance fixes — vectorizing back-projection, removing per-frame disk writes, dropping `num_samples`, replacing 3D DBSCAN. The timing instrumentation for those is already committed.
- The calibration files for the `drink` tool do not exist until Task 7's calibration is run on hardware. Until then, starting the node will raise the actionable `FileNotFoundError` from `set_tool`. This is expected — calibration is a required one-time setup step.
- Legacy `deca_perception.py` and `DECA/` are intentionally left in place; do not delete them.
```
