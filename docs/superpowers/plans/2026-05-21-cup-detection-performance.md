# Cup-Detection Performance Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `DrinkPerception` (cup-handle detection) fast enough for a Jetson by removing the implementation bottlenecks, without changing the detection algorithm's intent.

**Architecture:** The HSV-mask → cluster → RANSAC-plane → pose algorithm is sound; only its implementation is slow. Four fixes: (1) gate the per-frame debug PNG writes behind a flag, (2) vectorize the per-pixel depth back-projection, (3) replace 3D DBSCAN clustering with a 2D connected-components blob pick, (4) stop running the whole pipeline 3× per call. The pure pixel/blob math is extracted into a testable `drink_geometry.py`; the open3d RANSAC + pose math at the back of the pipeline is unchanged.

**Tech Stack:** Python 3.10, NumPy, OpenCV, Open3D, ROS 2 Humble, pytest.

---

## Background & Context (read before starting)

The file `src/rammp/perception/drink_perception/drink_perception.py` (`DrinkPerception` class) detects a colored cup handle and returns a 6-DOF pose + 2D bounding box. It is driven by `PerceptionInterface.perceive_cup_info` (`src/rammp/interfaces/perception_interface.py`), which is called by the cup-handle streaming loop in `drink_action_server.py` at 10 Hz.

**The measured bottlenecks** (timing instrumentation is already committed; section names like `drink/color_mask` come from `rammp.utils.timing`):
1. `drink_perception.py` writes `color_mask.png` and `handle_mask.png` to disk on **every** call — a probe measured this at ~18 ms of a ~20 ms total.
2. Depth back-projection is a per-pixel Python `for` loop calling `pixel2World` once per masked pixel.
3. Clustering uses `sklearn` `DBSCAN` over the 3D point cloud — heavy, and `sklearn` is an unwanted ARM dependency.
4. `perceive_cup_info` runs the entire pipeline `num_samples=3` times per call and keeps only the last success.

**What is NOT changing:** the HSV color thresholds, the Open3D RANSAC plane fit, and all the plane-basis / `minAreaRect` / pose-axis math at the back of `run_perception`. Open3D stays a dependency (RANSAC). The detection *algorithm* is unchanged — only the clustering method (3D DBSCAN → 2D connected components, which is appropriate because the handle is a single connected colored region in the image) and the implementation efficiency.

**Environment notes for the implementer:**
- The dev machine's system Python is externally-managed. A virtualenv exists at the repo root: **`.venv/`** — it has `numpy`, `scipy`, `opencv`, `open3d`, `scikit-learn`, `pytest`. **Run every `python` / `pytest` command via `.venv/bin/python` and `.venv/bin/pytest`.** `.venv/` is in `.gitignore`.
- `drink_geometry.py` (Task 1) is pure cv2/numpy and gets real pytest TDD.
- `drink_perception.py` imports `open3d` — importable in the venv, so an import + smoke check is possible, but it is NOT given a committed pytest file (to keep the committed test suite light). Its verification is `py_compile` + a one-off smoke command.
- `perception_interface.py` imports `rclpy` (not installed here) — `py_compile` only.

**Camera data shapes:** `rgb_image` is BGR uint8 1280×720; `depth_image` is single-channel depth in **millimeters**; `camera_info` is a ROS `CameraInfo` with intrinsics in `.k` (`fx=k[0]`, `fy=k[4]`, `cx=k[2]`, `cy=k[5]`).

---

## File Structure

**Create:**
- `src/rammp/perception/drink_perception/drink_geometry.py` — pure helpers: `largest_blob`, `backproject_mask`.
- `tests/perception/drink_perception/test_drink_geometry.py` — tests for the above.

**Modify:**
- `src/rammp/perception/drink_perception/drink_perception.py` — `run_perception` uses the new helpers; `debug` flag gates the PNG writes; DBSCAN/sklearn removed.
- `src/rammp/interfaces/perception_interface.py` — `perceive_cup_info` default `num_samples` 3 → 1.
- `pyproject.toml` — remove `scikit-learn` from the `full` extra (no longer used anywhere).

---

## Task 1: Pure cup-geometry helpers

**Files:**
- Create: `src/rammp/perception/drink_perception/drink_geometry.py`
- Test: `tests/perception/drink_perception/test_drink_geometry.py`

This module is pure cv2/numpy — no open3d, no sklearn — and is the testable core of fixes 2 and 3.

- [ ] **Step 1: Write the failing tests** — Create `tests/perception/drink_perception/test_drink_geometry.py`:
```python
import numpy as np

from rammp.perception.drink_perception import drink_geometry as dg


def test_largest_blob_picks_bigger_component():
    mask = np.zeros((40, 40), dtype=np.uint8)
    mask[2:5, 2:5] = 255       # small component: 9 px
    mask[10:25, 10:25] = 255   # big component: 225 px
    blob = dg.largest_blob(mask, min_area=50)
    assert blob is not None
    assert blob.dtype == np.uint8
    assert set(np.unique(blob)).issubset({0, 255})
    assert int((blob > 0).sum()) == 225
    assert blob[3, 3] == 0      # small component excluded
    assert blob[15, 15] == 255


def test_largest_blob_empty_mask_returns_none():
    assert dg.largest_blob(np.zeros((20, 20), dtype=np.uint8), min_area=50) is None


def test_largest_blob_below_min_area_returns_none():
    mask = np.zeros((20, 20), dtype=np.uint8)
    mask[1:3, 1:3] = 255        # 4 px
    assert dg.largest_blob(mask, min_area=50) is None


def test_backproject_mask_basic():
    mask = np.zeros((20, 20), dtype=np.uint8)
    mask[10, 10] = 255
    depth = np.full((20, 20), 500.0, dtype=np.float32)  # 0.5 m, in mm
    pts, px = dg.backproject_mask(mask, depth, fx=600, fy=600, cx=10, cy=10)
    assert pts.shape == (1, 3) and px.shape == (1, 2)
    np.testing.assert_allclose(pts[0], [0.0, 0.0, 0.5], atol=1e-9)
    np.testing.assert_array_equal(px[0], [10, 10])


def test_backproject_mask_offcenter():
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[250, 330] = 255
    depth = np.full((480, 640), 1000.0, dtype=np.float32)  # 1.0 m
    pts, _ = dg.backproject_mask(mask, depth, fx=500, fy=500, cx=320, cy=240)
    # X = (1.0/500)*(330-320) = 0.02 ; Y = (1.0/500)*(250-240) = 0.02
    np.testing.assert_allclose(pts[0], [0.02, 0.02, 1.0], atol=1e-9)


def test_backproject_mask_excludes_invalid_depth():
    mask = np.zeros((20, 20), dtype=np.uint8)
    mask[5, 5] = 255           # depth 0 -> invalid
    mask[6, 6] = 255           # depth valid
    depth = np.zeros((20, 20), dtype=np.float32)
    depth[6, 6] = 500.0
    pts, px = dg.backproject_mask(mask, depth, fx=600, fy=600, cx=10, cy=10)
    assert pts.shape == (1, 3)
    np.testing.assert_array_equal(px[0], [6, 6])


def test_backproject_mask_empty_mask():
    mask = np.zeros((20, 20), dtype=np.uint8)
    depth = np.full((20, 20), 500.0, dtype=np.float32)
    pts, px = dg.backproject_mask(mask, depth, fx=600, fy=600, cx=10, cy=10)
    assert pts.shape == (0, 3) and px.shape == (0, 2)
```

- [ ] **Step 2: Run tests to verify they fail** — Run: `.venv/bin/pytest tests/perception/drink_perception/test_drink_geometry.py -v` — Expect FAIL (`ModuleNotFoundError`). If the directory needs an `__init__.py` for collection (consistent with `tests/perception/head_perception/__init__.py`), create an empty `tests/perception/drink_perception/__init__.py`.

- [ ] **Step 3: Implement `drink_geometry.py`** — Create `src/rammp/perception/drink_perception/drink_geometry.py`:
```python
"""Pure geometry helpers for the drink (cup-handle) perception pipeline.

Only OpenCV and NumPy — no Open3D or scikit-learn — so these are
independently unit-testable. All 3D points are in the camera frame, meters.
"""

import cv2
import numpy as np


def largest_blob(mask: np.ndarray, min_area: int = 200) -> np.ndarray | None:
    """Return a binary mask (uint8, values 0/255) of the largest connected
    component of `mask`, or None if no component has at least `min_area`
    pixels.

    This replaces 3D DBSCAN clustering: the cup handle is a single connected
    colored region in the image, so the largest 2D connected component is it.
    """
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask, connectivity=8
    )
    if num_labels <= 1:  # label 0 is background; nothing else found
        return None
    # stats row 0 is the background; search components 1..num_labels-1.
    areas = stats[1:, cv2.CC_STAT_AREA]
    largest_label = 1 + int(np.argmax(areas))
    if int(stats[largest_label, cv2.CC_STAT_AREA]) < min_area:
        return None
    return (labels == largest_label).astype(np.uint8) * 255


def backproject_mask(
    mask: np.ndarray,
    depth_image: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    min_depth_m: float = 0.05,
    max_depth_m: float = 1.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Back-project every set pixel of `mask` to a 3D camera-frame point.

    mask: binary image (non-zero selects a pixel). depth_image: same height
    and width, depth in millimeters. Returns (points_3d, pixels):
      - points_3d: (M, 3) float64 camera-frame points in meters
      - pixels: (M, 2) int (x, y) pixel coordinates, aligned with points_3d
    Only pixels whose depth is finite and within [min_depth_m, max_depth_m]
    are included; M may be 0. This is the vectorized replacement for a
    per-pixel Python loop.
    """
    ys, xs = np.where(mask > 0)
    if xs.size == 0:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 2), dtype=int)

    depth_m = depth_image[ys, xs].astype(np.float64) / 1000.0
    valid = (
        np.isfinite(depth_m) & (depth_m >= min_depth_m) & (depth_m <= max_depth_m)
    )
    xs, ys, depth_m = xs[valid], ys[valid], depth_m[valid]

    world_x = (depth_m / fx) * (xs - cx)
    world_y = (depth_m / fy) * (ys - cy)
    points_3d = np.stack([world_x, world_y, depth_m], axis=1)
    pixels = np.stack([xs, ys], axis=1).astype(int)
    return points_3d, pixels
```

- [ ] **Step 4: Run tests to verify they pass** — Run: `.venv/bin/pytest tests/perception/drink_perception/test_drink_geometry.py -v` — Expect PASS, all 6 tests.

- [ ] **Step 5: Commit**
```bash
git add src/rammp/perception/drink_perception/drink_geometry.py tests/perception/drink_perception/
git commit -m "Add pure cup-geometry helpers for drink perception"
```

---

## Task 2: Refactor DrinkPerception.run_perception

**Files:**
- Modify: `src/rammp/perception/drink_perception/drink_perception.py`

Replace the ENTIRE file contents with the version below. Changes vs. the current file: a `debug` constructor flag gates the two `cv2.imwrite` calls; clustering is `drink_geometry.largest_blob` instead of `DBSCAN`; back-projection is `drink_geometry.backproject_mask` instead of the per-pixel loop; the `sklearn`/`DBSCAN` import and the now-unused `pixel2World` method and `os`/`sys`/`time`/`math` imports are removed. The Open3D RANSAC block and all pose math from `plane_cloud = ...` onward are unchanged.

- [ ] **Step 1: Replace `src/rammp/perception/drink_perception/drink_perception.py`** with EXACTLY:
```python
# Detects a colored cup handle and estimates its 6-DOF pose in the camera frame.

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import open3d as o3d

from rammp.perception.drink_perception import drink_geometry as dg
from rammp.utils.timing import timer

# Smallest connected colored region (pixels) accepted as a candidate handle.
_MIN_BLOB_AREA = 200
# Fewest valid 3D points required to attempt a plane/pose fit.
_MIN_CLUSTER_POINTS = 50


class DrinkPerception():
    def __init__(self, debug: bool = False):
        # When debug is True, run_perception writes color_mask.png and
        # handle_mask.png to the working directory. Off by default — those
        # per-frame disk writes dominated the runtime.
        self.debug = debug

    def pose_to_matrix(self, pose):
        position = pose[0]
        orientation = pose[1]
        pose_matrix = np.zeros((4, 4))
        pose_matrix[:3, 3] = position
        pose_matrix[:3, :3] = Rotation.from_quat(orientation).as_matrix()
        pose_matrix[3, 3] = 1
        return pose_matrix

    def matrix_to_pose(self, mat):
        position = mat[:3, 3]
        orientation = Rotation.from_matrix(mat[:3, :3]).as_quat()
        return (position, orientation)

    def run_perception(self, rgb_image, camera_info, depth_image, base_to_camera_transform):

        # -----------------------------
        # Color mask
        # -----------------------------
        with timer("drink/color_mask"):
            mask = self.detect_handle_color(rgb_image)

        if self.debug:
            vis = rgb_image.copy()
            vis[mask > 0] = (0, 255, 0)
            cv2.imwrite("color_mask.png", vis)

        with timer("drink/clean_mask"):
            mask = self.clean_mask(mask)

        # -----------------------------
        # Largest connected blob (replaces 3D DBSCAN)
        # -----------------------------
        with timer("drink/cluster"):
            cluster_mask = dg.largest_blob(mask, min_area=_MIN_BLOB_AREA)
        if cluster_mask is None:
            return None, None

        # -----------------------------
        # Back-project the blob to 3D (vectorized)
        # -----------------------------
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]
        with timer("drink/backproject"):
            cluster_points_3d, cluster_pixels = dg.backproject_mask(
                cluster_mask, depth_image, fx, fy, cx, cy
            )
        if len(cluster_points_3d) < _MIN_CLUSTER_POINTS:
            return None, None

        if self.debug:
            vis = rgb_image.copy()
            vis[cluster_mask > 0] = (0, 0, 255)
            cv2.imwrite("handle_mask.png", vis)

        with timer("drink/ransac_plane"):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(cluster_points_3d)

            plane_model, inliers = pcd.segment_plane(
                distance_threshold=0.003,
                ransac_n=3,
                num_iterations=500
            )

        plane_cloud = pcd.select_by_index(inliers)
        non_plane_cloud = pcd.select_by_index(inliers, invert=True)

        pts3d_planar = np.asarray(plane_cloud.points)
        pts3d_handle = np.asarray(non_plane_cloud.points)

        # Plane normal
        a, b, c, d = plane_model
        n = np.array([a, b, c])
        n = n / np.linalg.norm(n)

        # Build plane basis (camera is level)
        up = np.array([0, 0, 1])
        u = np.cross(up, n)
        u = u / np.linalg.norm(u)
        v = np.cross(n, u)

        # Project 3D points to 2D plane coordinates
        P0 = pts3d_planar.mean(axis=0)
        P = pts3d_planar - P0
        x = P @ u
        y = P @ v
        P2 = np.stack([x, y], axis=1).astype(np.float32)

        # Fit minimum-area rectangle
        rect = cv2.minAreaRect(P2)
        (center_2d, _, _) = rect

        # Back-project center to 3D
        center_3d = P0 + center_2d[0] * u + center_2d[1] * v

        # Get 4 rectangle corners in 2D (plane coordinates)
        box_2d = cv2.boxPoints(rect)  # shape (4,2)

        # Back-project corners to 3D
        corners_3d = []
        for x2d, y2d in box_2d:
            p3d = P0 + x2d * u + y2d * v
            corners_3d.append(p3d)

        corners_3d = np.array(corners_3d)

        points_to_show = np.vstack([center_3d.reshape(1, 3), corners_3d])  # (5,3)

        # Sort corners by image-space Y (top vs bottom)
        # Smaller Y = higher in image (top)
        ys = corners_3d[:, 1]
        top_idx = np.argsort(ys)[:2]
        bottom_idx = np.argsort(ys)[2:]

        top_pts = corners_3d[top_idx]
        bottom_pts = corners_3d[bottom_idx]

        # Sort left/right within top and bottom using X
        top_left, top_right = top_pts[np.argsort(top_pts[:, 0])]
        bottom_left, bottom_right = bottom_pts[np.argsort(bottom_pts[:, 0])]

        # X-axis: bottom → top
        x_axis = ((top_left + top_right) / 2.0) - ((bottom_left + bottom_right) / 2.0)
        x_axis = x_axis / np.linalg.norm(x_axis)

        # Y-axis: right → left
        y_axis = ((top_left + bottom_left) / 2.0) - ((top_right + bottom_right) / 2.0)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Z-axis: towards camera (right-handed)
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)

        # Re-orthogonalize Y to avoid drift
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Rotation matrix (columns are axes)
        R_mat = np.column_stack((x_axis, y_axis, z_axis))

        # cam to tag homogeneous transform
        camera_to_tag = np.zeros((4, 4))
        camera_to_tag[:3, :3] = R_mat
        camera_to_tag[:3, 3] = center_3d
        camera_to_tag[3, 3] = 1

        # base to tag homogeneous transform and update tf
        base_to_tag = np.dot(base_to_camera_transform, camera_to_tag)

        x_min, y_min = cluster_pixels.min(axis=0)
        x_max, y_max = cluster_pixels.max(axis=0)
        bounding_box = [int(x_min), int(y_min), int(x_max), int(y_max)]

        return self.matrix_to_pose(base_to_tag), bounding_box

    def detect_handle_color(self, bgr_image):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        lower = np.array([37, 205, 78])
        upper = np.array([105, 255, 255])

        return cv2.inRange(hsv, lower, upper)

    def clean_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask
```

- [ ] **Step 2: Verify it compiles** — Run: `.venv/bin/python -m py_compile src/rammp/perception/drink_perception/drink_perception.py` — Expect exit 0, no output.

- [ ] **Step 3: Smoke-test import + the debug flag** — Run this command from the repo root:
```bash
cd /home/swapnil/atdev/RAMMP && rm -f color_mask.png handle_mask.png && .venv/bin/python -c "
import sys, os
sys.path.insert(0, 'src')
import cv2, numpy as np
from types import SimpleNamespace
from rammp.perception.drink_perception.drink_perception import DrinkPerception
base = 'src/rammp/perception/drink_perception/'
rgb = cv2.imread(base + 'rgb.png', cv2.IMREAD_COLOR)
depth = cv2.imread(base + 'depth.png', cv2.IMREAD_UNCHANGED).astype(np.float32)
cam = SimpleNamespace(k=[920.0, 0, 640.0, 0, 920.0, 360.0, 0, 0, 1])
res = DrinkPerception().run_perception(rgb, cam, depth, np.eye(4))
assert isinstance(res, tuple) and len(res) == 2, res
assert not os.path.exists('color_mask.png'), 'debug=False must NOT write color_mask.png'
DrinkPerception(debug=True).run_perception(rgb, cam, depth, np.eye(4))
assert os.path.exists('color_mask.png'), 'debug=True SHOULD write color_mask.png'
print('smoke OK')
" && rm -f color_mask.png handle_mask.png
```
Expected: prints `smoke OK`. (The fixture's color does not match the HSV range, so `run_perception` returns `(None, None)` — that is fine; the smoke test checks the pipeline runs without exception and that the `debug` flag correctly gates the PNG writes.)

If the smoke test fails, report BLOCKED with the output — do not weaken the assertions.

- [ ] **Step 4: Commit**
```bash
git add src/rammp/perception/drink_perception/drink_perception.py
git commit -m "Vectorize cup detection: blob clustering, gated debug writes"
```

---

## Task 3: Drop redundant sampling and the sklearn dependency

**Files:**
- Modify: `src/rammp/interfaces/perception_interface.py`
- Modify: `pyproject.toml`

- [ ] **Step 1: Reduce `num_samples` default** — In `src/rammp/interfaces/perception_interface.py`, in the `perceive_cup_info` method definition, change:
```python
    def perceive_cup_info(self, num_samples: int = 3) -> CupInfo:
```
to:
```python
    def perceive_cup_info(self, num_samples: int = 1) -> CupInfo:
```
(The method kept only the *last* successful sample of the `num_samples` runs, so running it 3× per call tripled cost for no benefit. The 10 Hz streaming loop already provides retries.)

- [ ] **Step 2: Remove the unused `scikit-learn` dependency** — In `pyproject.toml`, in the `full` optional-dependency list, delete the line `"scikit-learn",`. (After Task 2, `sklearn`/`DBSCAN` is no longer imported anywhere in `src/rammp` — `DrinkPerception` was its only user.)

- [ ] **Step 3: Verify** — Run:
```bash
.venv/bin/python -m py_compile src/rammp/interfaces/perception_interface.py
grep -rn "sklearn\|DBSCAN" /home/swapnil/atdev/RAMMP/src/rammp || echo "no sklearn references remain"
```
Expected: `py_compile` exits 0 with no output; the grep prints `no sklearn references remain`.

- [ ] **Step 4: Commit**
```bash
git add src/rammp/interfaces/perception_interface.py pyproject.toml
git commit -m "Reduce cup sampling to 1 and drop unused scikit-learn dependency"
```

---

## Notes for the executor

- **Out of scope:** replacing the Open3D RANSAC plane fit with a NumPy implementation. Open3D remains a dependency. This could be a future task if Open3D proves problematic on the Jetson.
- The variables `pts3d_handle` and `points_to_show` in `run_perception` are computed but unused — they are pre-existing and left as-is to keep this change focused on the four performance fixes.
- The cup-detection algorithm's *behavior* changes in one intentional way: clustering is now the largest 2D connected component rather than the largest 3D DBSCAN cluster. For a single connected colored handle this is equivalent and faster.
- After this plan, the timing sections for cup detection are `drink/color_mask`, `drink/clean_mask`, `drink/cluster`, `drink/backproject`, `drink/ransac_plane` (the old `drink/dbscan` and `drink/debug_imwrite` sections are gone).
