"""Pure geometry helpers for the drink (cup-handle) perception pipeline.

Only OpenCV and NumPy — no Open3D or scikit-learn — so these are
independently unit-testable. All 3D points are in the camera frame, meters.
"""

import numpy as np
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN


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


def cluster_points(
    points_3d: np.ndarray,
    pixels: np.ndarray,
    eps: float = 0.07,
    min_samples: int = 50,
    max_points: int = 4000,
) -> np.ndarray:
    """DBSCAN labels for back-projected mask points (-1 = noise).

    Exactly ``DBSCAN(eps, min_samples)`` when there are at most `max_points`
    points. Above that, DBSCAN on a regular pixel-grid subsample (every k-th
    pixel in x and y, min_samples scaled by 1/k^2 to keep the same density
    criterion) and every remaining point takes the label of its nearest
    clustered subsample point within `eps`. Exact DBSCAN is quadratic on the
    dense pixel grids a close-up handle produces (seconds per frame); the
    subsampled version returns the same clusters in milliseconds.
    """
    n = len(points_3d)
    k = int(np.ceil(np.sqrt(n / max_points))) if n > max_points else 1
    if k <= 1:
        return DBSCAN(eps=eps, min_samples=min_samples).fit(points_3d).labels_
    sel = (pixels[:, 0] % k == 0) & (pixels[:, 1] % k == 0)
    sub_labels = DBSCAN(
        eps=eps, min_samples=max(1, min_samples // (k * k))
    ).fit(points_3d[sel]).labels_
    labels = np.full(n, -1, dtype=int)
    keep = sub_labels >= 0
    if keep.any():
        dist, idx = cKDTree(points_3d[sel][keep]).query(points_3d, distance_upper_bound=eps)
        hit = np.isfinite(dist)
        labels[hit] = sub_labels[keep][idx[hit]]
    return labels
