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
