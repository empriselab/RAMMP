import numpy as np

from rammp.perception.drink_perception import drink_geometry as dg


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


def _grid_blob(x0, x1, y0, y1, z, fx=600.0, cx=320.0, cy=240.0):
    ys, xs = np.mgrid[y0:y1, x0:x1]
    pts = np.stack([(xs - cx) * z / fx, (ys - cy) * z / fx, np.full(xs.shape, z)], axis=-1)
    return pts.reshape(-1, 3), np.stack([xs.ravel(), ys.ravel()], axis=1)


def test_cluster_points_small_input_is_exact_dbscan():
    from sklearn.cluster import DBSCAN
    pts, pix = _grid_blob(200, 240, 100, 140, 0.3)  # 1600 points, no subsampling
    labels = dg.cluster_points(pts, pix, max_points=4000)
    expected = DBSCAN(eps=0.07, min_samples=50).fit(pts).labels_
    np.testing.assert_array_equal(labels, expected)


def test_cluster_points_subsampled_matches_exact_clusters():
    from sklearn.cluster import DBSCAN
    p1, x1 = _grid_blob(200, 320, 100, 200, 0.3)   # big close-up blob
    p2, x2 = _grid_blob(500, 540, 300, 330, 0.5)   # separate blob ~20 cm away
    pts, pix = np.vstack([p1, p2]), np.vstack([x1, x2])
    labels = dg.cluster_points(pts, pix, max_points=2000)  # forces k > 1
    exact = DBSCAN(eps=0.07, min_samples=50).fit(pts).labels_
    assert labels.max() == exact.max() == 1
    big = labels == np.bincount(labels[labels >= 0]).argmax()
    big_exact = exact == np.bincount(exact[exact >= 0]).argmax()
    assert np.array_equal(big, big_exact)
