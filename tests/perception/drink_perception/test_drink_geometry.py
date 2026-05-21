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
