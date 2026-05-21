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
