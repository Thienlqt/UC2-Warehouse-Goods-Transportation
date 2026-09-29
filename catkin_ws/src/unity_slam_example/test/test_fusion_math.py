import numpy as np

from unity_slam_example.perception.fusion_math import (associate, box_corners, fused_score,
                                                        iou_1d, project_interval)


def test_projection_of_box_straight_ahead_is_centred():
    # Camera optical frame (x right, y down, z forward): a 0.4 m cube centred 2 m ahead.
    cam = np.array([(x, y, z) for x in (-0.2, 0.2) for y in (-0.2, 0.2) for z in (1.8, 2.2)])
    u_min, u_max, depth = project_interval(cam, fx=600.0, cx=320.0, image_width=640)
    assert abs((u_min + u_max) / 2 - 320) < 1e-6
    assert abs(depth - 1.8) < 1e-6                                 # near face
    assert abs(u_max - (320 + 600 * 0.2 / 1.8)) < 1e-6


def test_box_corners_are_rotated_about_the_centre():
    corners = box_corners(1.0, 2.0, np.pi / 2, 0.4, 0.2, 0.0, 0.5)
    assert corners.shape == (8, 3)
    assert np.allclose(corners[:, 0].min(), 0.9) and np.allclose(corners[:, 0].max(), 1.1)
    assert np.allclose(corners[:, 1].min(), 1.8) and np.allclose(corners[:, 1].max(), 2.2)


def test_projection_behind_camera_is_none():
    corners = np.array([[0.0, 0.0, -1.0], [0.1, 0.0, -2.0]])
    assert project_interval(corners, 600.0, 320.0, 640) is None


def test_iou_1d():
    assert iou_1d((0, 10), (5, 15)) == 5 / 15
    assert iou_1d((0, 1), (2, 3)) == 0.0


def test_association_labels_matches_and_merges_leg_clusters():
    tracks = {1: (100, 140), 2: (160, 200), 3: (400, 480)}       # two legs + a box
    camera = [(95, 205, 'person', 0.9), (395, 490, 'box', 0.8), (600, 640, 'shelf', 0.7)]
    labels = associate(tracks, camera)
    assert labels[1][0] == 'person' and labels[2][0] == 'person'
    assert labels[3][0] == 'box'
    assert all(label != 'shelf' for label, _, _ in labels.values())


def test_poor_overlap_is_not_matched():
    assert associate({1: (0, 100)}, [(90, 300, 'box', 0.9)]) == {}


def test_fused_score_discounts_by_overlap():
    assert fused_score(0.8, 1.0) == 0.8
    assert fused_score(0.8, 0.5) == 0.6
