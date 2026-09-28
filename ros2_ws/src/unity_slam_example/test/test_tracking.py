import math

from unity_slam_example.perception.lidar_segmentation import Box
from unity_slam_example.perception.tracking import boxes_to_frame, Tracker


def box(x, y):
    return Box(x, y, 0.0, 0.4, 0.4, 12, 0.005)


def test_moving_object_keeps_id_and_estimates_velocity():
    tracker = Tracker()
    for k in range(20):                      # 10 Hz, moving +x at 0.8 m/s
        tracks = tracker.step([box(1.0 + 0.08 * k, 0.5)], stamp=0.1 * k)
    assert len(tracks) == 1
    assert tracks[0].id == 1
    assert abs(tracks[0].x[2] - 0.8) < 0.1 and abs(tracks[0].x[3]) < 0.1
    assert tracks[0].confidence() > 0.8


def test_new_track_needs_two_hits_and_is_dropped_after_misses():
    tracker = Tracker(max_misses=3)
    assert tracker.step([box(1, 1)], 0.0) == []          # tentative after one scan
    assert len(tracker.step([box(1, 1)], 0.1)) == 1      # confirmed
    for k in range(4):
        tracks = tracker.step([], 0.2 + 0.1 * k)
    assert tracks == [] and tracker.tracks == []


def test_two_objects_are_not_swapped():
    tracker = Tracker()
    for k in range(10):
        tracks = tracker.step([box(2, -0.5 + 0.02 * k), box(2, 0.5 - 0.02 * k)], 0.1 * k)
    by_id = {t.id: t for t in tracks}
    assert by_id[1].x[1] < by_id[2].x[1]                  # first stays below the second


def test_boxes_to_frame_rotates_and_translates():
    (b,) = boxes_to_frame([box(1.0, 0.0)], 2.0, 3.0, math.pi / 2)
    assert abs(b.x - 2.0) < 1e-9 and abs(b.y - 4.0) < 1e-9
