"""Queue behavior that must hold before connecting to the live move_base server."""

import json

import pytest

from unity_slam_example.delivery_queue import DeliveryQueue, parse_request


def request(identifier, x=1.0):
    return parse_request(json.dumps({
        "id": identifier,
        "pickup": {"x": x, "y": 0, "yaw": 0},
        "dropoff": {"x": x + 1, "y": 1, "yaw": 1.57},
        "home": {"x": 0, "y": 0, "yaw": 0},
    }))


def test_requests_complete_all_three_legs_in_fifo_order():
    queue = DeliveryQueue()
    queue.enqueue(request("first"))
    queue.enqueue(request("second", 5))

    first, stage, pose = queue.begin_next()
    assert (first.request_id, stage, pose.x) == ("first", "pickup", 1)
    assert queue.begin_next() is None
    assert queue.snapshot()["pending"] == ["second"]

    first, stage, pose = queue.finish_leg(True)
    assert (first.request_id, stage, pose.x) == ("first", "dropoff", 2)
    first, stage, pose = queue.finish_leg(True)
    assert (first.request_id, stage, pose.x) == ("first", "home", 0)
    assert queue.snapshot()["active"] == {"id": "first", "stage": "home"}
    assert queue.finish_leg(True) is None

    second, stage, pose = queue.begin_next()
    assert (second.request_id, stage, pose.x) == ("second", "pickup", 5)


def test_failed_pickup_skips_dropoff_and_unblocks_next_request():
    queue = DeliveryQueue()
    queue.enqueue(request("blocked"))
    queue.enqueue(request("next"))
    queue.begin_next()
    assert queue.finish_leg(False) is None
    assert queue.begin_next()[0].request_id == "next"


@pytest.mark.parametrize("completed_legs", [0, 1, 2])
def test_failure_at_any_stop_unblocks_the_next_delivery(completed_legs):
    queue = DeliveryQueue()
    queue.enqueue(request("failed"))
    queue.enqueue(request("next"))
    queue.begin_next()
    for _ in range(completed_legs):
        queue.finish_leg(True)
    assert queue.finish_leg(False) is None
    assert queue.begin_next()[0].request_id == "next"


def test_cancel_only_removes_the_requested_pending_delivery():
    queue = DeliveryQueue()
    queue.enqueue(request("active"))
    queue.enqueue(request("cancel"))
    queue.enqueue(request("keep"))
    queue.begin_next()
    assert [item.request_id for item in queue.cancel_pending("cancel")] == ["cancel"]
    assert queue.snapshot() == {
        "active": {"id": "active", "stage": "pickup"},
        "pending": ["keep"],
    }


def test_duplicate_ids_and_queue_capacity_are_rejected():
    queue = DeliveryQueue(max_pending=1)
    queue.enqueue(request("one"))
    with pytest.raises(ValueError, match="already queued"):
        queue.enqueue(request("one"))
    with pytest.raises(ValueError, match="full"):
        queue.enqueue(request("two"))


def test_request_id_is_normalized_before_duplicate_check():
    queue = DeliveryQueue()
    queue.enqueue(request("order"))
    with pytest.raises(ValueError, match="already queued"):
        queue.enqueue(request(" order "))


@pytest.mark.parametrize("payload", [
    "not JSON",
    "[]",
    '{"id":"", "pickup":{}, "dropoff":{}}',
    '{"id":"bad", "pickup":{"x":1,"y":0,"yaw":0},'
    ' "dropoff":{"x":"NaN","y":0,"yaw":0}}',
    '{"id":"missing-home","pickup":{"x":1,"y":0,"yaw":0},'
    '"dropoff":{"x":2,"y":0,"yaw":0}}',
])
def test_invalid_requests_are_rejected(payload):
    with pytest.raises(ValueError):
        parse_request(payload)
