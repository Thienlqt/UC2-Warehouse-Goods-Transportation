import pytest

from unity_slam_example.reverse_limit import ReverseBudget


def reverse_for(limit, metres, v=-0.05, w=0.1, dt=0.1):
    return [limit(v, w, dt) for _ in range(int(round(metres / (-v * dt))))]


def test_short_reverse_passes_through():
    limit = ReverseBudget(budget=0.15)
    assert reverse_for(limit, 0.15) == [(-0.05, 0.1)] * 30


def test_long_reverse_becomes_turn_in_place_in_one_direction():
    # DWA reverses all the way to a goal behind the robot; past the budget, turn round instead.
    limit = ReverseBudget(budget=0.15, turn_rate=0.4)
    reverse_for(limit, 0.15, w=-0.1)
    assert limit(-0.05, -0.1, 0.1) == (0.0, -0.4)
    assert limit(-0.05, 0.2, 0.1) == (0.0, -0.4)     # DWA's turn sign flickers; keep turning


def test_forward_command_restores_the_budget():
    limit = ReverseBudget(budget=0.15)
    reverse_for(limit, 0.2)
    assert limit(0.1, 0.0, 0.1) == (0.1, 0.0)
    assert limit(-0.05, 0.1, 0.1) == (-0.05, 0.1)


def test_turns_and_stops_pass_through():
    limit = ReverseBudget(budget=0.15)
    reverse_for(limit, 0.2)
    assert limit(0.0, 0.3, 0.1) == (0.0, 0.3)
    assert limit(0.0, 0.0, 0.1) == (0.0, 0.0)


def test_gap_between_commands_counts_as_one_period():
    limit = ReverseBudget(budget=0.15, max_dt=0.2)
    assert limit(-0.05, 0.0, 10.0) == (-0.05, 0.0)    # 10 s gap: counts 0.01 m, not 0.5 m
    assert limit.used == pytest.approx(0.01)


def test_clock_jumping_back_adds_no_budget():
    # Unity Play restarted without restarting ROS: /clock goes back, dt < 0.
    limit = ReverseBudget(budget=0.15)
    limit(-0.05, 0.1, -600.0)
    assert limit.used == 0.0
    reverse_for(limit, 0.15)
    assert limit(-0.05, 0.1, 0.1) == (0.0, 0.4)
