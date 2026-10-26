"""move_base.yaml limits the busy-warehouse work relies on (see docs/obstacle_avoidance.md)."""

import os

import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'move_base.yaml')


def load():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


def test_abort_budget_matches_today():
    # Act within 2 s, but abort a goal only after ~45 s of failing, as with the old 15 s patience
    # and two recoveries: delivery_controller treats an abort as a failed delivery.
    c = load()
    assert c['controller_patience'] <= 2.0
    assert (len(c['recovery_behaviors']) + 1) * c['controller_patience'] >= 45.0


def test_dwa_reverses_slowly_and_unlocks_quickly():
    # -0.1 let a reverse arc win beside a box and the 0.05 m oscillation lock kept it reversing
    # (aisle_narrow, October 2026); -0.05 with a 0.01 m lock passed 15/15.
    dwa = load()['DWAPlannerROS']
    assert dwa['min_vel_x'] == -0.05
    assert dwa['oscillation_reset_dist'] == 0.01
