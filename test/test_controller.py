import math

from mini_nav.controller import PurePursuit
import pytest


def straight_path(length=3.0, step=0.05):
    n = int(length / step) + 1
    return [(i * step, 0.0) for i in range(n)]


def test_drives_forward_on_straight_path():
    pp = PurePursuit()
    pp.set_path(straight_path())
    v, w, reached = pp.compute(0.0, 0.0, 0.0)
    assert not reached
    assert v == pytest.approx(pp.max_linear)
    assert w == pytest.approx(0.0, abs=1e-6)


def test_turns_left_towards_path_on_the_left():
    pp = PurePursuit()
    pp.set_path(straight_path())
    v, w, _ = pp.compute(0.0, -0.2, 0.0)  # 経路より右にずれている
    assert v > 0.0
    assert w > 0.0


def test_rotates_in_place_when_facing_away():
    pp = PurePursuit()
    pp.set_path(straight_path())
    v, w, _ = pp.compute(0.0, 0.0, math.pi)
    assert v == 0.0
    assert abs(w) == pytest.approx(pp.max_angular)


def test_slows_down_near_goal():
    pp = PurePursuit(slow_down_radius=0.6)
    pp.set_path(straight_path())
    v, _, _ = pp.compute(2.7, 0.0, 0.0)
    assert 0.0 < v < pp.max_linear


def test_reports_goal_reached():
    pp = PurePursuit(goal_tolerance=0.1)
    pp.set_path(straight_path())
    assert pp.compute(2.95, 0.0, 0.0) == (0.0, 0.0, True)


def test_angular_velocity_is_limited():
    pp = PurePursuit(max_angular=0.5, rotate_threshold=math.radians(89))
    pp.set_path([(0.0, 0.0), (0.1, 0.5), (0.1, 1.0)])
    v, w, _ = pp.compute(0.0, 0.0, 0.0)
    assert abs(w) == pytest.approx(0.5)
    assert v > 0.0
