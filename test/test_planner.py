import math

from mini_nav.planner import GridPlanner, obstacle_distance
import numpy as np
import pytest


def make_planner(occupied, **kwargs):
    kwargs.setdefault('robot_radius', 0.1)
    kwargs.setdefault('inflation_radius', 0.2)
    return GridPlanner(np.asarray(occupied, dtype=bool), resolution=0.1, **kwargs)


def test_obstacle_distance_is_chebyshev():
    occupied = np.zeros((7, 7), dtype=bool)
    occupied[3, 3] = True
    dist = obstacle_distance(occupied, max_distance=2)
    assert dist[3, 3] == 0
    assert dist[4, 4] == 1
    assert dist[3, 5] == 2
    assert dist[0, 0] == 3  # 上限 + 1


def test_straight_path_in_empty_map():
    planner = make_planner(np.zeros((20, 20)))
    points, message = planner.plan_world((0.25, 1.05), (1.75, 1.05))
    assert message == 'ok'
    assert all(abs(y - 1.05) < 1e-9 for _, y in points)
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    assert length == pytest.approx(1.5, abs=0.11)


def test_path_goes_around_wall():
    occupied = np.zeros((30, 30), dtype=bool)
    occupied[0:25, 15] = True  # 上側だけ開いた縦の壁
    planner = make_planner(occupied)
    points, message = planner.plan_world((0.5, 0.5), (2.5, 0.5))
    assert message == 'ok'
    for x, y in points:
        assert not planner.lethal[int(y / 0.1), int(x / 0.1)]
    assert max(y for _, y in points) > 2.5  # 壁の上端を回り込んでいる


def test_unreachable_goal():
    occupied = np.zeros((30, 30), dtype=bool)
    occupied[:, 15] = True
    planner = make_planner(occupied)
    points, message = planner.plan_world((0.5, 0.5), (2.5, 0.5))
    assert points is None
    assert message == 'no path found'


def test_goal_inside_obstacle_is_rejected():
    occupied = np.zeros((20, 20), dtype=bool)
    occupied[10, 10] = True
    planner = make_planner(occupied)
    points, message = planner.plan_world((0.5, 0.5), (1.05, 1.05))
    assert points is None
    assert 'obstacle' in message


def test_start_near_wall_snaps_to_free_cell():
    occupied = np.zeros((20, 20), dtype=bool)
    occupied[:, 0] = True
    planner = make_planner(occupied)
    # x = 0.15 は lethal 領域 (壁 + ロボット半径) の中
    assert not planner.is_free(planner.world_to_cell(0.15, 1.0))
    points, message = planner.plan_world((0.15, 1.0), (1.5, 1.0))
    assert message == 'ok'
