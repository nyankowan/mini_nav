"""Planner + controller + simulated robot, end to end, without ROS."""

import math
import os

from mini_nav.controller import PurePursuit
from mini_nav.planner import GridPlanner
from mini_nav.sim_core import DiffDriveRobot, GridMap
import pytest

MAP_FILE = os.path.join(os.path.dirname(__file__), '..', 'maps', 'simple_room.txt')


@pytest.fixture(scope='module')
def world():
    with open(MAP_FILE, encoding='utf-8') as f:
        grid = GridMap.from_ascii(f.read(), cell_size=0.5, resolution=0.05)
    planner = GridPlanner(grid.occupied, grid.resolution, grid.origin,
                          robot_radius=0.18, inflation_radius=0.5, cost_weight=3.0)
    return grid, planner


@pytest.mark.parametrize('goal', [(10.5, 6.5), (9.5, 1.5), (2.0, 7.0)])
def test_robot_reaches_goal_without_collision(world, goal):
    grid, planner = world
    robot = DiffDriveRobot(1.0, 1.5, 0.0, radius=0.18)
    points, message = planner.plan_world((robot.x, robot.y), goal)
    assert points is not None, message

    controller = PurePursuit()
    controller.set_path(points)
    dt = 0.05
    reached = False
    for _ in range(int(120.0 / dt)):
        v, w, reached = controller.compute(robot.x, robot.y, robot.yaw)
        if reached:
            break
        assert robot.step(v, w, dt, grid), f'collision at ({robot.x:.2f}, {robot.y:.2f})'
    assert reached
    assert math.dist((robot.x, robot.y), goal) < controller.goal_tolerance
