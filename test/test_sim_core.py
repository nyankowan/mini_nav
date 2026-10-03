import math

from mini_nav.sim_core import DiffDriveRobot, GridMap, Lidar
import numpy as np
import pytest

BOX = """
#####
#...#
#...#
#...#
#####
"""


@pytest.fixture
def box():
    # 2.5 m 四方, 内側の空間は x, y ともに [0.5, 2.0]
    return GridMap.from_ascii(BOX, cell_size=0.5, resolution=0.05)


def test_from_ascii_bottom_row_is_first():
    grid = GridMap.from_ascii('#.\n..', cell_size=1.0, resolution=1.0)
    # テキストの1行目は地図の上側 (iy = 1)
    assert grid.occupied[1, 0]
    assert not grid.occupied[0, 0]


def test_from_ascii_rejects_ragged_rows():
    with pytest.raises(ValueError):
        GridMap.from_ascii('###\n##', cell_size=0.5, resolution=0.05)


def test_is_occupied_outside_map(box):
    assert box.is_occupied(-1.0, 1.0)
    assert box.is_occupied(1.0, 100.0)
    assert not box.is_occupied(1.25, 1.25)


def test_lidar_measures_distance_to_walls(box):
    lidar = Lidar(num_beams=360, max_range=5.0)
    ranges = lidar.scan(box, 1.25, 1.25, 0.0)
    front = ranges[np.argmin(np.abs(lidar.angles))]
    left = ranges[np.argmin(np.abs(lidar.angles - math.pi / 2))]
    assert front == pytest.approx(0.75, abs=0.03)
    assert left == pytest.approx(0.75, abs=0.03)


def test_lidar_rotates_with_robot(box):
    lidar = Lidar(num_beams=360, max_range=5.0)
    ranges = lidar.scan(box, 1.0, 1.25, math.pi / 2)
    # 左を向いているので, 前方ビームは +y 方向の壁 (0.75 m), 右ビームは +x 方向の壁 (1.0 m)
    assert ranges[np.argmin(np.abs(lidar.angles))] == pytest.approx(0.75, abs=0.03)
    assert ranges[np.argmin(np.abs(lidar.angles + math.pi / 2))] == pytest.approx(1.0, abs=0.03)


def test_robot_drives_straight():
    robot = DiffDriveRobot(0.0, 0.0, 0.0)
    for _ in range(10):
        robot.step(0.5, 0.0, 0.1)
    assert robot.x == pytest.approx(0.5)
    assert robot.y == pytest.approx(0.0)


def test_robot_drives_quarter_circle():
    robot = DiffDriveRobot(0.0, 0.0, 0.0)
    # 半径 1 m の円弧を 1/4 周
    for _ in range(100):
        robot.step(1.0, 1.0, math.pi / 2 / 100)
    assert robot.x == pytest.approx(1.0)
    assert robot.y == pytest.approx(1.0)
    assert robot.yaw == pytest.approx(math.pi / 2)


def test_robot_is_blocked_by_wall(box):
    robot = DiffDriveRobot(1.25, 1.25, 0.0, radius=0.18)
    for _ in range(100):
        robot.step(0.5, 0.0, 0.1, box)
    # 壁 (x = 2.0) から半径ぶん手前で止まる
    assert robot.x <= 2.0 - robot.radius
    assert robot.x > 1.7
    assert robot.v == 0.0
