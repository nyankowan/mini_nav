"""ROS-independent core of the 2D simulator: grid map, robot and LiDAR."""

import math

from mini_nav.geometry import normalize_angle
import numpy as np


class GridMap:
    """
    Occupancy grid map.

    ``occupied[iy, ix]`` is True for an obstacle cell. ``iy == 0`` is the
    bottom row (smallest y), which matches the nav_msgs/OccupancyGrid layout.
    """

    def __init__(self, occupied, resolution, origin=(0.0, 0.0)):
        self.occupied = np.asarray(occupied, dtype=bool)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))

    @property
    def width(self):
        return self.occupied.shape[1]

    @property
    def height(self):
        return self.occupied.shape[0]

    @classmethod
    def from_ascii(cls, text, cell_size, resolution, origin=(0.0, 0.0)):
        """
        Build a map from ASCII art ('#' = wall, anything else = free).

        One character covers ``cell_size`` metres and is upsampled to
        ``resolution``. The first line of the text is the top of the map.
        """
        lines = [line for line in text.splitlines() if line.strip()]
        if not lines:
            raise ValueError('map text is empty')
        if len({len(line) for line in lines}) != 1:
            raise ValueError('all map rows must have the same length')
        scale = int(round(cell_size / resolution))
        if scale < 1 or not math.isclose(scale * resolution, cell_size):
            raise ValueError('cell_size must be an integer multiple of resolution')
        # テキストは上の行が y の大きい側なので, 上下反転して iy=0 を下にする
        coarse = np.array([[c == '#' for c in line] for line in reversed(lines)])
        occupied = np.kron(coarse, np.ones((scale, scale), dtype=bool))
        return cls(occupied, resolution, origin)

    def world_to_grid(self, x, y):
        ix = np.floor((np.asarray(x, dtype=float) - self.origin[0]) / self.resolution)
        iy = np.floor((np.asarray(y, dtype=float) - self.origin[1]) / self.resolution)
        return ix.astype(int), iy.astype(int)

    def is_occupied(self, x, y):
        """Vectorised occupancy lookup. Points outside the map count as occupied."""
        ix, iy = self.world_to_grid(x, y)
        shape = ix.shape
        ix, iy = ix.ravel(), iy.ravel()
        inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        result = np.ones(ix.shape, dtype=bool)
        result[inside] = self.occupied[iy[inside], ix[inside]]
        return result.reshape(shape)

    def to_occupancy_data(self):
        """Flatten to nav_msgs/OccupancyGrid data (0 = free, 100 = occupied)."""
        return np.where(self.occupied.ravel(), 100, 0).tolist()


class DiffDriveRobot:
    """Differential-drive (unicycle) robot with a circular footprint."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, radius=0.18):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.radius = float(radius)
        self.v = 0.0
        self.w = 0.0
        self._ring = np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)

    def collides(self, grid, x, y):
        """Check whether the footprint centred at (x, y) overlaps a wall."""
        xs = np.append(x + self.radius * np.cos(self._ring), x)
        ys = np.append(y + self.radius * np.sin(self._ring), y)
        return bool(grid.is_occupied(xs, ys).any())

    def step(self, v, w, dt, grid=None):
        """Advance the robot by dt seconds. Returns False if a wall blocked it."""
        new_yaw = self.yaw + w * dt
        if abs(w) < 1e-9:
            nx = self.x + v * math.cos(self.yaw) * dt
            ny = self.y + v * math.sin(self.yaw) * dt
        else:
            # 一定の (v, w) では円弧を描くので, 厳密解で積分する
            r = v / w
            nx = self.x + r * (math.sin(new_yaw) - math.sin(self.yaw))
            ny = self.y - r * (math.cos(new_yaw) - math.cos(self.yaw))
        self.yaw = normalize_angle(new_yaw)
        self.w = w
        if grid is not None and self.collides(grid, nx, ny):
            # 壁にぶつかる移動は捨てる (その場回転だけは許す)
            self.v = 0.0
            return False
        self.x, self.y, self.v = nx, ny, v
        return True


class Lidar:
    """2D LiDAR simulated by marching rays through the grid."""

    def __init__(self, num_beams=360, max_range=8.0, min_range=0.1,
                 noise_std=0.0, seed=None):
        self.angles = np.linspace(-math.pi, math.pi, num_beams, endpoint=False)
        self.angle_increment = 2.0 * math.pi / num_beams
        self.max_range = float(max_range)
        self.min_range = float(min_range)
        self.noise_std = float(noise_std)
        self._rng = np.random.default_rng(seed)

    def scan(self, grid, x, y, yaw):
        """Return ranges for every beam (inf where nothing is hit)."""
        step = grid.resolution * 0.5
        dists = np.arange(self.min_range, self.max_range + 1e-9, step)
        beam = yaw + self.angles[:, None]
        # (ビーム数 x サンプル数) の点を一度に調べ, 各ビームで最初に当たった距離を採用
        occ = grid.is_occupied(x + np.cos(beam) * dists, y + np.sin(beam) * dists)
        hit = occ.any(axis=1)
        ranges = np.full(len(self.angles), np.inf)
        ranges[hit] = dists[occ.argmax(axis=1)[hit]]
        if self.noise_std > 0.0:
            ranges[hit] += self._rng.normal(0.0, self.noise_std, int(hit.sum()))
        return ranges
