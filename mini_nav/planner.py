"""Grid-based global planner: A* on an inflated occupancy grid."""

import heapq
import math

import numpy as np

_SQRT2 = math.sqrt(2.0)
_NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2),
]


def _dilate(mask):
    """Grow a boolean mask by one cell in all 8 directions."""
    rows = mask.copy()
    rows[1:, :] |= mask[:-1, :]
    rows[:-1, :] |= mask[1:, :]
    out = rows.copy()
    out[:, 1:] |= rows[:, :-1]
    out[:, :-1] |= rows[:, 1:]
    return out


def obstacle_distance(occupied, max_distance):
    """Chebyshev distance in cells to the nearest obstacle (capped at max_distance + 1)."""
    dist = np.full(occupied.shape, max_distance + 1, dtype=np.int32)
    frontier = np.asarray(occupied, dtype=bool)
    dist[frontier] = 0
    for d in range(1, max_distance + 1):
        grown = _dilate(frontier)
        dist[grown & ~frontier] = d
        frontier = grown
    return dist


class GridPlanner:
    """
    A* planner that keeps the robot footprint away from walls.

    Two layers are derived from the map (like a Nav2 costmap):
      * lethal: cells where the robot centre must never be (wall + robot radius)
      * cost:   soft penalty that pushes the path towards the middle of free space
    """

    def __init__(self, occupied, resolution, origin=(0.0, 0.0),
                 robot_radius=0.18, inflation_radius=0.5, cost_weight=3.0):
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.robot_cells = int(math.ceil(robot_radius / self.resolution))
        inflation_cells = max(int(math.ceil(inflation_radius / self.resolution)),
                              self.robot_cells)
        distance = obstacle_distance(np.asarray(occupied, dtype=bool), inflation_cells)
        self.lethal = distance <= self.robot_cells
        span = inflation_cells - self.robot_cells + 1
        self.cost = cost_weight * np.clip((inflation_cells + 1 - distance) / span, 0.0, 1.0)
        # A* の内側ループでは numpy の要素アクセスより list の方が速い
        self._lethal_list = self.lethal.tolist()
        self._cost_list = self.cost.tolist()

    @property
    def shape(self):
        return self.lethal.shape

    def world_to_cell(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def cell_to_world(self, cell):
        return (self.origin[0] + (cell[0] + 0.5) * self.resolution,
                self.origin[1] + (cell[1] + 0.5) * self.resolution)

    def in_bounds(self, cell):
        height, width = self.shape
        return 0 <= cell[0] < width and 0 <= cell[1] < height

    def is_free(self, cell):
        return self.in_bounds(cell) and not self._lethal_list[cell[1]][cell[0]]

    def nearest_free(self, cell, max_radius):
        """Closest non-lethal cell within max_radius (square rings), or None."""
        if self.is_free(cell):
            return cell
        cx, cy = cell
        for r in range(1, max_radius + 1):
            ring = [(cx + dx, cy + dy)
                    for dx in range(-r, r + 1) for dy in range(-r, r + 1)
                    if max(abs(dx), abs(dy)) == r]
            free = [c for c in ring if self.is_free(c)]
            if free:
                return min(free, key=lambda c: (c[0] - cx) ** 2 + (c[1] - cy) ** 2)
        return None

    def plan(self, start, goal):
        """Run A* between two free cells. Returns a list of (ix, iy) or None."""
        lethal, cost = self._lethal_list, self._cost_list
        height, width = self.shape
        gx, gy = goal

        def heuristic(cell):
            # 1 マス進むコストは最低 1 なので, ユークリッド距離は許容的なヒューリスティック
            return math.hypot(cell[0] - gx, cell[1] - gy)

        g_score = {start: 0.0}
        came_from = {}
        closed = set()
        open_heap = [(heuristic(start), 0.0, start)]
        while open_heap:
            _, g, current = heapq.heappop(open_heap)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]
            if current in closed:
                continue
            closed.add(current)
            cx, cy = current
            for dx, dy, step in _NEIGHBORS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height) or lethal[ny][nx]:
                    continue
                # 斜め移動で壁の角をすり抜けないようにする
                if dx and dy and (lethal[cy][nx] or lethal[ny][cx]):
                    continue
                new_g = g + step * (1.0 + cost[ny][nx])
                if new_g < g_score.get((nx, ny), math.inf):
                    g_score[(nx, ny)] = new_g
                    came_from[(nx, ny)] = current
                    heapq.heappush(open_heap, (new_g + heuristic((nx, ny)), new_g, (nx, ny)))
        return None

    def plan_world(self, start_xy, goal_xy):
        """Plan between world coordinates. Returns (points or None, message)."""
        start = self.world_to_cell(*start_xy)
        goal = self.world_to_cell(*goal_xy)
        if not self.in_bounds(goal):
            return None, 'goal is outside the map'
        if not self.is_free(goal):
            return None, 'goal is inside or too close to an obstacle'
        if not self.in_bounds(start):
            return None, 'robot is outside the map'
        # 離散化の都合でロボット中心が lethal に入ることがあるので, 近くの空きマスから始める
        start = self.nearest_free(start, max_radius=2 * self.robot_cells)
        if start is None:
            return None, 'robot is stuck inside an obstacle'
        cells = self.plan(start, goal)
        if cells is None:
            return None, 'no path found'
        points = [self.cell_to_world(c) for c in cells]
        points[-1] = (float(goal_xy[0]), float(goal_xy[1]))
        return points, 'ok'
