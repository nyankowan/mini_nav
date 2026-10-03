"""Pure pursuit path-following controller."""

import math

import numpy as np


class PurePursuit:
    """
    Follow a polyline by steering towards a point ``lookahead`` metres ahead.

    The curvature that brings the robot onto the lookahead point is
    ``kappa = 2 * y_local / L^2`` and the angular velocity is ``w = v * kappa``.
    """

    def __init__(self, lookahead=0.5, max_linear=0.4, max_angular=1.5,
                 goal_tolerance=0.1, slow_down_radius=0.6,
                 rotate_threshold=math.radians(60.0), min_linear=0.05):
        self.lookahead = lookahead
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.goal_tolerance = goal_tolerance
        self.slow_down_radius = slow_down_radius
        self.rotate_threshold = rotate_threshold
        self.min_linear = min_linear
        self.path = None
        self._index = 0

    @property
    def has_path(self):
        return self.path is not None

    def set_path(self, points):
        self.path = np.asarray(points, dtype=float).reshape(-1, 2)
        self._index = 0

    def clear(self):
        self.path = None
        self._index = 0

    def compute(self, x, y, yaw):
        """Return (linear velocity, angular velocity, goal reached)."""
        if self.path is None or len(self.path) == 0:
            return 0.0, 0.0, True
        position = np.array([x, y])
        goal = self.path[-1]
        dist_to_goal = float(np.hypot(*(goal - position)))
        if dist_to_goal < self.goal_tolerance:
            return 0.0, 0.0, True

        # 経路上の最近点を前方にだけ探す (後戻りや, 交差する経路での飛びを防ぐ)
        window = self.path[self._index:self._index + 60]
        self._index += int(np.argmin(np.hypot(*(window - position).T)))

        # 最近点から先で, ロボットから lookahead 以上離れた最初の点を目標にする
        remaining = self.path[self._index:]
        far = np.nonzero(np.hypot(*(remaining - position).T) >= self.lookahead)[0]
        target = remaining[far[0]] if far.size else goal

        # 目標点をロボット座標系に変換
        dx, dy = target - position
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(local_y, local_x)

        if abs(alpha) > self.rotate_threshold:
            # 目標が横や後ろにあるときは, まずその場で向きを合わせる
            return 0.0, float(np.clip(2.0 * alpha, -self.max_angular, self.max_angular)), False

        curvature = 2.0 * local_y / (local_x ** 2 + local_y ** 2)
        v = self.max_linear * min(1.0, dist_to_goal / self.slow_down_radius)
        v = max(v, self.min_linear)
        w = v * curvature
        if abs(w) > self.max_angular:
            # 角速度が上限を超えるなら, 曲率を保ったまま減速する
            w = math.copysign(self.max_angular, w)
            v = self.max_angular / abs(curvature)
        return v, w, False
