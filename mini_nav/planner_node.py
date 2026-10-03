"""
Global planner node: plans a path on /map to every /goal_pose.

Subscribes: /map (latched), /goal_pose (RViz "2D Goal Pose"), TF map->base_link
Publishes:  /plan (nav_msgs/Path, empty when planning fails)
"""

import math
import time

from geometry_msgs.msg import PoseStamped
from mini_nav.geometry import yaw_to_quaternion
from mini_nav.planner import GridPlanner
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformException, TransformListener


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner')
        self.robot_radius = self.declare_parameter('robot_radius', 0.18).value
        self.inflation_radius = self.declare_parameter('inflation_radius', 0.5).value
        self.cost_weight = self.declare_parameter('cost_weight', 3.0).value
        self.occupied_threshold = self.declare_parameter('occupied_threshold', 50).value
        self.unknown_is_obstacle = self.declare_parameter('unknown_is_obstacle', False).value
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value

        self.planner = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, 'plan', 10)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, latched)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 10)

    def on_map(self, msg):
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        occupied = data >= self.occupied_threshold
        if self.unknown_is_obstacle:
            occupied |= data < 0
        t0 = time.perf_counter()
        self.planner = GridPlanner(
            occupied, info.resolution,
            (info.origin.position.x, info.origin.position.y),
            robot_radius=self.robot_radius,
            inflation_radius=self.inflation_radius,
            cost_weight=self.cost_weight)
        self.get_logger().info(
            f'Map received ({info.width}x{info.height}), costmap built in '
            f'{(time.perf_counter() - t0) * 1e3:.0f} ms')

    def on_goal(self, msg):
        if self.planner is None:
            self.get_logger().warn('No map yet, ignoring goal')
            return
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f'Goal must be in "{self.global_frame}" frame (got "{msg.header.frame_id}")')
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'Robot pose unavailable: {e}')
            return

        start = (tf.transform.translation.x, tf.transform.translation.y)
        goal = (msg.pose.position.x, msg.pose.position.y)
        t0 = time.perf_counter()
        points, message = self.planner.plan_world(start, goal)
        elapsed_ms = (time.perf_counter() - t0) * 1e3

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.global_frame
        if points is None:
            self.get_logger().warn(f'Planning to ({goal[0]:.2f}, {goal[1]:.2f}) failed: {message}')
            self.path_pub.publish(path)  # 空の経路 = 停止指令
            return

        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if i + 1 < len(points):
                nx, ny = points[i + 1]
                q = yaw_to_quaternion(math.atan2(ny - y, nx - x))
                pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
                pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
            else:
                pose.pose.orientation = msg.pose.orientation
            path.poses.append(pose)
        self.path_pub.publish(path)

        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        self.get_logger().info(
            f'Planned path to ({goal[0]:.2f}, {goal[1]:.2f}): '
            f'{length:.2f} m, {len(points)} poses, {elapsed_ms:.0f} ms')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
