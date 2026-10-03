"""
Path-following controller node (pure pursuit + simple emergency stop).

Subscribes: /plan, /scan, TF map->base_link
Publishes:  /cmd_vel
"""

import math

from geometry_msgs.msg import Twist
from mini_nav.controller import PurePursuit
from mini_nav.geometry import normalize_angle, quaternion_to_yaw
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')
        self.controller = PurePursuit(
            lookahead=self.declare_parameter('lookahead_distance', 0.5).value,
            max_linear=self.declare_parameter('max_linear_speed', 0.4).value,
            max_angular=self.declare_parameter('max_angular_speed', 1.5).value,
            goal_tolerance=self.declare_parameter('goal_tolerance', 0.1).value,
            slow_down_radius=self.declare_parameter('slow_down_radius', 0.6).value,
            rotate_threshold=math.radians(
                self.declare_parameter('rotate_in_place_angle_deg', 60.0).value))
        self.stop_distance = self.declare_parameter('stop_distance', 0.35).value
        self.front_angle = math.radians(self.declare_parameter('front_angle_deg', 30.0).value)
        control_rate = self.declare_parameter('control_rate', 20.0).value
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value

        self.front_min = math.inf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_timer(1.0 / control_rate, self.on_timer)

    def on_plan(self, msg):
        if not msg.poses:
            if self.controller.has_path:
                self.get_logger().info('Received empty plan, stopping')
            self.controller.clear()
            self.publish_cmd(0.0, 0.0)
            return
        self.controller.set_path([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        self.get_logger().info(f'Following new path ({len(msg.poses)} poses)')

    def on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        front = np.abs(normalize_angle(angles)) <= self.front_angle
        valid = front & np.isfinite(ranges) & (ranges >= msg.range_min)
        self.front_min = float(ranges[valid].min()) if valid.any() else math.inf

    def on_timer(self):
        if not self.controller.has_path:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'Robot pose unavailable: {e}', throttle_duration_sec=2.0)
            self.publish_cmd(0.0, 0.0)
            return
        t, q = tf.transform.translation, tf.transform.rotation
        v, w, reached = self.controller.compute(t.x, t.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
        if reached:
            self.get_logger().info('Goal reached')
            self.controller.clear()
            self.publish_cmd(0.0, 0.0)
            return
        # 地図にない障害物が目の前に現れたら前進だけ止める (旋回は許す)
        if v > 0.0 and self.front_min < self.stop_distance:
            self.get_logger().warn(
                f'Obstacle {self.front_min:.2f} m ahead, holding', throttle_duration_sec=2.0)
            v = 0.0
        self.publish_cmd(v, w)

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
