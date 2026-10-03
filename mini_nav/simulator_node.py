"""
Lightweight 2D simulator node that stands in for Gazebo while learning.

Publishes:  /map (latched), /scan, /odom, /footprint, TF map->odom->base_link
Subscribes: /cmd_vel, /initialpose (RViz "2D Pose Estimate" teleports the robot)
"""

import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point32, PolygonStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped, Twist
from mini_nav.geometry import quaternion_to_yaw, yaw_to_quaternion
from mini_nav.sim_core import DiffDriveRobot, GridMap, Lidar
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _set_quaternion(q, yaw):
    q.x, q.y, q.z, q.w = yaw_to_quaternion(yaw)


class SimulatorNode(Node):

    def __init__(self):
        super().__init__('simulator')
        map_file = self.declare_parameter('map_file', '').value
        cell_size = self.declare_parameter('map_cell_size', 0.5).value
        resolution = self.declare_parameter('map_resolution', 0.05).value
        x0 = self.declare_parameter('initial_x', 1.0).value
        y0 = self.declare_parameter('initial_y', 1.5).value
        yaw0 = self.declare_parameter('initial_yaw', 0.0).value
        radius = self.declare_parameter('robot_radius', 0.18).value
        num_beams = self.declare_parameter('scan_num_beams', 360).value
        max_range = self.declare_parameter('scan_max_range', 8.0).value
        noise_std = self.declare_parameter('scan_noise_std', 0.01).value
        scan_rate = self.declare_parameter('scan_rate', 10.0).value
        sim_rate = self.declare_parameter('sim_rate', 50.0).value
        self.cmd_timeout = self.declare_parameter('cmd_vel_timeout', 0.5).value
        # SLAM (slam_toolbox など) を試すときは, この2つを false にする
        publish_map = self.declare_parameter('publish_map', True).value
        publish_map_to_odom = self.declare_parameter('publish_map_to_odom', True).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        if not map_file:
            map_file = os.path.join(
                get_package_share_directory('mini_nav'), 'maps', 'simple_room.txt')
        with open(map_file, encoding='utf-8') as f:
            self.grid = GridMap.from_ascii(f.read(), cell_size, resolution)
        self.robot = DiffDriveRobot(x0, y0, yaw0, radius)
        if self.robot.collides(self.grid, x0, y0):
            self.get_logger().error(f'Initial pose ({x0}, {y0}) overlaps a wall')
        self.lidar = Lidar(num_beams, max_range, noise_std=noise_std)
        self.scan_period = 1.0 / scan_rate
        self.dt = 1.0 / sim_rate

        self.cmd = (0.0, 0.0)
        self.last_cmd_time = None
        # 最後に TF として送信した (時刻, x, y, yaw)
        self.last_published_pose = None
        self.blocked = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', latched)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.footprint_pub = self.create_publisher(PolygonStamped, 'footprint', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.on_initial_pose, 10)

        if publish_map_to_odom:
            self.publish_static_map_to_odom()
        if publish_map:
            self.publish_map()
        self.create_timer(self.dt, self.on_sim_timer)
        self.create_timer(self.scan_period, self.on_scan_timer)
        self.get_logger().info(
            f'Simulator ready: map {self.grid.width}x{self.grid.height} cells '
            f'@ {resolution} m, robot at ({x0:.2f}, {y0:.2f})')

    # ---- callbacks -------------------------------------------------------

    def on_cmd_vel(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def on_initial_pose(self, msg):
        p = msg.pose.pose
        x, y = p.position.x, p.position.y
        if self.robot.collides(self.grid, x, y):
            self.get_logger().warn(f'Cannot teleport to ({x:.2f}, {y:.2f}): wall')
            return
        o = p.orientation
        # map->odom は恒等変換なので, map 上の姿勢をそのままロボットに設定してよい
        self.robot.x, self.robot.y = x, y
        self.robot.yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)
        self.get_logger().info(f'Teleported robot to ({x:.2f}, {y:.2f})')

    def on_sim_timer(self):
        now = self.get_clock().now()
        v, w = self.cmd
        # cmd_vel が途絶えたら止まる (実機のモータドライバでもよくある安全機構)
        if (self.last_cmd_time is None
                or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout):
            v, w = 0.0, 0.0
        moved = self.robot.step(v, w, self.dt, self.grid)
        if not moved and not self.blocked:
            self.get_logger().warn('Robot bumped into a wall')
        self.blocked = not moved
        self.publish_odom(now)

    def on_scan_timer(self):
        if self.last_published_pose is None:
            return
        # センサデータには, 送信済みの TF と同じ時刻・同じ姿勢を使う.
        # 受信側 (RViz など) はその時刻の TF をすでに持っているので待たずに変換できる.
        # 「今」の時刻を付けると TF がまだ届いておらず, 受信側で変換待ちが発生する
        # (Humble の RViz はこの待ちの処理でデッドロックすることがある).
        stamp, x, y, yaw = self.last_published_pose
        r = self.robot
        ranges = self.lidar.scan(self.grid, x, y, yaw)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min = float(self.lidar.angles[0])
        scan.angle_max = float(self.lidar.angles[-1])
        scan.angle_increment = self.lidar.angle_increment
        scan.scan_time = self.scan_period
        scan.range_min = self.lidar.min_range
        scan.range_max = self.lidar.max_range
        scan.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(scan)

        footprint = PolygonStamped()
        footprint.header.stamp = stamp
        footprint.header.frame_id = self.base_frame
        angles = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
        footprint.polygon.points = [
            Point32(x=float(r.radius * np.cos(a)), y=float(r.radius * np.sin(a)))
            for a in angles]
        # 向きがわかるように, 前方に「鼻」を付ける
        footprint.polygon.points.insert(0, Point32(x=float(r.radius * 1.4)))
        self.footprint_pub.publish(footprint)

    # ---- publishers ------------------------------------------------------

    def publish_odom(self, now):
        r = self.robot
        stamp = now.to_msg()

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = r.x
        tf.transform.translation.y = r.y
        _set_quaternion(tf.transform.rotation, r.yaw)
        self.tf_broadcaster.sendTransform(tf)
        self.last_published_pose = (stamp, r.x, r.y, r.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = r.x
        odom.pose.pose.position.y = r.y
        _set_quaternion(odom.pose.pose.orientation, r.yaw)
        odom.twist.twist.linear.x = r.v
        odom.twist.twist.angular.z = r.w
        self.odom_pub.publish(odom)

    def publish_static_map_to_odom(self):
        # 自己位置推定 (AMCL など) の代わり: 完全なオドメトリなので map と odom は一致する
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = self.grid.origin[0]
        msg.info.origin.position.y = self.grid.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.to_occupancy_data()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
