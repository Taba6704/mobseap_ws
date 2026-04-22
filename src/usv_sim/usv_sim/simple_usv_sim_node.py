#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster

from builtin_interfaces.msg import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile


class SimpleUsvSimNode(Node):
    def __init__(self):
        super().__init__('simple_usv_sim_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/sim_path')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)

        self.declare_parameter('publish_path', True)
        self.declare_parameter('path_publish_every_n', 5)
        self.declare_parameter('max_path_points', 1000)

        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)

        self.publish_path = bool(self.get_parameter('publish_path').value)
        self.path_publish_every_n = int(self.get_parameter('path_publish_every_n').value)
        self.max_path_points = int(self.get_parameter('max_path_points').value)

        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)

        # -----------------------------
        # State
        # -----------------------------
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.path_counter = 0

        # -----------------------------
        # Interfaces
        # -----------------------------
        qos = QoSProfile(depth=10)

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            qos
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, qos)
        self.path_pub = self.create_publisher(Path, self.path_topic, qos)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame

        timer_period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(timer_period, self.update)

        self.get_logger().info('Simple USV sim node started')
        self.get_logger().info(f'Subscribing to: {self.cmd_vel_topic}')
        self.get_logger().info(f'Publishing odom on: {self.odom_topic}')
        self.get_logger().info(f'Publishing path on: {self.path_topic}')
        self.get_logger().info(f'TF: {self.odom_frame} -> {self.base_frame}')

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.current_linear = msg.linear.x * self.linear_scale
        self.current_angular = msg.angular.z * self.angular_scale
        self.last_cmd_time = self.get_clock().now()

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt <= 0.0:
            return

        # Watchdog timeout: stop motion if cmd_vel gets stale
        if (now - self.last_cmd_time) > Duration(seconds=self.cmd_timeout_sec):
            linear = 0.0
            angular = 0.0
        else:
            linear = self.current_linear
            angular = self.current_angular

        # Simple planar kinematics
        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt
        self.yaw += angular * dt
        self.yaw = self.normalize_angle(self.yaw)

        stamp = now.to_msg()

        self.publish_tf(stamp)
        self.publish_odom(stamp, linear, angular)

        if self.publish_path:
            self.path_counter += 1
            if self.path_counter >= self.path_publish_every_n:
                self.path_counter = 0
                self.append_and_publish_path(stamp)

    def publish_tf(self, stamp: Time) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        qx, qy, qz, qw = self.quaternion_from_yaw(self.yaw)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_odom(self, stamp: Time, linear: float, angular: float) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.quaternion_from_yaw(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular

        self.odom_pub.publish(odom)

    def append_and_publish_path(self, stamp: Time) -> None:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.odom_frame

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.quaternion_from_yaw(self.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)

        if len(self.path_msg.poses) > self.max_path_points:
            self.path_msg.poses.pop(0)

        self.path_pub.publish(self.path_msg)

    @staticmethod
    def quaternion_from_yaw(yaw: float):
        half_yaw = yaw * 0.5
        qx = 0.0
        qy = 0.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        return qx, qy, qz, qw

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = SimpleUsvSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()