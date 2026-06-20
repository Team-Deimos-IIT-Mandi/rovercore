#!/usr/bin/env python3
"""ZUPT — Zero Velocity Update when rover is stationary.

Subscribes:
    /imu/filtered (sensor_msgs/Imu) — angular velocity magnitude
    /odom (nav_msgs/Odometry) — wheel velocity magnitude

Publishes:
    /constraints/zupt (nav_msgs/Odometry) — zero velocity with tiny covariance
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class ZuptNode(Node):
    def __init__(self):
        super().__init__('zupt_node')

        self.declare_parameter('gyro_threshold', 0.02)      # TUNE: rad/s below = "still"
        self.declare_parameter('wheel_threshold', 0.02)      # TUNE: m/s below = "still"
        self.declare_parameter('hold_time', 0.5)             # TUNE: seconds of stillness before ZUPT
        self.declare_parameter('zupt_covariance', 1e-4)      # TUNE: how tight the zero clamp is

        self.gyro_thresh = self.get_parameter('gyro_threshold').value
        self.wheel_thresh = self.get_parameter('wheel_threshold').value
        self.hold_time = self.get_parameter('hold_time').value
        self.zupt_cov = self.get_parameter('zupt_covariance').value

        self.gyro_still = False
        self.wheel_still = False
        self.still_since = None
        self.zupt_active = False

        self.sub_imu = self.create_subscription(Imu, '/imu/filtered', self.imu_cb, 50)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.pub = self.create_publisher(Odometry, '/constraints/zupt', 10)
        self.timer = self.create_timer(1.0 / 50.0, self.check_and_publish)

        self.get_logger().info('ZUPT node started')

    def imu_cb(self, msg: Imu):
        w = msg.angular_velocity
        magnitude = math.sqrt(w.x**2 + w.y**2 + w.z**2)
        self.gyro_still = magnitude < self.gyro_thresh

    def odom_cb(self, msg: Odometry):
        v = msg.twist.twist.linear
        magnitude = math.sqrt(v.x**2 + v.y**2)
        self.wheel_still = magnitude < self.wheel_thresh

    def check_and_publish(self):
        now = self.get_clock().now()

        if self.gyro_still and self.wheel_still:
            if self.still_since is None:
                self.still_since = now
            elapsed = (now - self.still_since).nanoseconds * 1e-9
            if elapsed >= self.hold_time:
                if not self.zupt_active:
                    self.get_logger().info('ZUPT activated — clamping velocity to zero')
                self.zupt_active = True
                self._publish_zero(now)
        else:
            if self.zupt_active:
                self.get_logger().info('ZUPT deactivated — rover moving')
            self.still_since = None
            self.zupt_active = False

    def _publish_zero(self, now):
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        # All twist values default to 0
        c = self.zupt_cov
        msg.twist.covariance[0] = c    # vx
        msg.twist.covariance[7] = c    # vy
        msg.twist.covariance[14] = c   # vz
        msg.twist.covariance[21] = c   # wx
        msg.twist.covariance[28] = c   # wy
        msg.twist.covariance[35] = c   # wz
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ZuptNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
