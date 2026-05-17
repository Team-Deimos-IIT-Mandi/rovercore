#!/usr/bin/env python3
"""Mahalanobis gating — 3-sigma gate on GPS, rejects multipath.

Subscribes:
    /gps/odom (nav_msgs/Odometry) — from navsat_transform
    /odometry/local (nav_msgs/Odometry) — from EKF Local

Publishes:
    /gps/odom_gated (nav_msgs/Odometry) — only passes if within 3σ
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class GpsGatingNode(Node):
    def __init__(self):
        super().__init__('gps_gating_node')

        self.declare_parameter('gate_sigma', 3.0)           # TUNE: sigma threshold
        self.declare_parameter('gps_position_cov', 0.25)    # TUNE: GPS variance (0.5m accuracy → 0.25 m²)

        self.gate_sigma = self.get_parameter('gate_sigma').value
        self.gps_cov = self.get_parameter('gps_position_cov').value

        self.local_x = 0.0
        self.local_y = 0.0
        self.local_cov_x = 1.0
        self.local_cov_y = 1.0

        self.sub_local = self.create_subscription(
            Odometry, '/odometry/local', self.local_cb, 10)
        self.sub_gps = self.create_subscription(
            Odometry, '/gps/odom', self.gps_cb, 10)
        self.pub = self.create_publisher(Odometry, '/gps/odom_gated', 10)

        self.get_logger().info(f'GPS gating node started (gate={self.gate_sigma}σ)')

    def local_cb(self, msg: Odometry):
        self.local_x = msg.pose.pose.position.x
        self.local_y = msg.pose.pose.position.y
        self.local_cov_x = msg.pose.covariance[0]
        self.local_cov_y = msg.pose.covariance[7]

    def gps_cb(self, msg: Odometry):
        gps_x = msg.pose.pose.position.x
        gps_y = msg.pose.pose.position.y

        # Innovation (difference between GPS and EKF estimate)
        dx = gps_x - self.local_x
        dy = gps_y - self.local_y

        # Innovation covariance = EKF cov + GPS cov
        s_x = max(self.local_cov_x, 0.01) + self.gps_cov
        s_y = max(self.local_cov_y, 0.01) + self.gps_cov

        # Mahalanobis distance (simplified, assumes diagonal covariance)
        d2 = (dx * dx / s_x) + (dy * dy / s_y)
        d = math.sqrt(d2)

        threshold = self.gate_sigma

        if d <= threshold:
            self.pub.publish(msg)
        else:
            self.get_logger().warn(
                f'GPS REJECTED: Mahalanobis d={d:.2f} > {threshold}σ '
                f'(jump={math.sqrt(dx*dx+dy*dy):.1f}m)',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GpsGatingNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
