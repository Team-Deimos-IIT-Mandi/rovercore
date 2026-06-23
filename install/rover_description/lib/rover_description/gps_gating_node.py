#!/usr/bin/env python3
"""Mahalanobis gating — 3-sigma gate on GPS, rejects multipath.

Subscribes:
    /gps/odom (nav_msgs/Odometry) — from navsat_transform
    /odometry/local (nav_msgs/Odometry) — from EKF Local

Publishes:
    /gps/odom_gated (nav_msgs/Odometry) — only passes if within gate
"""

import math
import copy
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class GpsGatingNode(Node):
    def __init__(self):
        super().__init__('gps_gating_node')

        self.declare_parameter('gate_sigma', 3.0)
        self.declare_parameter('gps_position_cov', 4.0)   # x/y variance in m^2
        self.declare_parameter('gps_z_cov', 100.0)
        self.declare_parameter('gps_yaw_cov', 100.0)

        self.gate_sigma = float(self.get_parameter('gate_sigma').value)
        self.gps_cov = float(self.get_parameter('gps_position_cov').value)
        self.gps_z_cov = float(self.get_parameter('gps_z_cov').value)
        self.gps_yaw_cov = float(self.get_parameter('gps_yaw_cov').value)

        self.local_x = 0.0
        self.local_y = 0.0
        self.local_cov_x = 1.0
        self.local_cov_y = 1.0

        self.sub_local = self.create_subscription(
            Odometry, '/odometry/global', self.local_cb, 10)
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

        dx = gps_x - self.local_x
        dy = gps_y - self.local_y

        s_x = max(self.local_cov_x, 0.01) + self.gps_cov
        s_y = max(self.local_cov_y, 0.01) + self.gps_cov

        d2 = (dx * dx / s_x) + (dy * dy / s_y)
        d = math.sqrt(d2)

        if d <= self.gate_sigma:
            out = copy.deepcopy(msg)

            # Do not leave covariance as zeros.
            cov = [0.0] * 36
            cov[0] = self.gps_cov       # x variance
            cov[7] = self.gps_cov       # y variance
            cov[14] = self.gps_z_cov    # z variance
            cov[21] = 1e6               # roll variance
            cov[28] = 1e6               # pitch variance
            cov[35] = self.gps_yaw_cov  # yaw variance
            out.pose.covariance = cov

            self.pub.publish(out)
        else:
            self.get_logger().warn(
                f'GPS REJECTED: Mahalanobis d={d:.2f} > {self.gate_sigma}σ '
                f'(jump={math.sqrt(dx*dx+dy*dy):.1f}m)',
                throttle_duration_sec=2.0)
            self.get_logger().info(
                f"GPS=({gps_x:.2f},{gps_y:.2f}) "
                f"GLOBAL=({self.local_x:.2f},{self.local_y:.2f}) "
                f"dx={dx:.2f} dy={dy:.2f} "
                f"cov=({self.local_cov_x:.2f},{self.local_cov_y:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GpsGatingNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
