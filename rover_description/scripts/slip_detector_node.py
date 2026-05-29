#!/usr/bin/env python3
"""Slip detection — compares wheel odom vs optical flow, inflates covariance on slip.

Subscribes:
    /odom (nav_msgs/Odometry) — raw wheel odometry
    /optical_flow/corrected (nav_msgs/Odometry) — de-rotated optical flow

Publishes:
    /odom/adjusted (nav_msgs/Odometry) — wheel odom with dynamic covariance
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class SlipDetectorNode(Node):
    def __init__(self):
        super().__init__('slip_detector_node')

        self.declare_parameter('slip_threshold', 0.5)      # TUNE: ratio above which = slip
        self.declare_parameter('nominal_cov_vx', 0.1)      # TUNE: normal wheel trust
        self.declare_parameter('nominal_cov_wz', 0.05)     # TUNE: normal yaw rate trust
        self.declare_parameter('slip_cov', 1e6)             # covariance when slipping

        self.slip_threshold = self.get_parameter('slip_threshold').value
        self.nominal_vx = self.get_parameter('nominal_cov_vx').value
        self.nominal_wz = self.get_parameter('nominal_cov_wz').value
        self.slip_cov = self.get_parameter('slip_cov').value

        self.flow_vx = 0.0
        self.last_flow_time = self.get_clock().now()
        self.flow_active = False

        self.sub_flow = self.create_subscription(
            Odometry, '/optical_flow/corrected', self.flow_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 50)
        self.pub = self.create_publisher(Odometry, '/odom/adjusted', 50)

        self.get_logger().info('Slip detector node started')

    def flow_cb(self, msg: Odometry):
        self.flow_vx = msg.twist.twist.linear.x
        self.last_flow_time = self.get_clock().now()
        self.flow_active = True

    def odom_cb(self, msg: Odometry):
        wheel_vx = msg.twist.twist.linear.x

        time_since_flow = (self.get_clock().now() - self.last_flow_time).nanoseconds / 1e9

        # Slip = wheels commanding motion but rover not moving.
        # If wheels report near-zero, no slip possible regardless of flow noise.
        MIN_WHEEL_SPEED = 0.05
        if abs(wheel_vx) < MIN_WHEEL_SPEED or not self.flow_active or time_since_flow > 0.5:
            slip_ratio = 0.0
        else:
            denom = max(abs(wheel_vx), abs(self.flow_vx), 0.01)
            slip_ratio = abs(wheel_vx - self.flow_vx) / denom

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.twist.twist = msg.twist.twist
        out.pose = msg.pose

        if slip_ratio > self.slip_threshold:
            # SLIP DETECTED — inflate covariance, EKF will ignore wheels
            out.twist.covariance[0] = self.slip_cov    # vx
            out.twist.covariance[35] = self.slip_cov   # wz
            self.get_logger().info(
                f'SLIP detected: ratio={slip_ratio:.2f}, inflating covariance',
                throttle_duration_sec=1.0)
        else:
            out.twist.covariance[0] = self.nominal_vx
            out.twist.covariance[35] = self.nominal_wz

        # Fill unused axes with large covariance
        out.twist.covariance[7] = 1e6    # vy
        out.twist.covariance[14] = 1e6   # vz
        out.twist.covariance[21] = 1e6   # wx
        out.twist.covariance[28] = 1e6   # wy

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SlipDetectorNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
