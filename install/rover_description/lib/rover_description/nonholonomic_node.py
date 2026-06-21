#!/usr/bin/env python3
"""Non-holonomic constraint — publishes Vy=0, Vz=0 at 50Hz.
The rover can't slide sideways. This kills lateral drift from IMU noise.

Publishes: /constraints/nonholonomic (nav_msgs/Odometry)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time


class NonHolonomicNode(Node):
    def __init__(self):
        super().__init__('nonholonomic_node')
        self.pub = self.create_publisher(Odometry, '/constraints/nonholonomic', 10)
        self.timer = self.create_timer(1.0 / 50.0, self.publish_constraint)  # 50Hz
        self.get_logger().info('Non-holonomic constraint node started (Vy=0, Vz=0)')

    def publish_constraint(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        # All twist values are 0 by default
        # Set large covariance on axes we DON'T constrain (EKF ignores them)
        msg.twist.covariance[0] = 1e6     # vx — not constrained
        msg.twist.covariance[7] = 1e-6    # TUNE: vy covariance (tighter = stronger constraint)
        msg.twist.covariance[14] = 1e-6   # TUNE: vz covariance
        msg.twist.covariance[21] = 1e6    # wx — not constrained
        msg.twist.covariance[28] = 1e6    # wy — not constrained
        msg.twist.covariance[35] = 1e6    # wz — not constrained
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NonHolonomicNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
