#!/usr/bin/env python3
"""Optical flow de-rotation — removes false velocity from rover pitch/roll.

Vx_true = Vx_raw - (omega_y * Z_tof)
Vy_true = Vy_raw + (omega_x * Z_tof)

Subscribes:
    /optical_flow/odom (nav_msgs/Odometry) — raw flow velocity
    /imu/filtered (sensor_msgs/Imu) — angular velocity
    /range/height (sensor_msgs/Range) — height above ground

Publishes:
    /optical_flow/corrected (nav_msgs/Odometry) — de-rotated flow velocity
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range


class FlowDerotationNode(Node):
    def __init__(self):
        super().__init__('flow_derotation_node')

        self.height = 0.3  # default
        self.omega_x = 0.0
        self.omega_y = 0.0

        best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub_imu = self.create_subscription(
            Imu, '/imu/filtered', self.imu_cb, 50)
        self.sub_range = self.create_subscription(
            Range, '/range/height', self.range_cb, best_effort)
        self.sub_flow = self.create_subscription(
            Odometry, '/optical_flow/odom', self.flow_cb, 10)

        self.pub = self.create_publisher(Odometry, '/optical_flow/corrected', 10)

        self.get_logger().info('Flow de-rotation node started')

    def imu_cb(self, msg: Imu):
        self.omega_x = msg.angular_velocity.x
        self.omega_y = msg.angular_velocity.y

    def range_cb(self, msg: Range):
        if msg.range > msg.min_range and msg.range < msg.max_range:
            self.height = msg.range

    def flow_cb(self, msg: Odometry):
        vx_raw = msg.twist.twist.linear.x
        vy_raw = msg.twist.twist.linear.y

        # De-rotation correction
        vx_true = vx_raw - (self.omega_y * self.height)
        vy_true = vy_raw + (self.omega_x * self.height)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.twist.twist.linear.x = vx_true
        out.twist.twist.linear.y = vy_true
        out.twist.covariance = msg.twist.covariance  # preserve SQUAL-based covariance

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FlowDerotationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
