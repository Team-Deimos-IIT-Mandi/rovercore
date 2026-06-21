#!/usr/bin/env python3
"""Injects position covariance into NavSatFix when the bridge leaves it zero.

In simulation, Gazebo's NavSat sensor → ros_gz_bridge produces NavSatFix with
position_covariance = [0]*9 and covariance_type = COVARIANCE_TYPE_UNKNOWN.
The navsat_transform_node needs valid covariance to function correctly.

This node passes through the fix, injecting a configurable default covariance
when the incoming message has COVARIANCE_TYPE_UNKNOWN.

Subscribes:
    /gps/fix_raw (sensor_msgs/NavSatFix)

Publishes:
    /gps/fix (sensor_msgs/NavSatFix) — with covariance populated
"""

import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsFixCovarianceNode(Node):
    def __init__(self):
        super().__init__('gps_fix_covariance_node')

        # Default position variance in m² — typical consumer-grade GPS
        self.declare_parameter('default_xy_variance', 4.0)    # ~2m std dev
        self.declare_parameter('default_z_variance', 16.0)    # ~4m std dev (vertical worse)

        self.xy_var = self.get_parameter('default_xy_variance').value
        self.z_var = self.get_parameter('default_z_variance').value

        self.sub = self.create_subscription(
            NavSatFix, '/gps/fix_raw', self.fix_cb, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.get_logger().info(
            f'GPS covariance injector started (xy_var={self.xy_var}, z_var={self.z_var})')

    def fix_cb(self, msg: NavSatFix):
        out = copy.deepcopy(msg)

        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            out.position_covariance = [
                self.xy_var, 0.0,        0.0,
                0.0,         self.xy_var, 0.0,
                0.0,         0.0,        self.z_var,
            ]
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GpsFixCovarianceNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
