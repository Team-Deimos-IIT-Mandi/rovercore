#!/usr/bin/env python3
"""IMU filter node — notch filter on accelerometer.

Subscribes: /imu/data (sensor_msgs/Imu)
Publishes:  /imu/filtered (sensor_msgs/Imu)

Parameters loaded from imu_calibration.yaml.

Note: Magnetometer calibration (hard/soft iron) is NOT applied here because
sensor_msgs/Imu does not carry magnetometer data, and Gazebo's IMU plugin
doesn't publish it. Mag calibration will be added when porting to real hardware
via a separate MagneticField subscriber.
"""

import numpy as np
from scipy.signal import iirnotch, lfilter
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFilterNode(Node):
    def __init__(self):
        super().__init__('imu_filter_node')

        self.declare_parameter('notch_frequency', 45.0)
        self.declare_parameter('notch_bandwidth', 5.0)
        self.declare_parameter('notch_enabled', True)

        freq = self.get_parameter('notch_frequency').value
        bw = self.get_parameter('notch_bandwidth').value
        self.notch_enabled = self.get_parameter('notch_enabled').value

        # Design notch filter for 200Hz sample rate
        fs = 200.0
        quality = freq / bw
        self.notch_b, self.notch_a = iirnotch(freq, quality, fs)

        # Filter state for each accel axis (maintains state between callbacks)
        self.zi_x = np.zeros(max(len(self.notch_a), len(self.notch_b)) - 1)
        self.zi_y = np.zeros_like(self.zi_x)
        self.zi_z = np.zeros_like(self.zi_x)

        self.sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)
        self.pub = self.create_publisher(Imu, '/imu/filtered', 50)

        self.get_logger().info(
            f'IMU filter: notch={freq}Hz bw={bw}Hz enabled={self.notch_enabled}')

    def imu_cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        if self.notch_enabled:
            # Apply IIR notch filter sample-by-sample with state
            [ax], self.zi_x = lfilter(self.notch_b, self.notch_a, [ax], zi=self.zi_x)
            [ay], self.zi_y = lfilter(self.notch_b, self.notch_a, [ay], zi=self.zi_y)
            [az], self.zi_z = lfilter(self.notch_b, self.notch_a, [az], zi=self.zi_z)

        out.linear_acceleration.x = float(ax)
        out.linear_acceleration.y = float(ay)
        out.linear_acceleration.z = float(az)
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
