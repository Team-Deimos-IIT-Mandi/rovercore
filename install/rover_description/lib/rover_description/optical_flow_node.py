#!/usr/bin/env python3
"""Optical flow node simulating PMW3901 behavior.

Subscribes to downward camera + rangefinder, computes ground velocity
via Farneback dense optical flow, publishes Odometry with SQUAL-based covariance.

Topics subscribed:
    /flow_cam/image (sensor_msgs/Image)
    /range/height (sensor_msgs/LaserScan - single beam)

Topics published:
    /optical_flow/odom (nav_msgs/Odometry) - twist only, covariance from SQUAL
    /optical_flow/squal (std_msgs/UInt8) - surface quality indicator
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from cv_bridge import CvBridge


class OpticalFlowNode(Node):
    def __init__(self):
        super().__init__('optical_flow_node')

        # TUNE: These parameters control flow->velocity conversion
        self.declare_parameter('camera_fov_rad', 1.0472)       # 60 deg horizontal FOV
        self.declare_parameter('image_width', 120)              # pixels
        self.declare_parameter('base_covariance', 0.01)         # TUNE: base twist covariance when SQUAL is max
        self.declare_parameter('max_squal', 200)                # TUNE: maximum expected SQUAL value
        self.declare_parameter('min_squal_threshold', 20)       # TUNE: below this, covariance goes to 1e6

        self.fov = self.get_parameter('camera_fov_rad').value
        self.img_w = self.get_parameter('image_width').value
        self.base_cov = self.get_parameter('base_covariance').value
        self.max_squal = self.get_parameter('max_squal').value
        self.min_squal = self.get_parameter('min_squal_threshold').value

        self.bridge = CvBridge()
        self.prev_gray = None
        self.current_height = 0.3  # default height in meters
        self.prev_stamp = None

        best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub_image = self.create_subscription(
            Image, '/flow_cam/image', self.image_cb, best_effort)
        self.sub_range = self.create_subscription(
            LaserScan, '/range/height', self.range_cb, best_effort)

        self.pub_odom = self.create_publisher(Odometry, '/optical_flow/odom', 10)
        self.pub_squal = self.create_publisher(UInt8, '/optical_flow/squal', 10)

        self.get_logger().info('Optical flow node started')

    def range_cb(self, msg: LaserScan):
        if len(msg.ranges) > 0 and msg.ranges[0] > msg.range_min:
            self.current_height = msg.ranges[0]

    def image_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        if self.prev_gray is None or self.prev_stamp is None:
            self.prev_gray = gray
            self.prev_stamp = stamp
            return

        dt = (stamp - self.prev_stamp).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0:
            self.prev_gray = gray
            self.prev_stamp = stamp
            return

        # Farneback dense optical flow
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray, None,
            pyr_scale=0.5, levels=3, winsize=15,
            iterations=3, poly_n=5, poly_sigma=1.2, flags=0)

        # Average flow across image (pixels/frame)
        mean_flow_x = np.mean(flow[:, :, 0])
        mean_flow_y = np.mean(flow[:, :, 1])

        # Compute SQUAL from image contrast (feature count proxy)
        # Higher std dev in image = more texture = higher SQUAL
        squal_raw = min(int(np.std(gray) * 3.0), 255)  # TUNE: scaling factor
        squal = max(squal_raw, 1)

        # Convert pixels/frame -> m/s using pinhole model
        # pixels_per_meter = (image_width / (2 * height * tan(fov/2)))
        focal_length_px = self.img_w / (2.0 * np.tan(self.fov / 2.0))
        if self.current_height > 0.02:
            vx = (mean_flow_x / focal_length_px) * self.current_height / dt
            vy = (mean_flow_y / focal_length_px) * self.current_height / dt
        else:
            vx = 0.0
            vy = 0.0

        # SQUAL-based covariance: covariance = base_cov * (max_squal / squal)^2
        # TUNE: This mapping directly controls how much the EKF trusts optical flow
        if squal < self.min_squal:
            cov = 1e6  # effectively disable
        else:
            cov = self.base_cov * (self.max_squal / squal) ** 2

        # Publish Odometry (twist only)
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        # Set twist covariance (6x6 row-major, indices 0,7 are vx,vy)
        odom.twist.covariance[0] = cov   # vx
        odom.twist.covariance[7] = cov   # vy
        odom.twist.covariance[14] = 1e6  # vz (unused)
        odom.twist.covariance[21] = 1e6  # wx (unused)
        odom.twist.covariance[28] = 1e6  # wy (unused)
        odom.twist.covariance[35] = 1e6  # wz (unused)
        self.pub_odom.publish(odom)

        # Publish SQUAL
        squal_msg = UInt8()
        squal_msg.data = squal
        self.pub_squal.publish(squal_msg)

        self.prev_gray = gray
        self.prev_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
