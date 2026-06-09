#!/usr/bin/env python3
"""Subscribe to the gripper RGB camera and show the HSV and mask in OpenCV windows.

Usage:
  python3 display_hsv.py [--topic TOPIC]

Default topic: /arm/gripper/rgbd_camera/image
"""
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

DEFAULT_TOPIC = '/arm/gripper/rgbd_camera/image'


class HSVViewer(Node):
    def __init__(self, topic):
        super().__init__('hsv_viewer')
        self.br = CvBridge()
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.get_logger().info(f'HsvViewer subscribed to {topic}')

    def cb(self, msg: Image):
        try:
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Use same mask thresholds as manip.py (low brightness mask)
        lower = np.array([0, 0, 0])
        upper = np.array([180, 255, 75])
        mask = cv2.inRange(hsv, lower, upper)

        # Compose display
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack((img, cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), mask_bgr))

        cv2.imshow('RGB | HSV | Mask', combined)
        cv2.waitKey(1)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default=DEFAULT_TOPIC)
    args = parser.parse_args(argv)

    rclpy.init()
    node = HSVViewer(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
