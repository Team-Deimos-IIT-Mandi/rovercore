#!/usr/bin/env python3
"""
Standalone HSV mask tuning tool.
Subscribes to a camera topic, shows raw + masked frames with interactive HSV trackbars.
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class HSVTestNode(Node):
    def __init__(self):
        super().__init__('hsv_test_node')

        self.declare_parameter('camera_topic', '/cam/front/image_raw')

        topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(Image, topic, self.image_callback, 10)
        self.bridge = CvBridge()

        self.win_ctrl = 'HSV Controls'
        self.win_mask = 'Mask'
        self.win_frame = 'Frame'

        self.lock = threading.Lock()
        self.frame = None

        cv2.namedWindow(self.win_ctrl, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win_ctrl, 520, 320)
        cv2.namedWindow(self.win_mask, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.win_frame, cv2.WINDOW_NORMAL)

        cv2.createTrackbar('L-H', self.win_ctrl, 0, 179, self.noop)
        cv2.createTrackbar('L-S', self.win_ctrl, 0, 255, self.noop)
        cv2.createTrackbar('L-V', self.win_ctrl, 0, 255, self.noop)
        cv2.createTrackbar('U-H', self.win_ctrl, 179, 179, self.noop)
        cv2.createTrackbar('U-S', self.win_ctrl, 255, 255, self.noop)
        cv2.createTrackbar('U-V', self.win_ctrl, 255, 255, self.noop)

        self.create_timer(0.033, self.render)  # ~30 FPS refresh

        self.get_logger().info(f'HSV test node listening on {topic}')

    @staticmethod
    def noop(_):
        pass

    def image_callback(self, msg):
        with self.lock:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def render(self):
        with self.lock:
            if self.frame is None:
                return
            frame = self.frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lh = cv2.getTrackbarPos('L-H', self.win_ctrl)
        ls = cv2.getTrackbarPos('L-S', self.win_ctrl)
        lv = cv2.getTrackbarPos('L-V', self.win_ctrl)
        uh = cv2.getTrackbarPos('U-H', self.win_ctrl)
        us = cv2.getTrackbarPos('U-S', self.win_ctrl)
        uv = cv2.getTrackbarPos('U-V', self.win_ctrl)

        lower = np.array([min(lh, uh), min(ls, us), min(lv, uv)])
        upper = np.array([max(lh, uh), max(ls, us), max(lv, uv)])

        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Overlay HSV values on frame
        cv2.putText(frame, f'Lower {lower.tolist()}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f'Upper {upper.tolist()}', (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f'Mask pixels: {cv2.countNonZero(mask)}', (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow(self.win_frame, frame)
        cv2.imshow(self.win_mask, result)
        cv2.waitKey(1)

    def cleanup(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HSVTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
