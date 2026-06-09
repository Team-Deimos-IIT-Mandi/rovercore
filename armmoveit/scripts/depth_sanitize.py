#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

TOPIC_IN = '/arm/gripper/rgbd_camera/depth_image'
TOPIC_OUT = '/depth_sanitized'
MAX_DEPTH_M = 4.0  # clip to 4 meters; adjust to your sensor

class DepthSanitizer(Node):
    def __init__(self):
        super().__init__('depth_sanitizer')
        self.br = CvBridge()
        self.pub = self.create_publisher(Image, TOPIC_OUT, 10)
        self.sub = self.create_subscription(Image, TOPIC_IN, self.cb, 10)

    def cb(self, msg):
        try:
            depth = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return
        depth = depth.astype(np.float32)
        # Convert values <=0 or non-finite to MAX_DEPTH_M, clip others to [0, MAX_DEPTH_M]
        mask_bad = ~np.isfinite(depth) | (depth <= 0.0)
        depth[mask_bad] = MAX_DEPTH_M
        np.clip(depth, 0.0, MAX_DEPTH_M, out=depth)
        out = self.br.cv2_to_imgmsg(depth, encoding='32FC1')
        out.header = msg.header
        self.pub.publish(out)

def main():
    rclpy.init()
    node = DepthSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()