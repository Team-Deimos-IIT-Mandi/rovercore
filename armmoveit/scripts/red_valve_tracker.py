#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np

class DualCameraTrackerNode(Node):
    def __init__(self):
        super().__init__('dual_camera_tracker')
        self.bridge = CvBridge()

        # 1. Gripper Camera Subscriptions
        self.gripper_sub = self.create_subscription(
            Image, '/arm/gripper/rgbd_camera/image', self.gripper_cb, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, '/arm/gripper/rgbd_camera/depth_image', self.depth_cb, qos_profile_sensor_data)

        # 2. Front Camera Subscription
        self.front_sub = self.create_subscription(
            Image, '/cam/front/image_raw', self.front_cb, qos_profile_sensor_data)

        # 3. Coordinate Publishers
        self.gripper_center_pub = self.create_publisher(Point, '/arm/gripper/valve_center', 10)
        self.front_center_pub = self.create_publisher(Point, '/rover/front/valve_center', 10)

        # 4. Image Publishers for Dashboard/RViz
        self.mask_pub = self.create_publisher(Image, '/arm/gripper/valve_mask', 10)
        self.bbox_pub = self.create_publisher(Image, '/arm/gripper/valve_detection', 10)

        self.latest_depth_image = None
        self.get_logger().info("Dual Camera Tracker Online. Filtering for CIRCULAR red targets only...")

    def depth_cb(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Depth Bridge Error: {e}")

    def process_red_mask(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red_1, upper_red_1 = np.array([0, 120, 70]), np.array([10, 255, 255])
        lower_red_2, upper_red_2 = np.array([170, 120, 70]), np.array([180, 255, 255])
        return cv2.inRange(hsv, lower_red_1, upper_red_1) + cv2.inRange(hsv, lower_red_2, upper_red_2)

    def get_largest_circular_contour(self, contours):
        """ Evaluates all contours and returns the largest one that is circular. """
        valid_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area > 100:
                perimeter = cv2.arcLength(c, True)
                if perimeter > 0:
                    # Circularity Formula
                    circularity = 4 * np.pi * (area / (perimeter * perimeter))
                    
                    # 0.75 Threshold: Filters out elongated ellipses (wheels) and rectangles
                    if circularity > 0.75:
                        valid_contours.append(c)
        
        if valid_contours:
            # Return the largest among the valid circular ones
            return max(valid_contours, key=cv2.contourArea)
        return None

    def gripper_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        red_mask = self.process_red_mask(cv_image)
        detection_image = cv_image.copy()
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Use our new geometry filter!
        target_contour = self.get_largest_circular_contour(contours)

        if target_contour is not None:
            x, y, w, h = cv2.boundingRect(target_contour)
            cx, cy = int(x + w/2), int(y + h/2)

            # Draw tracking boxes on the frame copy
            cv2.rectangle(detection_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(detection_image, (cx, cy), 4, (255, 0, 0), -1)

            # Fetch matching depth map values
            z_depth = 0.0
            if self.latest_depth_image is not None:
                h_d, w_d = self.latest_depth_image.shape
                if 0 <= cx < w_d and 0 <= cy < h_d:
                    raw_depth = self.latest_depth_image[cy, cx]
                    if not np.isnan(raw_depth) and not np.isinf(raw_depth):
                        z_depth = float(raw_depth)

            # Publish 3D Tracking Coordinates
            pt = Point(x=float(cx), y=float(cy), z=z_depth)
            self.gripper_center_pub.publish(pt)

        # Publish processed images
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(red_mask, encoding="mono8")
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

            bbox_msg = self.bridge.cv2_to_imgmsg(detection_image, encoding="bgr8")
            bbox_msg.header = msg.header
            self.bbox_pub.publish(bbox_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to publish image pipeline: {e}")

    def front_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        red_mask = self.process_red_mask(cv_image)
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Apply the exact same circularity check to the front camera
        target_contour = self.get_largest_circular_contour(contours)

        if target_contour is not None:
            x, y, w, h = cv2.boundingRect(target_contour)
            cx, cy = int(x + w/2), int(y + h/2)
            
            # Z=1.0 functions as a boolean flag showing the target is active on the chassis camera
            self.front_center_pub.publish(Point(x=float(cx), y=float(cy), z=1.0))

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraTrackerNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()