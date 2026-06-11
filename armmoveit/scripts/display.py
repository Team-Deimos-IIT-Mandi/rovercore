import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class TripleDisplayNode(Node):
    def __init__(self):
        super().__init__('triple_display_node')
        
        self.bridge = CvBridge()
        
        # Placeholders to store the latest incoming frames
        self.raw_frame = None
        self.mask_frame = None
        self.detect_frame = None

        # Subscribe to all three streams
        self.sub_raw = self.create_subscription(
            Image, '/arm/gripper/rgbd_camera/image', self.raw_cb, 10)
            
        self.sub_mask = self.create_subscription(
            Image, '/arm/gripper/valve_mask', self.mask_cb, 10)
            
        self.sub_detect = self.create_subscription(
            Image, '/arm/gripper/valve_detection', self.detect_cb, 10)

        # Create a timer to refresh the OpenCV window at ~30 FPS
        self.timer = self.create_timer(0.033, self.update_display)
        self.get_logger().info("Display Node started. Waiting for all three video streams...")

    # --- Callbacks to grab the latest frames ---
    def raw_cb(self, msg):
        try:
            self.raw_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Raw image error: {e}")

    def mask_cb(self, msg):
        try:
            # The mask is published as Mono8 (1 channel). 
            # We must convert it back to 3 channels (BGR) so NumPy can stack it with the others.
            mono_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self.mask_frame = cv2.cvtColor(mono_img, cv2.COLOR_GRAY2BGR)
        except CvBridgeError as e:
            self.get_logger().error(f"Mask image error: {e}")

    def detect_cb(self, msg):
        try:
            self.detect_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Detection image error: {e}")

    # --- GUI Update Loop ---
    def update_display(self):
        # Only render if we have received at least one frame from all three topics
        if self.raw_frame is not None and self.mask_frame is not None and self.detect_frame is not None:
            
            # Resize them so they fit nicely on a standard monitor (480x360 each)
            dim = (480, 360)
            raw_rs = cv2.resize(self.raw_frame, dim)
            mask_rs = cv2.resize(self.mask_frame, dim)
            detect_rs = cv2.resize(self.detect_frame, dim)

            # Add labels to the images before stacking
            cv2.putText(raw_rs, "1. Raw Feed", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(mask_rs, "2. Red Filter Mask", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(detect_rs, "3. Bounding Box", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Stack the three images horizontally
            dashboard = np.hstack((raw_rs, mask_rs, detect_rs))
            
            cv2.imshow("Rover Arm Vision Dashboard", dashboard)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TripleDisplayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()