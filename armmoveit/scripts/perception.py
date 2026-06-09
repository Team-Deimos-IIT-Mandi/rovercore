#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
import tf2_ros
import cv2
from cv_bridge import CvBridge
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('rover_perception_node')
        
        # Target Topics from your system
        self.rgb_sub = self.create_subscription(Image, '/arm/gripper/rgbd_camera/image', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/arm/gripper/rgbd_camera/depth_image', self.depth_callback, 10)
        
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.latest_depth_img = None
        
        # Camera Intrinsic Parameters (Replace with values from your camera_info if available)
        self.fx = 525.0  
        self.fy = 525.0  
        self.cx = 319.5  
        self.cy = 239.5  

    def depth_callback(self, msg):
        # Store latest depth image (handling 16UC1 or 32FC1 formats)
        self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        if self.latest_depth_img is None:
            return
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # --------------------------------------------------------------------
        # TODO: Insert Object Detection Model Here (e.g., YOLOv8 / Contour Detection)
        # For now, we assume an object is detected at the center pixels of the frame
        # --------------------------------------------------------------------
        h, w, _ = cv_image.shape
        u, v = int(w / 2), int(h / 2) 
        
        # Get depth value at the object center (in meters or millimeters depending on driver)
        depth = self.latest_depth_img[v, u]
        if depth == 0: 
            return # Ignore invalid readings
            
        # Convert depth to meters if the camera output is in mm
        z = float(depth) / 1000.0 if depth > 20 else float(depth)
        
        # Deproject 2D pixel coordinates (u, v) to 3D camera space (X, Y, Z)
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        self.broadcast_object_transform(x, y, z, msg.header.stamp)

    def broadcast_object_transform(self, x, y, z, timestamp):
        t = TransformStamped()
        # Must match the optical frame link name attached to your gripper camera in your URDF
        t.header.frame_id = 'gripper_camera_optical_frame' 
        t.header.stamp = timestamp
        t.child_frame_id = 'target_object'
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # No rotation offset relative to camera frame initially
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.terminate()

if __name__ == '__main__':
    main()