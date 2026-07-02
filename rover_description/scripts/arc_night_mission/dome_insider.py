#!/usr/bin/env python3
"""
Dome insider helper node to autonomously enter a dome structure.
Uses the depth camera to center the rover between the dome entrance walls.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

class DomeInsiderNode(Node):
    def __init__(self):
        super().__init__('dome_insider_node')
        
        self.bridge = CvBridge()
        
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(String, 'rover/mission_state', self.global_state_callback, qos_profile)
        self.sub_depth = self.create_subscription(Image, '/rgbd_camera/depth_image', self.depth_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/dome_insider_debug', 10) 
        
        # Service Client
        self.fsm_client = self.create_client(Trigger, 'mission/complete_dome_entry')
        
        self.global_mission_state = "BOOTING"
        self.is_finished_reported = False
        
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.start_odom_x = None
        self.start_odom_y = None
        
        # Distance to push inside the dome
        self.target_push_distance = 4.0 
        
        self.get_logger().info("DEPTH-BASED DOME INSIDER NODE STARTED. Waiting for DOME_ENTRY state...")

    def global_state_callback(self, msg):
        self.global_mission_state = msg.data

    def odom_callback(self, msg):
        if self.global_mission_state != 'DOME_ENTRY':
            return

        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        
        if self.start_odom_x is None:
            self.start_odom_x = self.current_odom_x
            self.start_odom_y = self.current_odom_y

    def depth_callback(self, msg):
        if self.global_mission_state != 'DOME_ENTRY' or self.is_finished_reported:
            return

        if self.start_odom_x is None:
            return # Wait for odometry to lock in

        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
            return

        height, width = cv_depth.shape
        left_region = cv_depth[:, :width//3]
        center_region = cv_depth[:, width//3 : 2*width//3]
        right_region = cv_depth[:, 2*width//3:]

        def get_safe_depth(region):
            mask = np.isfinite(region) & (region > 0.4)
            valid = region[mask]
            if len(valid) == 0:
                return 10.0  # Safe default if no obstacles in range
            return np.percentile(valid, 20)

        left_depth = get_safe_depth(left_region)
        center_depth = get_safe_depth(center_region)
        right_depth = get_safe_depth(right_region)

        # Check Odometry to see if we reached target depth
        dist = math.sqrt((self.current_odom_x - self.start_odom_x)**2 + (self.current_odom_y - self.start_odom_y)**2)
        
        twist = Twist()
        
        if dist >= self.target_push_distance:
            # We are done!
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("SUCCESSFULLY ENTERED THE DOME!")
            self.notify_mission_control_complete()
        else:
            # Emergency Stop if too close to a wall head-on
            if center_depth < 0.6:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                # Drive forward
                twist.linear.x = 0.6
                
                # Proportional control for centering between the left and right walls of the gate
                diff = left_depth - right_depth
                twist.angular.z = max(min(0.3 * diff, 0.5), -0.5)
                
            self.cmd_pub.publish(twist)

        # Publish Debug image
        self.publish_debug_windows(cv_depth, left_depth, center_depth, right_depth)

    def publish_debug_windows(self, cv_depth, left_depth, center_depth, right_depth):
        try:
            # Normalize depth for visualization (cap at 10 meters for colormap visibility)
            depth_clipped = np.clip(cv_depth, 0, 10.0)
            depth_norm = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            # Draw lines for the center 1/3 region
            h, w = depth_color.shape[:2]
            cv2.line(depth_color, (w//3, 0), (w//3, h), (255, 255, 255), 2)
            cv2.line(depth_color, (2*w//3, 0), (2*w//3, h), (255, 255, 255), 2)

            # Status Overlays
            cv2.putText(depth_color, "STATE: DEPTH SERVOING (ENTRY)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(depth_color, f"L:{left_depth:.1f} C:{center_depth:.1f} R:{right_depth:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show distance left
            if self.start_odom_x is not None:
                dist = math.sqrt((self.current_odom_x - self.start_odom_x)**2 + (self.current_odom_y - self.start_odom_y)**2)
                cv2.putText(depth_color, f"DIST: {dist:.1f}/{self.target_push_distance:.1f}m", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            try:
                debug_msg = self.bridge.cv2_to_imgmsg(depth_color, encoding="bgr8")
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                pass
            
            cv2.imshow("DEPTH_ENTRY_DEBUG", depth_color)
            cv2.waitKey(1)
        except cv2.error as e:
            pass

    def notify_mission_control_complete(self):
        if not self.is_finished_reported:
            self.is_finished_reported = True
            if not self.fsm_client.service_is_ready():
                self.get_logger().warn("FSM Service not ready.")
                return
            req = Trigger.Request()
            future = self.fsm_client.call_async(req)
            future.add_done_callback(self.fsm_response_callback)

    def fsm_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info("Central FSM acknowledged dome entry complete.")
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DomeInsiderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
