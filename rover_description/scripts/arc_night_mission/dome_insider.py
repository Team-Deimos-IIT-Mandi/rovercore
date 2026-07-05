#!/usr/bin/env python3
"""
Dome insider helper node to autonomously enter a dome structure.
Uses a hybrid approach:
1. When 2 ArUco markers are seen, it uses visual servoing on their midpoint to aim at the gate center.
2. If < 2 markers are seen (e.g., getting close to the gate and losing sight of one), 
   it INSTANTLY falls back to Depth Servoing, using the physical walls to perfectly squeeze through without colliding!
3. Depth camera threshold stops the rover precisely when near the back wall.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

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
        self.sub_rgb = self.create_subscription(Image, '/cam/front/image_raw', self.image_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/rgbd_camera/depth_image', self.depth_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/dome_insider_debug', 10) 
        
        # Service Client
        self.fsm_client = self.create_client(Trigger, 'mission/complete_dome_entry')
        
        # Internal State
        self.global_mission_state = 'BOOTING'
        self.is_finished_reported = False
        
        self.aruco_error = None
        self.aruco_last_seen_time = 0.0
        
        # ArUco Tracker
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Goal Stop distance inside dome
        self.target_stop_depth = 1.5  # Stop 1.5m from the back wall
        
        self.get_logger().info("SMART AVOIDANCE DOME INSIDER NODE STARTED. Waiting for DOME_ENTRY state.")

    def global_state_callback(self, msg):
        self.global_mission_state = msg.data

    def image_callback(self, msg):
        if self.global_mission_state != 'DOME_ENTRY' or self.is_finished_reported:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        h, w = cv_image.shape[:2]
        center_x = w / 2.0

        if ids is not None and len(ids) >= 2:
            # Find the center X of all detected markers
            all_x = []
            for corner in corners:
                c_x = np.mean(corner[0][:, 0])
                all_x.append(c_x)
                
            midpoint_x = np.mean(all_x)
            
            # Calculate pixel error for centering
            self.aruco_error = center_x - midpoint_x
            self.aruco_last_seen_time = time.time()
            
            # Draw on image
            cv2.circle(cv_image, (int(midpoint_x), int(h//2)), 10, (0, 255, 0), -1)
            cv2.line(cv_image, (int(center_x), 0), (int(center_x), h), (255, 0, 0), 2)
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            cv2.putText(cv_image, f"ARUCO SERVOING (2 MARKERS)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            if ids is not None and len(ids) == 1:
                cv2.putText(cv_image, "1 MARKER (FALLBACK TO DEPTH)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            else:
                cv2.putText(cv_image, "NO MARKERS (FALLBACK TO DEPTH)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("RGB_ENTRY_DEBUG", cv_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        if self.global_mission_state != 'DOME_ENTRY' or self.is_finished_reported:
            return

        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            return

        height, width = cv_depth.shape
        left_region = cv_depth[:, :width//3]
        center_region = cv_depth[:, width//3 : 2*width//3]
        right_region = cv_depth[:, 2*width//3:]

        def get_safe_depth(region):
            mask = np.isfinite(region) & (region > 0.4)
            valid = region[mask]
            if len(valid) == 0:
                return 10.0
            return np.percentile(valid, 20)

        left_depth = get_safe_depth(left_region)
        center_depth = get_safe_depth(center_region)
        right_depth = get_safe_depth(right_region)

        twist = Twist()
        
        # Stop condition: Center depth is less than the threshold (we reached the back wall)
        if center_depth < self.target_stop_depth and left_depth < 2.5 and right_depth < 2.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("SUCCESSFULLY ENTERED THE DOME AND REACHED THE BACK WALL!")
            self.notify_mission_control_complete()
        else:
            # Emergency Stop if too close to a wall head-on
            if center_depth < 0.6:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                mode_text = "EMERGENCY STOP"
            else:
                # Drive forward
                twist.linear.x = 0.5
                
                # Check if we should use ArUco or Depth for centering
                time_since_aruco = time.time() - self.aruco_last_seen_time
                
                # ONLY use ArUco if we have seen BOTH markers in the last 0.5 seconds
                if self.aruco_error is not None and time_since_aruco < 0.5:
                    # ArUco Servoing Mode (High Priority)
                    twist.angular.z = max(min(0.002 * self.aruco_error, 0.5), -0.5)
                    mode_text = "ARUCO SERVO"
                else:
                    # Depth Servoing Mode (Collision Avoidance / Squeeze through)
                    # This automatically repels the rover from the closer wall!
                    diff = left_depth - right_depth
                    twist.angular.z = max(min(0.3 * diff, 0.5), -0.5)
                    mode_text = "DEPTH SERVO (AVOIDING WALLS)"
                
            self.cmd_pub.publish(twist)

        # Publish Debug image
        self.publish_debug_windows(cv_depth, left_depth, center_depth, right_depth, mode_text)

    def publish_debug_windows(self, cv_depth, left_depth, center_depth, right_depth, mode_text):
        try:
            depth_clipped = np.clip(cv_depth, 0, 10.0)
            depth_norm = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            h, w = depth_color.shape[:2]
            cv2.line(depth_color, (w//3, 0), (w//3, h), (255, 255, 255), 2)
            cv2.line(depth_color, (2*w//3, 0), (2*w//3, h), (255, 255, 255), 2)

            cv2.putText(depth_color, f"MODE: {mode_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(depth_color, f"L:{left_depth:.1f} C:{center_depth:.1f} R:{right_depth:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(depth_color, encoding="bgr8")
                self.debug_pub.publish(debug_msg)
            except Exception:
                pass
            
            cv2.imshow("DEPTH_ENTRY_DEBUG", depth_color)
            cv2.waitKey(1)
        except cv2.error:
            pass

    def notify_mission_control_complete(self):
        self.is_finished_reported = True

        if not self.fsm_client.service_is_ready():
            self.get_logger().info("Waiting for Mission Manager dome-entry service...")
            self.is_finished_reported = False
            return

        req = Trigger.Request()
        future = self.fsm_client.call_async(req)
        future.add_done_callback(self.fsm_response_callback)

    def fsm_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info("SUCCESS: Central FSM acknowledged dome entry complete.")
            else:
                self.get_logger().error(f"FSM rejected dome entry completion: {res.message}")
                self.is_finished_reported = False
        except Exception as e:
            self.get_logger().error(f"Service communication failed: {e}")
            self.is_finished_reported = False

def main(args=None):
    rclpy.init(args=args)
    node = DomeInsiderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
