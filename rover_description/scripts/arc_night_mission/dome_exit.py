"""
Dome exit helper node to autonomously exit from a dome structure.
Processes depth feeds to find the max distance (the exit), aligns to it, 
and uses side-camera tripwires and odometry to confirm the exit.
"""

import logging
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

logger = logging.getLogger(__name__)

def get_yaw_from_quaternion(q):
    """ Helper to extract yaw from a quaternion """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class AirlockExitNode(Node):
    def __init__(self):
        super().__init__('airlock_exit_node')
        
        self.bridge = CvBridge()
        
        # --- QoS Profile for state tracking ---
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Subscribers (Replaced front RGB with Depth)
        self.state_sub = self.create_subscription(String, 'rover/mission_state', self.global_state_callback, qos_profile)
        self.sub_depth = self.create_subscription(Image, '/rgbd_camera/depth_image', self.depth_callback, 10)
        self.sub_left = self.create_subscription(Image, '/cam/left/image_raw', self.left_callback, 10)
        self.sub_right = self.create_subscription(Image, '/cam/right/image_raw', self.right_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/airlock_debug', 10) 
        
        # Service Client
        self.fsm_client = self.create_client(Trigger, 'mission/complete_dome_exit')
        
        # Global & Internal State
        self.global_mission_state = "BOOTING"
        
        # States: 0 = 360 Scan, 1 = Aligning to Best Yaw, 2 = Driving Forward, 3 = Blind Push out, 4 = Done
        self.internal_state = 0 
        self.is_finished_reported = False
        
        # Depth Scan Variables
        self.search_rotation_speed = 0.3
        self.target_forward_speed = 1.0  
        self.current_yaw = 0.0
        self.last_yaw = None
        self.yaw_swept = 0.0
        self.best_yaw = 0.0
        self.best_depth_score = -1.0
        
        # Emergency Kit Search Variables
        self.largest_kit_area = 0
        self.best_kit_yaw = None
        self.sub_rgb = self.create_subscription(Image, '/rgbd_camera/image', self.rgb_callback, 10)
        self.latest_cv_depth = None
        
        # Odometry tracking for exit verification
        self.start_odom_x = None
        self.start_odom_y = None
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        
        # Side tripwire logic
        self.tripwire_threshold = 15000   
        self.blind_push_distance = 0.5
        self.tripwire_counter = 0

        self.debug_windows_ready = False

        self.get_logger().info("DEPTH-BASED DOME EXIT NODE STARTED. Listening to global FSM...")

    def global_state_callback(self, msg):
        self.global_mission_state = msg.data

    def odom_callback(self, msg):
        if self.global_mission_state != 'DOME_EXIT':
            return

        # Track Position
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        
        # Track Yaw for the 360 Sweep
        self.current_yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)
        
        if self.internal_state in [0, 4]:
            if self.last_yaw is None:
                self.last_yaw = self.current_yaw
            
            # Calculate swept angle continuously 
            dyaw = self.current_yaw - self.last_yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw)) # Normalize to [-pi, pi]
            self.yaw_swept += abs(dyaw)
            self.last_yaw = self.current_yaw

        # Blind push out of the door
        if self.internal_state == 3 and self.start_odom_x is not None:
            dist = math.sqrt((self.current_odom_x - self.start_odom_x)**2 + (self.current_odom_y - self.start_odom_y)**2)
            if dist >= self.blind_push_distance:
                self.get_logger().info(f"==> TAIL CLEARED. Transitioning to Kit Search (State 4).")
                self.internal_state = 4 
                self.yaw_swept = 0.0
                self.last_yaw = None
                self.largest_kit_area = 0
                self.best_kit_yaw = None

        # Push outward if kit not found
        if self.internal_state == 7 and self.start_odom_x is not None:
            dist = math.sqrt((self.current_odom_x - self.start_odom_x)**2 + (self.current_odom_y - self.start_odom_y)**2)
            if dist >= 1.0:
                self.internal_state = 4
                self.yaw_swept = 0.0
                self.last_yaw = None
                self.largest_kit_area = 0
                self.best_kit_yaw = None

    def check_tripwire(self, msg, camera_name):
        # Only check tripwire while actively driving toward the exit
        if self.global_mission_state != 'DOME_EXIT' or self.internal_state != 2:
            return 
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.tripwire_threshold:
                self.tripwire_counter += 1
                if self.tripwire_counter > 5:
                    self.get_logger().info(f"==> EXIT TRIPWIRE TRIGGERED BY {camera_name}!")
                    self.start_odom_x = self.current_odom_x
                    self.start_odom_y = self.current_odom_y
                    self.internal_state = 3  # Switch to blind push
            else:
                self.tripwire_counter = 0

    def left_callback(self, msg):
        self.check_tripwire(msg, "LEFT")

    def right_callback(self, msg):
        self.check_tripwire(msg, "RIGHT")

    def rgb_callback(self, msg):
        if self.global_mission_state != 'DOME_EXIT' or self.internal_state != 4:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # White color bounds
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            best_area = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = float(w) / h if h > 0 else 0
                    
                    if 1.5 <= aspect_ratio <= 2.5:
                        if self.latest_cv_depth is not None:
                            h_depth, w_depth = self.latest_cv_depth.shape
                            x1, y1 = max(0, x), max(0, y)
                            x2, y2 = min(w_depth, x+w), min(h_depth, y+h)
                            
                            roi_depth = self.latest_cv_depth[y1:y2, x1:x2]
                            valid_depths = roi_depth[np.isfinite(roi_depth) & (roi_depth > 0.1)]
                            
                            if len(valid_depths) > 0:
                                median_depth = np.median(valid_depths)
                                if 0.5 <= median_depth <= 3.0:
                                    if area > best_area:
                                        best_area = area
                                        
            if best_area > self.largest_kit_area:
                self.largest_kit_area = best_area
                self.best_kit_yaw = self.current_yaw

    def depth_callback(self, msg):
        if self.global_mission_state != 'DOME_EXIT':
            return

        try:
            # 32FC1 (floats in meters) or 16UC1 (millimeters). Assuming meters (32FC1).
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_cv_depth = cv_depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
            return

        height, width = cv_depth.shape
        left_region = cv_depth[:, :width//3]
        center_region = cv_depth[:, width//3 : 2*width//3]
        right_region = cv_depth[:, 2*width//3:]

        def get_safe_depth(region):
            mask = np.isfinite(region) & (region > 0.6)
            valid = region[mask]
            if len(valid) == 0:
                return 10.0  # Safe default if no obstacles in range
            return np.percentile(valid, 20)

        left_depth = get_safe_depth(left_region)
        center_depth = get_safe_depth(center_region)
        right_depth = get_safe_depth(right_region)

        # For the 360 scan, use the 90th percentile of the center region
        valid_center_mask = np.isfinite(center_region) & (center_region > 0.6)
        valid_center_depths = center_region[valid_center_mask]
        if len(valid_center_depths) > 0:
            score = np.percentile(valid_center_depths, 90)
        else:
            score = 0.0

        twist = Twist()

        # --- Internal State 0: 360 Degree Scan ---
        if self.internal_state == 0:
            twist.angular.z = self.search_rotation_speed
            self.cmd_pub.publish(twist)

            if score > self.best_depth_score:
                # Ignore boundary yaws near ±180 deg to avoid tricky wraparound edge cases
                if abs(abs(self.current_yaw) - math.pi) > 0.15:
                    self.best_depth_score = score
                    self.best_yaw = self.current_yaw

            # Check if we have completed a full rotation
            if self.yaw_swept >= 2 * math.pi:
                self.get_logger().info(f"SCAN COMPLETE. Best depth: {self.best_depth_score:.2f}m at yaw {self.best_yaw:.2f} rad")
                self.internal_state = 1

        # --- Internal State 1: Align to Best Yaw ---
        elif self.internal_state == 1:
            yaw_error = self.best_yaw - self.current_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error)) # Normalize

            if abs(yaw_error) < 0.1: # Within ~5 degrees
                self.get_logger().info("ALIGNED TO EXIT. Driving forward.")
                self.internal_state = 2
                self.cmd_pub.publish(Twist()) # Stop turning
            else:
                # Simple P-controller for rotation
                twist.angular.z = max(min(1.0 * yaw_error, 0.5), -0.5) 
                self.cmd_pub.publish(twist)

        # --- Internal State 2: Drive Toward Exit (Visual Servoing) ---
        elif self.internal_state == 2:
            # 1. Emergency Stop (Highest Priority)
            EMERGENCY_DISTANCE = 0.7
            if (center_depth < EMERGENCY_DISTANCE or
                left_depth < EMERGENCY_DISTANCE or
                right_depth < EMERGENCY_DISTANCE):
                self.get_logger().error("EMERGENCY STOP: WALL TOO CLOSE!", throttle_duration_sec=1.0)
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return
            
            # 2. Side Wall Protection
            MIN_SIDE_CLEARANCE = 1.0
            if left_depth < MIN_SIDE_CLEARANCE:
                twist.linear.x = 0.1
                twist.angular.z = -0.6
                self.cmd_pub.publish(twist)
                return
            if right_depth < MIN_SIDE_CLEARANCE:
                twist.linear.x = 0.1
                twist.angular.z = 0.6
                self.cmd_pub.publish(twist)
                return

            # 3. Adaptive Forward Speed
            if center_depth > 6.0:
                twist.linear.x = 1.0
            elif center_depth > 3.0:
                twist.linear.x = 0.5
            else:
                twist.linear.x = 0.2
            
            # 4. Proportional control for centering
            diff = left_depth - right_depth
            twist.angular.z = max(min(0.2 * diff, 0.5), -0.5)
            
            self.cmd_pub.publish(twist)

        # --- Internal State 3: Blind Push (Triggered by Side Cams) ---
        elif self.internal_state == 3:
            twist.linear.x = self.target_forward_speed
            self.cmd_pub.publish(twist)

        # --- Internal State 4: Kit Search ---
        elif self.internal_state == 4:
            twist.angular.z = 0.2
            self.cmd_pub.publish(twist)

            if self.yaw_swept >= 2 * math.pi:
                if self.best_kit_yaw is not None:
                    self.get_logger().info(f"KIT FOUND! Aligning to yaw {self.best_kit_yaw:.2f}")
                    self.internal_state = 5
                else:
                    self.get_logger().info("KIT NOT FOUND! Moving away from dome and trying again...")
                    self.start_odom_x = self.current_odom_x
                    self.start_odom_y = self.current_odom_y
                    self.internal_state = 7

        # --- Internal State 5: Approach Kit ---
        elif self.internal_state == 5:
            yaw_error = self.best_kit_yaw - self.current_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            
            if abs(yaw_error) > 0.1:
                twist.angular.z = max(min(1.0 * yaw_error, 0.5), -0.5)
            else:
                twist.linear.x = 0.5
                if center_depth > 0 and center_depth < 1.5:
                    self.get_logger().info("ARRIVED AT EMERGENCY KIT!")
                    self.internal_state = 6
                    
            self.cmd_pub.publish(twist)

        # --- Internal State 6: Finished ---
        elif self.internal_state == 6:
            self.cmd_pub.publish(Twist())
            if not self.is_finished_reported:
                self.notify_mission_control_complete()

        # --- Internal State 7: Push outward ---
        elif self.internal_state == 7:
            twist.linear.x = 0.5
            self.cmd_pub.publish(twist)

        # Generate a colormap visualization for debugging
        self.publish_debug_windows(cv_depth, self.internal_state, left_depth, center_depth, right_depth, score)

    def publish_debug_windows(self, cv_depth, state, left_depth, center_depth, right_depth, score):
        try:
            if not self.debug_windows_ready:
                cv2.namedWindow("DEPTH_DEBUG", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("DEPTH_DEBUG", 640, 480)
                self.debug_windows_ready = True

            # Normalize depth for visualization (cap at 10 meters for colormap visibility)
            depth_clipped = np.clip(cv_depth, 0, 10.0)
            depth_norm = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            # Draw lines for the center 1/3 region
            h, w = depth_color.shape[:2]
            cv2.line(depth_color, (w//3, 0), (w//3, h), (255, 255, 255), 2)
            cv2.line(depth_color, (2*w//3, 0), (2*w//3, h), (255, 255, 255), 2)

            # Status Overlays
            state_text = ["0: SCAN EXIT", "1: ALIGN", "2: SERVOING", "3: PUSH", "4: KIT SCAN", "5: APPROACH", "6: DONE", "7: OUTWARD"]
            cv2.putText(depth_color, f"STATE: {state_text[state] if state < len(state_text) else str(state)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(depth_color, f"L:{left_depth:.1f} C:{center_depth:.1f} R:{right_depth:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(depth_color, encoding="bgr8")
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish debug image: {e}")
            
            cv2.imshow("DEPTH_DEBUG", depth_color)
            cv2.waitKey(1)
        except cv2.error as e:
            pass

    def notify_mission_control_complete(self):
        self.is_finished_reported = True
        if not self.fsm_client.service_is_ready():
            return
        req = Trigger.Request()
        future = self.fsm_client.call_async(req)
        future.add_done_callback(self.fsm_response_callback)

    def fsm_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info("SUCCESS: Central FSM acknowledged dome exit complete.")
            else:
                self.is_finished_reported = False 
        except Exception as e:
            self.is_finished_reported = False

def main(args=None):
    rclpy.init(args=args)
    node = AirlockExitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
