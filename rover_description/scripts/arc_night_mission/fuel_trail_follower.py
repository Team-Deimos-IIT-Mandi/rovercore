#!/usr/bin/env python3
"""
Made by: Reman Dey
Roll no.: b25331
Task: fuel trail follower node for mars rover integrated into central FSM.
"""

import logging
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np

logger = logging.getLogger(__name__)

class FuelTrailFollower(Node):
    def __init__(self):
        super().__init__('fuel_trail_follower')
        
        # --- QoS Profile for state tracking ---
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            String, 
            'rover/mission_state', 
            self.global_state_callback, 
            qos_profile
        )
        
        # NOTE: Change topic name here based on Gazebo setup (e.g., '/cam/front/image_raw')
        self.subscription = self.create_subscription(
            Image,
            '/cam/front/image_raw', 
            self.image_callback,
            10
        )
        
        # Publisher for rover movement
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vision_done_pub = self.create_publisher(Bool, '/vision_task_complete', 10)
        self.debug_cam_pub = self.create_publisher(Image, '/fuel_trail/debug_frame', 10)
        self.debug_mask_pub = self.create_publisher(Image, '/fuel_trail/debug_mask', 10)
        
        # Service client to signal mission manager
        self.complete_fuel_trail_client = self.create_client(Trigger, 'mission/complete_fuel_trail')
        
        self.bridge = CvBridge()
        self.debug_window_name = "Fuel Trail Debug"
        self.debug_mask_window_name = "Fuel Trail Mask"
        self.tuner_window_name = "HSV Color Selector"
        self.debug_windows_ready = False
        self.active_color_profile = None
        self.color_profiles = {
            'trail': {
                'label': 'Dark trail',
                'lower': np.array([0, 19, 14]),
                'upper': np.array([168, 255, 48]),
            },
            'orange': {
                'label': 'Orange rocket',
                'lower': np.array([10, 97, 20]),
                'upper': np.array([28, 128, 255]),
            },
            'hole': {
                'label': 'Rocket hole',
                'lower': np.array([10, 97, 20]),
                'upper': np.array([28, 128, 255]),
            },
        }
        
        # Tracking states
        self.global_mission_state = "BOOTING"
        self.current_state = "WAITING"
        self.frames_without_trail = 0
        
        # Control Parameters
        self.declare_parameter('linear_speed', 1.0)  # Constant forward speed
        self.declare_parameter('kp_line', 0.005)     # Proportional gain for steering (Tune this!)
        self.declare_parameter('horizon_ratio', 0.45)
        
        self.get_logger().info("Mars Rover Trail Follower Node Started. Awaiting FSM state...")

    def ensure_debug_windows(self):
        if self.debug_windows_ready:
            return

        cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.debug_mask_window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.tuner_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.debug_window_name, 960, 540)
        cv2.resizeWindow(self.debug_mask_window_name, 480, 270)
        cv2.resizeWindow(self.tuner_window_name, 520, 320)
        for name, max_value in [
            ('L-H', 179), ('L-S', 255), ('L-V', 255),
            ('U-H', 179), ('U-S', 255), ('U-V', 255),
        ]:
            cv2.createTrackbar(name, self.tuner_window_name, 0, max_value, self.noop)
        self.debug_windows_ready = True
        self.set_active_color_profile('trail', force=True)

    @staticmethod
    def noop(_value):
        pass

    def set_active_color_profile(self, profile_name, force=False):
        if self.active_color_profile == profile_name and not force:
            return

        if self.active_color_profile is not None:
            self.save_trackbar_values()

        self.active_color_profile = profile_name
        if not self.debug_windows_ready:
            return

        profile = self.color_profiles[profile_name]
        lower = profile['lower']
        upper = profile['upper']
        for trackbar_name, value in [
            ('L-H', int(lower[0])), ('L-S', int(lower[1])), ('L-V', int(lower[2])),
            ('U-H', int(upper[0])), ('U-S', int(upper[1])), ('U-V', int(upper[2])),
        ]:
            cv2.setTrackbarPos(trackbar_name, self.tuner_window_name, value)

    def save_trackbar_values(self):
        if not self.debug_windows_ready or self.active_color_profile is None:
            return

        try:
            lower, upper = self.get_tuned_hsv_range()
        except cv2.error:
            return

        profile = self.color_profiles[self.active_color_profile]
        profile['lower'] = lower
        profile['upper'] = upper

    def get_tuned_hsv_range(self):
        if not self.debug_windows_ready or self.active_color_profile is None:
            profile = self.color_profiles[self.active_color_profile or 'trail']
            return profile['lower'], profile['upper']

        raw_lower = np.array([
            cv2.getTrackbarPos('L-H', self.tuner_window_name),
            cv2.getTrackbarPos('L-S', self.tuner_window_name),
            cv2.getTrackbarPos('L-V', self.tuner_window_name),
        ])
        raw_upper = np.array([
            cv2.getTrackbarPos('U-H', self.tuner_window_name),
            cv2.getTrackbarPos('U-S', self.tuner_window_name),
            cv2.getTrackbarPos('U-V', self.tuner_window_name),
        ])
        lower = np.minimum(raw_lower, raw_upper)
        upper = np.maximum(raw_lower, raw_upper)
        self.color_profiles[self.active_color_profile]['lower'] = lower
        self.color_profiles[self.active_color_profile]['upper'] = upper
        return lower, upper

    def apply_hsv_mask(self, hsv, profile_name):
        try:
            self.ensure_debug_windows()
            self.set_active_color_profile(profile_name)
            lower, upper = self.get_tuned_hsv_range()
        except cv2.error as e:
            profile = self.color_profiles[profile_name]
            lower = profile['lower']
            upper = profile['upper']
            self.get_logger().error(f"Failed to open HSV color selector: {e}", throttle_duration_sec=2.0)

        return cv2.inRange(hsv, lower, upper)

    def global_state_callback(self, msg):
        """ Watches the central master state machine """
        self.global_mission_state = msg.data
        if msg.data == 'FUEL_TRAIL_FOLLOW' and self.current_state == 'WAITING':
            self.current_state = 'FOLLOW_TRAIL'
            self.get_logger().info("FSM activated fuel trail follower. Taking over wheels.")

    def image_callback(self, msg):
        # SAFETY INTERLOCK: Only run if the state machine is explicitly tracking the fuel trail
        if self.global_mission_state != 'FUEL_TRAIL_FOLLOW':
            return

        if self.current_state in ['WAITING', 'DONE']:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        center_screen = w // 2
        
        twist = Twist()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        debug_mask = np.zeros((h, w), dtype=np.uint8)
        horizon_ratio = self.get_parameter('horizon_ratio').get_parameter_value().double_value
        horizon_ratio = max(0.0, min(0.95, horizon_ratio))
        horizon_row = int(h * horizon_ratio)

        # STATE 1: FOLLOW FUEL TRAIL
        if self.current_state in ['FOLLOW_TRAIL', 'SEARCH']:
            # Dual detection: look for orange rocket in every frame
            mask_orange = self.apply_hsv_mask(hsv, 'orange')
            contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_orange = [c for c in contours_orange if cv2.contourArea(c) > 2000]
            
            if len(valid_orange) > 0:
                debug_mask = mask_orange
                self.get_logger().info("Orange rocket detected! Switching to rocket scan.")
                self.current_state = 'SEARCHING_HOLE'
            else:
                mask = self.apply_hsv_mask(hsv, 'trail')
                # Ignore dark sky pixels: only the lower image region can be treated as trail.
                roi_mask = np.zeros((h, w), dtype=np.uint8)
                roi_mask[horizon_row:h, 0:w] = 255
                mask = cv2.bitwise_and(mask, roi_mask)
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                debug_mask = mask
                
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = [c for c in contours if cv2.contourArea(c) > 250]
                
                if len(contours) > 0:
                    self.current_state = 'FOLLOW_TRAIL'
                    self.frames_without_trail = 0 
                    
                    largest_contour = max(contours, key=cv2.contourArea)
                    M = cv2.moments(largest_contour)
                    
                    if M["m00"] > 0:
                        cX = int(M["m10"] / M["m00"])
                        error = center_screen - cX
                        
                        kp_l = self.get_parameter('kp_line').get_parameter_value().double_value
                        linear_v = self.get_parameter('linear_speed').get_parameter_value().double_value
                        
                        if abs(error) > 200:
                            twist.linear.x = linear_v * 0.5 
                        else:
                            twist.linear.x = linear_v
                            
                        twist.angular.z = float(error) * kp_l
                        cv2.circle(frame, (cX, h//2), 10, (0, 255, 0), -1)
                        self.get_logger().info(f"Following trail. Error: {error}", throttle_duration_sec=0.5)
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                else:
                    self.current_state = 'SEARCH'
                    self.frames_without_trail += 1
                    
                    if self.frames_without_trail > 100:
                        self.get_logger().warning("Trail lost for 100 frames. Switching to rocket scan mode.")
                        self.current_state = 'SEARCHING_HOLE'
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.9 

        # STATE 2: SCAN FOR THE ROCKET
        elif self.current_state == 'SEARCHING_HOLE':
            twist.linear.x = 0.0
            twist.angular.z = 0.2 
            
            mask_orange = self.apply_hsv_mask(hsv, 'orange')
            debug_mask = mask_orange
            
            contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [c for c in contours if cv2.contourArea(c) > 2000]
            
            if len(valid_contours) > 0:
                self.get_logger().info("Orange rocket spotted! Looking for the hole...")
                self.current_state = 'ALIGNING_HOLE'

        # STATE 3: CENTER HOLE AND SNAP PHOTO
        elif self.current_state == 'ALIGNING_HOLE':
            mask_white = self.apply_hsv_mask(hsv, 'hole')
            debug_mask = mask_white
            
            contours_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_hole_contours = [c for c in contours_white if cv2.contourArea(c) > 400]
            
            if len(valid_hole_contours) > 0:
                largest_hole = max(valid_hole_contours, key=cv2.contourArea)
                M = cv2.moments(largest_hole)
                
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    error = center_screen - cX
                    
                    cv2.drawContours(frame, [largest_hole], -1, (255, 0, 0), 2)
                    cv2.line(frame, (cX - 15, cY), (cX + 15, cY), (0, 0, 255), 2)
                    cv2.line(frame, (cX, cY - 15), (cX, cY + 15), (0, 0, 255), 2)
                    
                    if abs(error) > 20:
                        twist.linear.x = 0.0
                        twist.angular.z = float(error) * 0.003 
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.publisher_.publish(twist) 
                        
                        photo_path = os.path.join(os.getcwd(), 'rocket_damage_report.jpg')
                        cv2.imwrite(photo_path, frame)
                        self.get_logger().info(f"Photo successfully saved to: {photo_path}")
                        
                        self.vision_done_pub.publish(Bool(data=True))
                        self.call_complete_fuel_trail_service()
                        self.current_state = 'DONE'
            else:
                profile = self.color_profiles['orange']
                mask_orange = cv2.inRange(hsv, profile['lower'], profile['upper'])
                
                contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                valid_orange = [c for c in contours_orange if cv2.contourArea(c) > 2000]
                
                if len(valid_orange) > 0:
                    largest_orange = max(valid_orange, key=cv2.contourArea)
                    M = cv2.moments(largest_orange)
                    if M["m00"] > 0:
                        cX_orange = int(M["m10"] / M["m00"])
                        error_orange = center_screen - cX_orange
                        
                        twist.linear.x = 0.0
                        twist.angular.z = float(error_orange) * 0.003 
                else:
                    self.current_state = 'SEARCHING_HOLE'

        self.publisher_.publish(twist)

        cv2.line(frame, (center_screen, 0), (center_screen, h), (255, 0, 0), 1)
        cv2.line(frame, (0, horizon_row), (w, horizon_row), (0, 0, 255), 2)
        cv2.putText(frame, f"State: {self.current_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if self.active_color_profile is not None:
            profile = self.color_profiles[self.active_color_profile]
            lower = profile['lower']
            upper = profile['upper']
            cv2.putText(
                frame,
                f"Tuning {profile['label']}: L{lower.tolist()} U{upper.tolist()}",
                (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )
            cv2.putText(
                frame,
                f"Mask pixels: {cv2.countNonZero(debug_mask)}",
                (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

        # OpenCV DEBUG WINDOWS: Requires a graphical session, e.g. a local desktop or X forwarding.
        try:
            self.ensure_debug_windows()
            cv2.imshow(self.debug_window_name, frame)
            cv2.imshow(self.debug_mask_window_name, debug_mask)
            cv2.waitKey(1)
        except cv2.error as e:
            self.get_logger().error(f"Failed to open OpenCV debug window: {e}", throttle_duration_sec=2.0)

        # 9. ROS STREAMING: Convert matrices back into ROS topics for rqt_image_view
        try:
            self.debug_cam_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
            self.debug_mask_pub.publish(self.bridge.cv2_to_imgmsg(debug_mask, encoding='mono8'))
        except Exception as e:
            self.get_logger().error(f"Failed to compile debug frames: {e}")

    def stop_rover(self, twist):
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def call_complete_fuel_trail_service(self):
        if not self.complete_fuel_trail_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("Mission manager service not available, will retry on next call.")
            return
        req = Trigger.Request()
        future = self.complete_fuel_trail_client.call_async(req)
        future.add_done_callback(self.complete_fuel_trail_done_callback)

    def complete_fuel_trail_done_callback(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"Mission manager acknowledged: {resp.message}")
            else:
                self.get_logger().error("Mission manager rejected the completion request.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FuelTrailFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission Aborted. Stopping Rover...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()