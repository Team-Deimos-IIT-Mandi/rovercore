#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # 1. Subscribers
        self.cam_sub = self.create_subscription(Image, '/cam/front/image_raw', self.image_callback, 10)
        # NEW: Listen for the trigger from mission_initializer.py
        self.trigger_sub = self.create_subscription(Bool, '/start_fuel_trail', self.start_callback, 10)
        
        # 2. Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # NEW: Tell mission_initializer.py we are done taking the photo
        self.vision_done_pub = self.create_publisher(Bool, '/vision_task_complete', 10)
        
        self.bridge = CvBridge()
        
        # 3. Mission States
        # Removed GO_TO_ASTRONAUT. Replaced with WAITING.
        self.current_state = 'WAITING'
        
        # Parameters for OpenCV driving
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('kp_line', 0.005) 
        
        # Counter to know when the trail actually ends at the rocket
        self.frames_without_trail = 0

        self.get_logger().info("Vision Node Ready: Waiting for trigger from Mission Initializer...")

    def start_callback(self, msg):
        """Wakes up the script when mission_initializer reaches the astronaut."""
        if msg.data and self.current_state == 'WAITING':
            self.current_state = 'FOLLOW_TRAIL'
            self.get_logger().info("Trigger received! Taking over wheels to follow fuel trail.")

    def image_callback(self, msg):
        # Do nothing if we haven't been triggered yet, or if we are already done
        if self.current_state in ['WAITING', 'DONE']:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        center_screen = w // 2
        
        twist = Twist()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ─── STATE 1: FOLLOW FUEL TRAIL ───
        if self.current_state in ['FOLLOW_TRAIL', 'SEARCH']:
            lower_shimmer = np.array([40, 50, 80])
            upper_shimmer = np.array([90, 255, 255]) 
            
            mask = cv2.inRange(hsv, lower_shimmer, upper_shimmer)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                self.current_state = 'FOLLOW_TRAIL'
                self.frames_without_trail = 0 # Reset counter because we see the trail
                
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
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
            else:
                self.current_state = 'SEARCH'
                self.frames_without_trail += 1
                
                # If we lose the trail for 40 frames, assume we arrived at the rocket base
                if self.frames_without_trail > 40:
                    self.get_logger().warning("Trail ended. Switching to Rocket Scan mode.")
                    self.current_state = 'SEARCHING_HOLE'
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3 # Spin to find the lost trail

        # ─── STATE 2: SCAN FOR THE GREY HOLE ───
        elif self.current_state == 'SEARCHING_HOLE':
            twist.linear.x = 0.0
            twist.angular.z = 0.2 # Spin slowly
            
            # Low saturation grey HSV limits
            lower_grey = np.array([0, 0, 50])
            upper_grey = np.array([180, 60, 200]) 
            mask = cv2.inRange(hsv, lower_grey, upper_grey)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [c for c in contours if cv2.contourArea(c) > 500]
            
            if len(valid_contours) > 0:
                self.get_logger().info("Target hole spotted! Aligning...")
                self.current_state = 'ALIGNING_HOLE'

        # ─── STATE 3: CENTER HOLE AND SNAP PHOTO ───
        elif self.current_state == 'ALIGNING_HOLE':
            lower_grey = np.array([0, 0, 50])
            upper_grey = np.array([180, 60, 200]) 
            mask = cv2.inRange(hsv, lower_grey, upper_grey)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    error = center_screen - cX
                    
                    if abs(error) > 20:
                        twist.linear.x = 0.0
                        twist.angular.z = float(error) * 0.005
                    else:
                        # Centered! Hit the brakes
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_pub.publish(twist) 
                        
                        # Save the photo
                        photo_path = os.path.join(os.getcwd(), 'rocket_damage_report.jpg')
                        cv2.imwrite(photo_path, frame)
                        self.get_logger().info(f"Photo successfully saved to: {photo_path}")
                        
                        # Hand control back to mission_initializer.py
                        self.vision_done_pub.publish(Bool(data=True))
                        self.current_state = 'DONE'
            else:
                self.current_state = 'SEARCHING_HOLE'

        # Publish the motor commands
        self.cmd_pub.publish(twist)

        # Show debugging frames
        cv2.putText(frame, f"State: {self.current_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)
   
def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission Aborted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
