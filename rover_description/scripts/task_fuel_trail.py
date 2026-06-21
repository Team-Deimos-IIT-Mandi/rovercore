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

class VisionWorker(Node):
    def __init__(self):
        super().__init__('vision_worker')
        
        # 1. Subscribers
        self.cam_sub = self.create_subscription(Image, '/cam/front/image_raw', self.image_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, '/start_fuel_trail', self.start_callback, 10)
        
        # 2. Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vision_done_pub = self.create_publisher(Bool, '/vision_task_complete', 10)
        
        self.bridge = CvBridge()
        
        # 3. Mission States
        self.current_state = 'WAITING'
        
        # Parameters for OpenCV driving
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('kp_line', 0.005) 
        
        # Counter to know when the trail ends
        self.frames_without_trail = 0

        self.get_logger().info("Vision Worker Ready. Waiting for Nav2 to reach the Astronaut...")

    def start_callback(self, msg):
        if msg.data and self.current_state == 'WAITING':
            self.current_state = 'FOLLOW_TRAIL'
            self.get_logger().info("Trigger received! Taking over wheels to follow fuel trail.")

    def image_callback(self, msg):
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
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
            else:
                self.current_state = 'SEARCH'
                self.frames_without_trail += 1
                
                if self.frames_without_trail > 40:
                    self.get_logger().warning("Trail ended. Switching to Rocket Scan mode.")
                    self.current_state = 'SEARCHING_HOLE'
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3 

        # ─── STATE 2: SCAN FOR THE ROCKET ───
        elif self.current_state == 'SEARCHING_HOLE':
            twist.linear.x = 0.0
            twist.angular.z = 0.2 
            
            lower_orange = np.array([5, 120, 120])
            upper_orange = np.array([25, 255, 255]) 
            mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
            
            contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [c for c in contours if cv2.contourArea(c) > 2000]
            
            if len(valid_contours) > 0:
                self.get_logger().info("Orange rocket spotted! Looking for the hole...")
                self.current_state = 'ALIGNING_HOLE'

        # ─── STATE 3: CENTER HOLE AND SNAP PHOTO ───
        elif self.current_state == 'ALIGNING_HOLE':
            # 1. Target the damage using the tuned HSV values
            lower_white = np.array([0, 0, 38])
            upper_white = np.array([179, 126, 255]) 
            mask_white = cv2.inRange(hsv, lower_white, upper_white)
            
            contours_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_hole_contours = [c for c in contours_white if cv2.contourArea(c) > 400]
            
            if len(valid_hole_contours) > 0:
                # TARGET FOUND! Align precisely to the shape.
                largest_hole = max(valid_hole_contours, key=cv2.contourArea)
                M = cv2.moments(largest_hole)
                
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    error = center_screen - cX
                    
                    # Draw a blue outline around the damage and a red crosshair targeting the actual hole
                    cv2.drawContours(frame, [largest_hole], -1, (255, 0, 0), 2)
                    cv2.line(frame, (cX - 15, cY), (cX + 15, cY), (0, 0, 255), 2)
                    cv2.line(frame, (cX, cY - 15), (cX, cY + 15), (0, 0, 255), 2)
                    
                    if abs(error) > 20:
                        twist.linear.x = 0.0
                        twist.angular.z = float(error) * 0.003 
                    else:
                        # Centered! Hit the brakes
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_pub.publish(twist) 
                        
                        # Save the photo
                        photo_path = os.path.join(os.getcwd(), 'rocket_damage_report.jpg')
                        cv2.imwrite(photo_path, frame)
                        self.get_logger().info(f"Photo successfully saved to: {photo_path}")
                        
                        # Hand control back to mission_coordinator.py
                        self.vision_done_pub.publish(Bool(data=True))
                        self.current_state = 'DONE'
            else:
                # TARGET LOST! Lock onto the ORANGE rocket to keep it in frame.
                lower_orange = np.array([5, 120, 120])
                upper_orange = np.array([25, 255, 255]) 
                mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
                
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
                    # We lost both the target AND the rocket. Back to scanning.
                    self.current_state = 'SEARCHING_HOLE'

        self.cmd_pub.publish(twist)

        cv2.putText(frame, f"State: {self.current_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)
   
def main(args=None):
    rclpy.init(args=args)
    node = VisionWorker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Vision Node Aborted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()