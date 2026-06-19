"""
Dome exit helper node to autonomously exit from a dome structure.
Processes camera feeds and odometry to detect doors and guide the rover out.
"""

import logging
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String       # Fixed the tokenizer bug
from std_srvs.srv import Trigger       # For notifying Mission Control
from cv_bridge import CvBridge
import cv2
import math

logger = logging.getLogger(__name__)

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
        
        # Subscribers
        self.state_sub = self.create_subscription(String, 'rover/mission_state', self.global_state_callback, qos_profile)
        self.sub_front = self.create_subscription(Image, '/cam/front/image_raw', self.front_callback, 10)
        self.sub_left = self.create_subscription(Image, '/cam/left/image_raw', self.left_callback, 10)
        self.sub_right = self.create_subscription(Image, '/cam/right/image_raw', self.right_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/airlock_debug', 10) 
        
        # Service Client to signal the central FSM that we are done
        self.fsm_client = self.create_client(Trigger, 'mission/complete_dome_exit')
        
        # State Tracking
        self.global_mission_state = "BOOTING"
        self.internal_state = 0 
        self.has_seen_front_door = False
        self.is_finished_reported = False  # Guard to call service exactly once
        
        self.start_odom_x = None
        self.start_odom_y = None
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        
        self.target_forward_speed = 1.0  
        self.angular_kp = 0.003          
        
        self.min_front_contour = 20000    
        self.tripwire_threshold = 15000   
        self.blind_push_distance = 0.9   
        self.kit_threshold_value = 180    

        self.get_logger().info("INTEGRATED DOME EXIT NODE STARTED. Listening to global FSM...")

    def global_state_callback(self, msg):
        """ Listens to the master state machine """
        self.global_mission_state = msg.data

    def odom_callback(self, msg):
        # Interlock: Do nothing unless global state is explicitly DOME_EXIT
        if self.global_mission_state != 'DOME_EXIT':
            return

        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        
        if self.internal_state == 1 and self.start_odom_x is not None:
            dist = math.sqrt((self.current_odom_x - self.start_odom_x)**2 + (self.current_odom_y - self.start_odom_y)**2)
            self.get_logger().info(f"[ODOM DEBUG] Blind Push Distance: {dist:.2f}m / {self.blind_push_distance}m", throttle_duration_sec=0.5)
            
            if dist >= self.blind_push_distance:
                self.get_logger().info(f"==> TAIL CLEARED. Switching to Internal State 2 (Kit Approach).")
                self.internal_state = 2 

    def check_tripwire(self, msg, camera_name):
        # Interlock: Do nothing unless global state is explicitly DOME_EXIT
        if self.global_mission_state != 'DOME_EXIT' or self.internal_state != 0:
            return 
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            self.get_logger().info(f"[{camera_name} CAM DEBUG] Largest Bright Area: {int(area)} pixels", throttle_duration_sec=0.5)
            
            if area > self.tripwire_threshold:
                self.get_logger().info(f"==> TRIPWIRE TRIGGERED BY {camera_name}! Bright Area ({int(area)}) > Threshold ({self.tripwire_threshold})")
                self.start_odom_x = self.current_odom_x
                self.start_odom_y = self.current_odom_y
                self.internal_state = 1

    def left_callback(self, msg):
        self.check_tripwire(msg, "LEFT")

    def right_callback(self, msg):
        self.check_tripwire(msg, "RIGHT")

    def front_callback(self, msg):
        # Interlock: If the global machine isn't running DOME_EXIT, cut motor output completely
        if self.global_mission_state != 'DOME_EXIT':
            # Optional: safe default if another node isn't controlling cmd_vel yet
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape
        image_center_x = width // 2
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        twist = Twist()

        # --- Internal State 0: Track exit door ---
        if self.internal_state == 0:
            _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            door_center_x = None
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                cv2.drawContours(cv_image, [largest_contour], -1, (0, 0, 255), 3)
                self.get_logger().info(f"[FRONT CAM - DOOR] Area: {int(area)}px | Required: {self.min_front_contour}px", throttle_duration_sec=0.5)
                
                if area > self.min_front_contour:
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    door_center_x = x + (w // 2)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    cv2.putText(cv_image, "INTERNAL 0: EXIT DOME", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            if door_center_x is not None:
                self.has_seen_front_door = True 
                twist.linear.x = self.target_forward_speed
                twist.angular.z = -float(door_center_x - image_center_x) * self.angular_kp
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist()) 

        # --- Internal State 1: Blind Push Past Threshold ---
        elif self.internal_state == 1:
            twist.linear.x = self.target_forward_speed
            self.cmd_pub.publish(twist)
            cv2.putText(cv_image, "INTERNAL 1: BLIND PUSH", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # --- Internal State 2: Approach Equipment Kit ---
        elif self.internal_state == 2:
            target_pixel_width = int(126 * (width / 1280.0))
            _, thresh = cv2.threshold(gray, self.kit_threshold_value, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                valid_contours = [c for c in contours if cv2.contourArea(c) > 50]
                
                if valid_contours:
                    largest_contour = max(valid_contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest_contour)
                    cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 255), 3)
                    
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    kit_center_x = x + (w // 2)
                    
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"INTERNAL 2: KIT | Width: {w}/{target_pixel_width}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    bottom_edge = y + h
                    image_bottom_limit = height - 10 
                    
                    # Target criteria check
                    if w >= target_pixel_width or bottom_edge >= image_bottom_limit:
                        self.get_logger().info(f"STOP TRIGGER ACHIEVED! Halting rover.")
                        self.internal_state = 3
                        self.cmd_pub.publish(Twist()) 
                    else:
                        twist.linear.x = self.target_forward_speed
                        twist.angular.z = -float(kit_center_x - image_center_x) * self.angular_kp
                        self.cmd_pub.publish(twist)
                else:
                    twist.linear.x = self.target_forward_speed * 0.5
                    self.cmd_pub.publish(twist)
            else:
                twist.linear.x = self.target_forward_speed * 0.5
                self.cmd_pub.publish(twist)

        # --- Internal State 3: Task Finished, Contact Master FSM ---
        elif self.internal_state == 3:
            self.cmd_pub.publish(Twist()) 
            cv2.putText(cv_image, "INTERNAL 3: ARRIVED AT KIT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            if not self.is_finished_reported:
                self.notify_mission_control_complete()

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

    def notify_mission_control_complete(self):
        """ Contact the central mission master service """
        self.is_finished_reported = True
        
        if not self.fsm_client.service_is_ready():
            self.get_logger().info("Waiting for Mission Manager service server...")
            return
            
        req = Trigger.Request()
        future = self.fsm_client.call_async(req)
        future.add_done_callback(self.fsm_response_callback)

    def fsm_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info("SUCCESS: Central FSM acknowledged dome exit complete. Transitioning out.")
            else:
                self.get_logger().error(f"FSM rejected transition request: {res.message}")
                self.is_finished_reported = False # Allow re-try if master rejected it
        except Exception as e:
            self.get_logger().error(f"Service communication failed: {e}")
            self.is_finished_reported = False

def main(args=None):
    rclpy.init(args=args)
    node = AirlockExitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()