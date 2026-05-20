'''
Made by: Reman Dey
Roll no.: b25331
Task: fuel trail follower node for mars rover
Tunings needed: i)[line37]Kp value for steering control 
               ii)[line51-52]HSV range for fuel trail detection (we need to test and tune it based on actual color of trail in Gazebo) 
              iii)[line26]Camera topic name (we need to change it based on our setup in Gazebo)

'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class FuelTrailFollower(Node):
    def __init__(self):
        super().__init__('fuel_trail_follower')
        
        # Subscriber to the camera topic (we need to change the topic name)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publisher for rover movement
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        
        # Control Parameters
        self.declare_parameter('linear_speed', 0.2)  # Constant forward speed
        self.declare_parameter('kp', 0.005)          # Proportional gain for steering-yeh change krna ho skta hain- we need to tune it
        
        self.get_logger().info("Mars Rover Trail Follower Node Started")

    def image_callback(self, msg):
        #converting ros image to opencv format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        center_screen = w // 2

        # changing colors to hsv for segmentation of fuel trail
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define 'shimmer' (hume yeh change krna padega based on actual color of trail)
        lower_shimmer = np.array([0, 0, 200])   # Bright white/cyan glow
        upper_shimmer = np.array([180, 50, 255]) 
        
        mask = cv2.inRange(hsv, lower_shimmer, upper_shimmer)
        
        #Clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        #Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        twist = Twist()

        if len(contours) > 0:
            # Get the largest contour (the fuel trail)
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M["m00"] > 0:
                # Calculate center of the trail (cX)
                cX = int(M["m10"] / M["m00"])
                
                # Calculate error (Distance from center of screen)
                error = center_screen - cX
                
                # Proportional Control
                kp = self.get_parameter('kp').get_parameter_value().double_value
                linear_v = self.get_parameter('linear_speed').get_parameter_value().double_value
                
                # Set velocities
                twist.linear.x = linear_v
                twist.angular.z = float(error) * kp
                
                self.get_logger().info(f"Following trail. Error: {error}")
            else:
                self.stop_rover(twist)
        else:
            # No trail detected - stop and rotate to search
            self.get_logger().warn("Trail lost! Searching...")
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # Slow rotation to scan for shimmering
            
        self.publisher_.publish(twist)

        # Debugging window(we can remove this/must remove this for deployment)
        cv2.circle(frame, (cX, h//2), 10, (0, 255, 0), -1)
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

    def stop_rover(self, twist):
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FuelTrailFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission Aborted. Stopping Rover...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()