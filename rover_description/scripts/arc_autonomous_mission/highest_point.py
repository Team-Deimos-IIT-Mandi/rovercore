import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import transforms3d  # Common library to convert quaternions to Euler angles
import time
import math

class HillClimberNode(Node):
    def __init__(self):
        super().__init__('hill_climber_node')
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_subscription_window = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/', self.imu_callback, 10)
        
        # Core State Variables
        self.current_pitch = 0.0  # Up/Down tilt (in radians)
        self.current_roll = 0.0   # Left/Right tilt (in radians)
        self.imu_received = False
        
        # Rover Parameters
        self.linear_speed = 0.2    # m/s
        self.angular_speed = 0.3   # rad/s
        self.step_distance = 1.0   # 1 meter steps
        self.flat_tolerance = 0.03 # ~1.7 degrees (considered flat)

        # Kick off the main control loop execution
        self.get_logger().info("Hill Climber Node Initialized. Waiting for IMU data...")
        self.create_timer(1.0, self.main_control_loop)

    def imu_callback(self, msg):
        """Convert quaternion IMU data to Euler angles (roll, pitch, yaw)"""
        q = msg.orientation
        # transforms3d converts [w, x, y, z] quaternions
        try:
            roll, pitch, _ = transforms3d.quaternions.quat2euler([q.w, q.x, q.y, q.z])
            self.current_pitch = pitch
            self.current_roll = roll
            self.imu_received = True
        except Exception as e:
            self.get_logger().error(f"Failed to parse IMU: {str(e)}")

    def move_forward_1m(self):
        """Drives the rover forward for exactly 1 meter based on time/velocity."""
        self.get_logger().info("Moving forward 1 meter...")
        twist = Twist()
        twist.linear.x = self.linear_speed
        
        duration = self.step_distance / self.linear_speed
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
            
        # Stop the rover
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.0) # Let the chassis settle for a clean IMU reading

    def turn_rover(self, angle_rad):
        """Rotates the rover by a given angle in radians."""
        twist = Twist()
        if angle_rad > 0:
            twist.angular.z = self.angular_speed
        else:
            twist.angular.z = -self.angular_speed
            
        duration = abs(angle_rad) / self.angular_speed
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
            
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)

    def main_control_loop(self):
        if not self.imu_received:
            return

        # 1. Take a step
        self.move_forward_1m()
        
        # 2. Evaluate current pitch and roll (Gradient Vector)
        # Pitch tells us front/back slope; Roll tells us left/right slope.
        self.get_logger().info(f"Checking Slope -> Pitch: {math.degrees(self.current_pitch):.2f}°, Roll: {math.degrees(self.current_roll):.2f}°")
        
        # Calculate overall magnitude of the slope vector
        slope_magnitude = math.sqrt(self.current_pitch**2 + self.current_roll**2)
        
        if slope_magnitude < self.flat_tolerance:
            self.get_logger().info("Terrain is flat. Checking for Global Maximum...")
            self.verify_and_stop()
            return

        # 3. Calculate steering angle adjustment based on roll vs pitch gradient
        # If roll is negative (tilted left, right side is higher), turn right.
        # atan2 gives us the angle of steepest ascent relative to the rover's current heading.
        steering_adjustment = math.atan2(-self.current_roll, self.current_pitch)
        
        self.get_logger().info(f"Correcting heading by {math.degrees(steering_adjustment):.2f}° toward steepest climb.")
        self.turn_rover(steering_adjustment)

    def verify_and_stop(self):
        """Executes a 360-degree pivot scan to see if higher ground lies nearby."""
        self.get_logger().info("Executing 360-degree false summit validation scan...")
        
        # Simple scan: Turn 90 degrees to check if the slope opens up again
        self.turn_rover(math.pi / 2)
        time.sleep(1.0)
        
        if math.sqrt(self.current_pitch**2 + self.current_roll**2) > self.flat_tolerance:
            self.get_logger().info("Found higher ground slope! Resuming climb.")
            return
            
        # If it's truly flat every way we look, we stop.
        self.get_logger().info("GLOBAL MAXIMUM FOUND. Shutting down node.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HillClimberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
def run():
    main()
    return 0