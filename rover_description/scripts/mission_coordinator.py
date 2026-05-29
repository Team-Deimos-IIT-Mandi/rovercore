#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')
        
        # 1. Nav2 Action Client (The Muscles)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 2. OpenCV Pub/Sub (The Eyes)
        self.start_vision_pub = self.create_publisher(Bool, '/start_fuel_trail', 10)
        self.vision_done_sub = self.create_subscription(Bool, '/vision_task_complete', self.vision_done_cb, 10)

        # 3. Mission Waypoints (X, Y)
        # Using parameters so you can change them in the launch file
        self.declare_parameter('wp_astronaut', [16.0, -12.0])
        self.declare_parameter('wp_oxygen', [23.8, -6.0])
        self.declare_parameter('wp_base', [-2.0, 3.5])

        # 4. State Machine Tracker
        self.current_phase = 'INIT'
        self.get_logger().info("Mission Coordinator waiting for Nav2 Server...")
        
        # Wait for Nav2 to spin up
        self.nav_client.wait_for_server()
        
        self.get_logger().info("Nav2 Server is up. Waiting 15 seconds for TF tree to establish...")
        time.sleep(15.0)

        self.get_logger().info("Nav2 Online! Starting Mission Phase 1: Astronaut.")
        
        self.current_phase = 'DRIVE_ASTRONAUT'
        self.execute_phase()

    def execute_phase(self):
        """State Machine to handle the sequence of the mission."""
        if self.current_phase == 'DRIVE_ASTRONAUT':
            wp = self.get_parameter('wp_astronaut').value
            self.send_nav_goal(wp[0], wp[1], next_phase='VISION_TASKS')
            
        elif self.current_phase == 'VISION_TASKS':
            self.get_logger().info("Arrived at Astronaut. Triggering OpenCV Fuel Trail...")
            self.start_vision_pub.publish(Bool(data=True))
            # The script now pauses here. It waits for vision_done_cb() to trigger.
            
        elif self.current_phase == 'DRIVE_OXYGEN':
            self.get_logger().info("Photo confirmed! Resuming Nav2. Navigating to Oxygen...")
            wp = self.get_parameter('wp_oxygen').value
            self.send_nav_goal(wp[0], wp[1], next_phase='DRIVE_BASE')
            
        elif self.current_phase == 'DRIVE_BASE':
            self.get_logger().info("Navigating back to Base...")
            wp = self.get_parameter('wp_base').value
            self.send_nav_goal(wp[0], wp[1], next_phase='COMPLETE')
            
        elif self.current_phase == 'COMPLETE':
            self.get_logger().info("MISSION COMPLETE! Rover securing systems.")

    # -----------------------------------------------------------
    # Navigation Methods (Talking to Nav2)
    # -----------------------------------------------------------
    def send_nav_goal(self, x, y, next_phase):
        self.target_next_phase = next_phase # Store what phase to jump to next
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0 

        # Send goal asynchronously so we don't freeze the node
        self.get_logger().info(f"Sending Nav2 Goal: X={x:.2f}, Y={y:.2f}")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 REJECTED the goal. Check costmaps or obstacles!")
            return

        self.get_logger().info("Goal accepted by Nav2. Driving...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result().status
        if result == 4: # 4 = SUCCEEDED
            self.get_logger().info("Destination Reached successfully.")
            self.current_phase = self.target_next_phase
            self.execute_phase()
        else:
            self.get_logger().error(f"Nav2 failed with status code: {result}")

    # -----------------------------------------------------------
    # Vision Methods (Talking to OpenCV)
    # -----------------------------------------------------------
    def vision_done_cb(self, msg):
        """Triggered when your separate OpenCV script says it finished taking the photo."""
        if msg.data and self.current_phase == 'VISION_TASKS':
            self.current_phase = 'DRIVE_OXYGEN'
            self.execute_phase()

def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
