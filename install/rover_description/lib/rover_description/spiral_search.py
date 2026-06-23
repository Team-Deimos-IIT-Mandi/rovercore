#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
import math

class SpiralWaypointGenerator(Node):
    def __init__(self):
        super().__init__('spiral_waypoint_generator')

        # Parameters
        self.declare_parameter('threshold_distance', 2.0)
        self.declare_parameter('spiral_radius', 30.0)
        self.declare_parameter('step_increment', 0.1)
        self.threshold_distance = self.get_parameter('threshold_distance').get_parameter_value().double_value
        self.spiral_radius = self.get_parameter('spiral_radius').get_parameter_value().double_value
        self.step_increment = self.get_parameter('step_increment').get_parameter_value().double_value
        self.mission_started = False
        self.initial_pose = None
        self.waypoints = []
        
        self.current_position = None
        self.ar_active = False

        # QoS Profiles
        qos_profile = 10

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry/local', self.odom_callback, qos_profile)
        self.ar_subscriber = self.create_subscription(Bool, '/AR', self.ar_callback, qos_profile)
        self.start_sub = self.create_subscription(Bool, '/start_search', self.start_cb, 10)
        
        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)
        self.marker_publisher = self.create_publisher(Marker, '/waypoints_marker', qos_profile)

        # Timer
        self.timer = self.create_timer(0.5, self.control_loop)

        self.get_logger().info("Spiral Search Node Started")

    def start_cb(self, msg):
        if msg.data:
            self.mission_started = True
            self.get_logger().info("Search signal received. Starting operation!")

    def ar_callback(self, msg):
        if msg.data:
            self.ar_active = True
            self.waypoints = [] 
            self.get_logger().warn("SPIRAL ABORTED: Marker detected.")

    def generate_spiral_waypoints(self):
        waypoints = []
        t = 0.0
        while t <= self.spiral_radius:
            x = self.initial_pose.x + (math.cos(t) * t / 3.0)
            y = self.initial_pose.y + (math.sin(t) * t / 3.0)
            
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'odom'
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position = Point(x=x, y=y, z=0.0)
            
            tangent_angle = math.atan2(y + t * math.cos(t), x - t * math.sin(t))
            q = self.euler_to_quaternion(0, 0, tangent_angle)
            
            waypoint.pose.orientation.x = q[0]
            waypoint.pose.orientation.y = q[1]
            waypoint.pose.orientation.z = q[2]
            waypoint.pose.orientation.w = q[3]

            waypoints.append(waypoint)
            t += self.step_increment 
        
        return waypoints

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def publish_waypoints_markers(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'spiral_waypoints'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2; marker.scale.y = 0.2
        marker.color.a = 1.0; marker.color.r = 1.0

        for wp in self.waypoints:
            marker.points.append(wp.pose.position)

        self.marker_publisher.publish(marker)

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def control_loop(self):
        if not self.mission_started: return
        if self.ar_active or self.current_position is None: return

        if self.initial_pose is None:
            self.initial_pose = self.current_position
            self.waypoints = self.generate_spiral_waypoints()
            self.get_logger().info(f"Generated {len(self.waypoints)} waypoints.")
        
        self.publish_waypoints_markers()
        
        if self.waypoints:
            if self.ar_active: return
            next_wp = self.waypoints[0].pose.position
            distance = math.sqrt((self.current_position.x - next_wp.x)**2 +
                                 (self.current_position.y - next_wp.y)**2)
            
            if distance < self.threshold_distance:
                goal_msg = self.waypoints.pop(0)
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                self.goal_publisher.publish(goal_msg)
                self.get_logger().info("Next waypoint published.")
        else:
            if not hasattr(self, '_search_finished_logged'):
                self._search_finished_logged = True
                self.get_logger().info("Spiral Search Finished")

def main(args=None):
    rclpy.init(args=args)
    node = SpiralWaypointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()