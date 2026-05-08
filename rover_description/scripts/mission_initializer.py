#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
import utm
import math

class MissionInitializer(Node):
    def __init__(self):
        super().__init__('mission_initializer')
        
        # 1. Declare Parameters (These will be filled by the launch file)
        self.declare_parameter('target_lat', 0.0)
        self.declare_parameter('target_lon', 0.0)

        # 2. Pubs and Subs
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.start_search_pub = self.create_publisher(Bool, '/start_search', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/local', self.odom_cb, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        
        self.origin_utm = None
        self.current_odom_pos = None
        self.target_odom_x = None
        self.target_odom_y = None
        self.goal_sent = False
        self.mission_started = False

        self.get_logger().info("Mission Initializer waiting for GPS origin...")
        self.timer = self.create_timer(1.0, self.mission_control_loop)

    def gps_cb(self, msg):
        if self.origin_utm is None:
            easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
            self.origin_utm = (easting, northing)
            
            # 3. Retrieve the arguments passed via terminal
            t_lat = self.get_parameter('target_lat').get_parameter_value().double_value
            t_lon = self.get_parameter('target_lon').get_parameter_value().double_value
            
            # Convert target to UTM
            t_easting, t_northing, _, _ = utm.from_latlon(t_lat, t_lon)
            self.target_odom_x = t_easting - self.origin_utm[0]
            self.target_odom_y = t_northing - self.origin_utm[1]
            
            self.get_logger().info(f"Target Set: {t_lat}, {t_lon} -> Odom: X={self.target_odom_x:.2f}, Y={self.target_odom_y:.2f}")

    def odom_cb(self, msg):
        self.current_odom_pos = msg.pose.pose.position

    def mission_control_loop(self):
        if self.target_odom_x is None or self.current_odom_pos is None:
            return

        if not self.goal_sent:
            goal = PoseStamped()
            goal.header.stamp = rclpy.time.Time().to_msg()
            goal.header.frame_id = 'odom'
            goal.pose.position.x = self.target_odom_x
            goal.pose.position.y = self.target_odom_y
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.goal_sent = True

        dist = math.sqrt((self.current_odom_pos.x - self.target_odom_x)**2 + 
                         (self.current_odom_pos.y - self.target_odom_y)**2)
        
        if dist < 2.0 and not self.mission_started:
            self.mission_started = True
            self.start_search_pub.publish(Bool(data=True))
            self.get_logger().warn("GPS GOAL REACHED. STARTING SEARCH PHASE.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionInitializer()
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