#!/usr/bin/env python3
"""
Mission worker node that drives the rover to the oxygen tank coordinate.

The node is gated by the central mission FSM and only publishes /cmd_vel while
the global state is OXYGEN_TANK_NAV.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu


# Defaults to the arc_oxygen_tank_night_mission pose in mars.world.sdf.
TARGET_X = 25.8
TARGET_Y = -6.0 
ARRIVAL_TOLERANCE = 4.0


class OxygenTankNavigator(Node):
    def __init__(self):
        super().__init__('oxygen_tank_navigator')

        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        self.state_sub = self.create_subscription(
            String,
            'rover/mission_state',
            self.global_state_callback,
            qos_profile
        )
        self.odom_sub = self.create_subscription(Odometry, '/gps/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/filtered', self.imu_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.fsm_client = self.create_client(Trigger, 'mission/complete_oxygen_tank_nav')

        self.current_yaw = 0.0
        self.global_mission_state = 'BOOTING'
        self.is_finished_reported = False
        self.arrived = False

        self.max_linear_speed = 1.5
        self.min_linear_speed = 0.2
        self.max_angular_speed = 0.8
        self.angular_kp = 1.5
        self.slowdown_distance = 3.0
        self.heading_drive_threshold = 0.35

        self.get_logger().info(
            f"OXYGEN TANK NAVIGATOR INITIALIZED. Target: x={TARGET_X:.2f}, "
            f"y={TARGET_Y:.2f}, tolerance={ARRIVAL_TOLERANCE:.2f}m"
        )

    def global_state_callback(self, msg):
        self.global_mission_state = msg.data

    def odom_callback(self, msg):
        if self.global_mission_state != 'OXYGEN_TANK_NAV':
            return

        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        curr_yaw = self.current_yaw

        dx = TARGET_X - curr_x
        dy = TARGET_Y - curr_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= ARRIVAL_TOLERANCE:
            self.arrived = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f"Oxygen tank target reached. Distance: {distance:.2f}m",
                throttle_duration_sec=1.0
            )
            if not self.is_finished_reported:
                self.notify_mission_control_complete()
            return

        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - curr_yaw)

        twist = Twist()
        twist.angular.z = self.clamp(
            self.angular_kp * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        if abs(heading_error) < self.heading_drive_threshold:
            speed_scale = min(distance / self.slowdown_distance, 1.0)
            twist.linear.x = max(self.min_linear_speed, self.max_linear_speed * speed_scale)
        else:
            twist.linear.x = 0.0

        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f"Driving to oxygen tank. Distance: {distance:.2f}m | "
            f"Heading error: {heading_error:.2f}rad",
            throttle_duration_sec=0.5
        )

    def imu_callback(self, msg):
        self.current_yaw = self.quaternion_to_yaw(msg.orientation)

    def notify_mission_control_complete(self):
        self.is_finished_reported = True

        if not self.fsm_client.service_is_ready():
            self.get_logger().info("Waiting for Mission Manager oxygen-tank service...")
            self.is_finished_reported = False
            return

        req = Trigger.Request()
        future = self.fsm_client.call_async(req)
        future.add_done_callback(self.fsm_response_callback)

    def fsm_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info("SUCCESS: Central FSM acknowledged oxygen tank navigation complete.")
            else:
                self.get_logger().error(f"FSM rejected oxygen-tank completion: {res.message}")
                self.is_finished_reported = False
        except Exception as e:
            self.get_logger().error(f"Service communication failed: {e}")
            self.is_finished_reported = False

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))


def main(args=None):
    rclpy.init(args=args)
    node = OxygenTankNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
