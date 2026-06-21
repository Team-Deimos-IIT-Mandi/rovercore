#!/usr/bin/env python3
"""
Drone ArUco Relay — receives ArUco detections from the companion drone,
transforms them into rover world frame, and publishes as Nav2 goals.

Silences automatically once ground-based aruco_detection locks the marker
(receives /marker_goal_reached=True).

Published topics:
  /aruco/world_pose  (geometry_msgs/PoseStamped) — marker in map frame

Subscribed topics:
  /drone/aruco_pose       (geometry_msgs/PoseStamped) — marker in drone body frame
  /mavros/local_position/odom (nav_msgs/Odometry)    — drone world position
  /marker_goal_reached    (std_msgs/Bool)             — silences relay when True

Parameters:
  relay_timeout_sec  (float) — stop relaying if no drone msg for this long (default 5.0)
  map_frame          (str)   — world frame id (default "map")
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.time import Time


class DroneArucoRelay(Node):
    def __init__(self):
        super().__init__('drone_aruco_relay')

        self.declare_parameter('relay_timeout_sec', 5.0)
        self.declare_parameter('map_frame', 'map')

        self._timeout = self.get_parameter('relay_timeout_sec').value
        self._map_frame = self.get_parameter('map_frame').value

        self._silenced = False
        self._drone_odom: Odometry | None = None
        self._last_drone_msg_time = None

        self._world_pose_pub = self.create_publisher(
            PoseStamped, '/aruco/world_pose', 10)

        self.create_subscription(
            PoseStamped, '/drone/aruco_pose', self._drone_aruco_cb, 10)
        self.create_subscription(
            Odometry, '/mavros/local_position/odom', self._drone_odom_cb, 10)
        self.create_subscription(
            Bool, '/marker_goal_reached', self._goal_reached_cb, 10)

        self.create_timer(1.0, self._timeout_check)
        self.get_logger().info('Drone ArUco relay started')

    def _drone_odom_cb(self, msg: Odometry):
        self._drone_odom = msg

    def _goal_reached_cb(self, msg: Bool):
        if msg.data and not self._silenced:
            self.get_logger().info('Ground aruco locked — silencing drone relay')
            self._silenced = True

    def _drone_aruco_cb(self, msg: PoseStamped):
        if self._silenced:
            return
        if self._drone_odom is None:
            self.get_logger().warn('No drone odom yet — skipping ArUco relay')
            return

        self._last_drone_msg_time = self.get_clock().now()

        # Transform drone body-frame ArUco pose to world frame.
        # Drone odom gives drone position in world frame.
        # Simple translation: world_pos = drone_world_pos + body_offset
        # (rotation from drone orientation is applied via quaternion)
        dx = msg.pose.position.x
        dy = msg.pose.position.y
        dz = msg.pose.position.z

        drone_x = self._drone_odom.pose.pose.position.x
        drone_y = self._drone_odom.pose.pose.position.y
        # Yaw from drone quaternion
        q = self._drone_odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        world_x = drone_x + dx * math.cos(yaw) - dy * math.sin(yaw)
        world_y = drone_y + dx * math.sin(yaw) + dy * math.cos(yaw)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._map_frame
        out.pose.position.x = world_x
        out.pose.position.y = world_y
        out.pose.position.z = 0.0
        out.pose.orientation.w = 1.0
        self._world_pose_pub.publish(out)
        self.get_logger().debug(f'Relayed drone ArUco → world ({world_x:.2f}, {world_y:.2f})')

    def _timeout_check(self):
        if self._silenced or self._last_drone_msg_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_drone_msg_time).nanoseconds * 1e-9
        if elapsed > self._timeout:
            self.get_logger().warn(
                f'Drone ArUco silent for {elapsed:.1f}s (timeout={self._timeout}s)')


def main(args=None):
    rclpy.init(args=args)
    node = DroneArucoRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
