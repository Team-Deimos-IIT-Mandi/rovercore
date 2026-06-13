#!/usr/bin/env python3
"""
GPS Navigation Node — Phase 1.2
Sequences through mission waypoints from a YAML file, converting lat/lon to
map-frame goals via UTM and driving Nav2's NavigateToPose action client.

Parameters:
  mission_file      (string)  — absolute path to the waypoints YAML
  arrival_tolerance (float)   — radius in metres to consider a waypoint reached (default 0.5)

Published topics:
  /mission/waypoint_index  (std_msgs/Int32)   — 0-based index of the current waypoint
  /mission/status          (std_msgs/String)  — NAVIGATING | ARRIVED | COMPLETE | FAILED

Subscribed topics:
  /odometry/local   (nav_msgs/Odometry)   — current pose in odom frame
  /gps/odom_gated   (nav_msgs/Odometry)   — GPS expressed in odom frame (navsat_transform)

═══════════════════════════════════════════════════════════════════════════
BUG FIXED (2026-06):
  DATUM_LAT / DATUM_LON were hardcoded to MDRS Utah (38.4063, -110.7918).
  The mars world SDF sets the spherical origin at (39.8942, 32.7845) — a
  completely different location.  All GPS→map conversions were off by
  thousands of kilometres.  Fixed to match mars.world.sdf exactly.
═══════════════════════════════════════════════════════════════════════════
"""

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import utm  # python-utm

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String

from ament_index_python.packages import get_package_share_directory


class GpsNavNode(Node):
    """Waypoint-sequencing GPS navigation node."""

    # ── World datum — MUST match mars.world.sdf <spherical_coordinates> ────
    # FIX: was (38.4063, -110.7918) = MDRS Utah — completely wrong location.
    DATUM_LAT = 39.8942
    DATUM_LON = 32.7845

    STATUS_IDLE       = "IDLE"
    STATUS_NAVIGATING = "NAVIGATING"
    STATUS_ARRIVED    = "ARRIVED"
    STATUS_COMPLETE   = "COMPLETE"
    STATUS_FAILED     = "FAILED"

    def __init__(self):
        super().__init__("gps_nav_node")

        default_mission_file = os.path.join(
            get_package_share_directory("rover_description"),
            "config", "mission_waypoints.yaml",
        )
        self.declare_parameter("mission_file", default_mission_file)
        self.declare_parameter("arrival_tolerance", 0.5)

        mission_file = (
            self.get_parameter("mission_file").get_parameter_value().string_value
        )
        self.arrival_tolerance = (
            self.get_parameter("arrival_tolerance").get_parameter_value().double_value
        )

        self.waypoints      = []
        self.wp_index       = 0
        self.origin_utm     = None
        self.current_pos    = None
        self.gps_odom_pos   = None
        self.mission_active = False
        self.goal_handle    = None
        self.nav_in_flight  = False
        self.status         = self.STATUS_IDLE

        self._load_waypoints(mission_file)

        self.wp_index_pub = self.create_publisher(Int32,  "/mission/waypoint_index", 10)
        self.status_pub   = self.create_publisher(String, "/mission/status",          10)

        cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            Odometry, "/odometry/local",  self._odom_cb,     10, callback_group=cb_group)
        self.create_subscription(
            Odometry, "/gps/odom_gated",  self._gps_odom_cb, 10, callback_group=cb_group)

        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=cb_group)

        self.create_timer(0.5, self._mission_loop, callback_group=cb_group)

        self.get_logger().info(
            f"GPS Nav Node started. {len(self.waypoints)} waypoints from '{mission_file}'. "
            f"Datum: lat={self.DATUM_LAT}, lon={self.DATUM_LON}. "
            f"Arrival tolerance: {self.arrival_tolerance} m."
        )

    # ── Waypoint loading ────────────────────────────────────────────────────
    def _load_waypoints(self, path: str):
        if not os.path.isfile(path):
            self.get_logger().error(f"Mission file not found: '{path}'")
            return
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        try:
            self.waypoints = data["mission"]["waypoints"]
            self.get_logger().info(
                f"Mission '{data['mission']['name']}' — "
                f"{len(self.waypoints)} waypoints loaded."
            )
        except (KeyError, TypeError) as e:
            self.get_logger().error(f"Invalid mission YAML structure: {e}")

    # ── Callbacks ───────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.current_pos = msg.pose.pose.position

    def _gps_odom_cb(self, msg: Odometry):
        self.gps_odom_pos = msg.pose.pose.position
        if self.origin_utm is None:
            self.origin_utm = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.get_logger().info("GPS odom origin anchored. Starting mission sequencer.")
            self.mission_active = True

    # ── UTM conversion using the corrected datum ────────────────────────────
    def _latlon_to_map(self, lat: float, lon: float):
        """
        Convert a lat/lon waypoint to map-frame (x, y) using UTM.

        Uses the world datum from mars.world.sdf (was wrong MDRS coords before).
        """
        datum_e, datum_n, _, _ = utm.from_latlon(self.DATUM_LAT, self.DATUM_LON)
        target_e, target_n, _, _ = utm.from_latlon(lat, lon)
        return target_e - datum_e, target_n - datum_n

    # ── Navigation helpers ──────────────────────────────────────────────────
    def _build_goal(self, map_x: float, map_y: float) -> NavigateToPose.Goal:
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = map_x
        pose.pose.position.y = map_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _publish_status(self, status: str):
        self.status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_wp_index(self):
        msg = Int32()
        msg.data = self.wp_index
        self.wp_index_pub.publish(msg)

    def _distance_to_goal(self, map_x: float, map_y: float) -> float:
        if self.current_pos is None:
            return float("inf")
        return math.sqrt(
            (self.current_pos.x - map_x) ** 2 +
            (self.current_pos.y - map_y) ** 2
        )

    # ── Mission loop ────────────────────────────────────────────────────────
    def _mission_loop(self):
        if not self.mission_active or not self.waypoints:
            return

        if self.wp_index >= len(self.waypoints):
            if self.status != self.STATUS_COMPLETE:
                self._publish_status(self.STATUS_COMPLETE)
                self.get_logger().info("All waypoints complete. Mission COMPLETE.")
            return

        wp = self.waypoints[self.wp_index]
        self._publish_wp_index()

        lat = float(wp["lat"])
        lon = float(wp["lon"])
        if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
            self.get_logger().error(
                f"Invalid coords for WP '{wp.get('label', self.wp_index)}': "
                f"lat={lat}, lon={lon}. Skipping."
            )
            self.wp_index += 1
            return

        map_x, map_y = self._latlon_to_map(lat, lon)

        if not self.nav_in_flight:
            if self._distance_to_goal(map_x, map_y) < self.arrival_tolerance:
                self._on_waypoint_reached(wp)
                return

            if not self._nav_client.server_is_ready():
                self.get_logger().warn(
                    "NavigateToPose action server not ready — waiting…",
                    throttle_duration_sec=5.0,
                )
                return

            self.get_logger().info(
                f"Navigating to WP[{self.wp_index}] '{wp.get('label', '')}' "
                f"lat={lat:.6f} lon={lon:.6f} → map ({map_x:.2f}, {map_y:.2f})"
            )
            self._publish_status(self.STATUS_NAVIGATING)
            self.nav_in_flight = True

            goal = self._build_goal(map_x, map_y)
            send_future = self._nav_client.send_goal_async(
                goal, feedback_callback=self._feedback_cb)
            send_future.add_done_callback(
                lambda fut: self._goal_response_cb(fut, wp, map_x, map_y))

    def _feedback_cb(self, feedback_msg):
        pass

    def _goal_response_cb(self, future, wp, map_x, map_y):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f"Goal for WP '{wp.get('label', self.wp_index)}' REJECTED by Nav2.")
            self._publish_status(self.STATUS_FAILED)
            self.nav_in_flight = False
            return
        self.goal_handle   = goal_handle
        result_future      = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self._result_cb(fut, wp, map_x, map_y))

    def _result_cb(self, future, wp, map_x, map_y):
        self.nav_in_flight = False
        result = future.result()
        if result.status == 4:      # GoalStatus.SUCCEEDED
            self._on_waypoint_reached(wp)
        else:
            self.get_logger().error(
                f"Navigation to '{wp.get('label', self.wp_index)}' failed "
                f"(Nav2 status={result.status})."
            )
            self._publish_status(self.STATUS_FAILED)

    def _on_waypoint_reached(self, wp):
        self.get_logger().info(
            f"ARRIVED at WP[{self.wp_index}] '{wp.get('label', '')}' "
            f"(type={wp.get('type', 'gps_nav')})"
        )
        self._publish_status(self.STATUS_ARRIVED)
        self.wp_index += 1
        self._publish_wp_index()


def main(args=None):
    rclpy.init(args=args)
    node = GpsNavNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
