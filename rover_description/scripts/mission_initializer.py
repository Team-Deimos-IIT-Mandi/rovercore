#!/usr/bin/env python3
"""
MissionInitializer — sends a Nav2 goal derived from GPS lat/lon parameters.

Parameters (must match full_autonomy.launch.py):
  target_lat          (float)  — target GPS latitude   (default = world datum lat)
  target_lon          (float)  — target GPS longitude  (default = world datum lon)
  arrival_threshold   (float)  — distance in metres to consider arrived (default 2.0)

The GPS coordinates are converted to map-frame (x, y) using the world datum
declared in mars.world.sdf <spherical_coordinates>, so the result is directly
usable as a Nav2 /goal_pose in the 'map' frame.

Topic flow:
  /odometry/global  →  wait for first fix before publishing goal
  /goal_pose        →  publishes PoseStamped once (Nav2 reacts immediately)
  /start_search     →  publishes True when rover arrives at target
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

try:
    import utm as _utm_lib
    _HAS_UTM = True
except ImportError:
    _HAS_UTM = False


class MissionInitializer(Node):
    # ── World datum from mars.world.sdf <spherical_coordinates> ────────────
    # These must match the SDF exactly; navsat_transform uses the same origin.
    DATUM_LAT = 39.8942
    DATUM_LON = 32.7845

    def __init__(self):
        super().__init__('mission_initializer')

        # ── FIX: declare target_lat / target_lon (matching full_autonomy.launch.py)
        # Previously the node declared target_x / target_y, so the GPS params
        # passed by the launch file were silently ignored and defaults (5.0, 0.0)
        # were used — sending the rover 17 m away from the astronaut.
        self.declare_parameter('target_lat', self.DATUM_LAT)
        self.declare_parameter('target_lon', self.DATUM_LON)
        self.declare_parameter('arrival_threshold', 2.0)

        target_lat = (self.get_parameter('target_lat')
                      .get_parameter_value().double_value)
        target_lon = (self.get_parameter('target_lon')
                      .get_parameter_value().double_value)

        # Convert GPS → map-frame Cartesian at startup (one-time, not per tick)
        self.target_x, self.target_y = self._gps_to_map(target_lat, target_lon)

        self.get_logger().info(
            f"Target GPS  : lat={target_lat:.7f}  lon={target_lon:.7f}")
        self.get_logger().info(
            f"Target (map): X={self.target_x:.3f} m  Y={self.target_y:.3f} m  "
            f"(datum lat={self.DATUM_LAT}, lon={self.DATUM_LON})")

        # ── Publishers / Subscribers ────────────────────────────────────────
        self.goal_pub        = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.start_search_pub = self.create_publisher(Bool, '/start_search', 10)

        # Use /odometry/global (map frame) so distance check is in map coords
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/global', self.odom_cb, qos_profile_sensor_data)

        self.current_odom_pos = None
        self.goal_sent        = False
        self.mission_started  = False

        self.get_logger().info("Mission Initializer waiting for /odometry/global …")
        self.timer = self.create_timer(1.0, self.mission_control_loop)

    # ── GPS → map frame conversion ──────────────────────────────────────────
    def _gps_to_map(self, lat: float, lon: float):
        """
        Convert a GPS lat/lon to map-frame (x, y) in metres.

        Preferred method: UTM (python-utm library) — same approach as gps_nav_node.py.
        Fallback: equirectangular approximation, accurate to <1 m within 10 km.

        In both cases the datum is subtracted so the map origin = world SDF origin.
        """
        if _HAS_UTM:
            datum_e, datum_n, _, _ = _utm_lib.from_latlon(self.DATUM_LAT, self.DATUM_LON)
            target_e, target_n, _, _ = _utm_lib.from_latlon(lat, lon)
            return target_e - datum_e, target_n - datum_n
        else:
            self.get_logger().warning(
                "python-utm not installed; using equirectangular approximation.")
            R = 6_378_137.0          # WGS84 equatorial radius in metres
            dlat = lat - self.DATUM_LAT
            dlon = lon - self.DATUM_LON
            x = math.radians(dlon) * math.cos(math.radians(self.DATUM_LAT)) * R
            y = math.radians(dlat) * R
            return x, y

    # ── Callbacks ───────────────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.current_odom_pos = msg.pose.pose.position

    def mission_control_loop(self):
        if self.current_odom_pos is None:
            return

        # ── Publish goal exactly once, as soon as odometry arrives ─────────
        if not self.goal_sent:
            goal = PoseStamped()
            goal.header.stamp    = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = self.target_x
            goal.pose.position.y = self.target_y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0   # Nav2 will compute its own heading

            self.goal_pub.publish(goal)
            self.goal_sent = True
            self.get_logger().info(
                f"Goal published → map X={self.target_x:.2f} m, Y={self.target_y:.2f} m")

        # ── Arrival check ───────────────────────────────────────────────────
        dist = math.sqrt(
            (self.current_odom_pos.x - self.target_x) ** 2 +
            (self.current_odom_pos.y - self.target_y) ** 2
        )
        threshold = (self.get_parameter('arrival_threshold')
                     .get_parameter_value().double_value)

        if dist < threshold and not self.mission_started:
            self.mission_started = True
            self.start_search_pub.publish(Bool(data=True))
            self.get_logger().warning(
                f"TARGET REACHED (dist={dist:.2f} m < {threshold:.2f} m). "
                "STARTING SEARCH PHASE.")


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
