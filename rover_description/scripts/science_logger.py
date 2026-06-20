#!/usr/bin/env python3
"""
science_logger.py — ROS 2 node for science task data logging.

Service:   /science/sample  (std_srvs/Trigger)
Subscribes:
  /science/soil_readings  (std_msgs/Float32MultiArray)  [pH, moisture%, temperature_C]
  /gps/fix                (sensor_msgs/NavSatFix)
  /odometry/local         (nav_msgs/Odometry)

On service call, appends one CSV row to ~/science_results.csv (or output_file param).

CSV columns:
  timestamp, lat, lon, odom_x, odom_y, pH, moisture_pct, temp_C, notes
"""

import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

CSV_HEADER = ["timestamp", "lat", "lon", "odom_x", "odom_y", "pH", "moisture_pct", "temp_C", "notes"]

# Placeholder values used when soil sensor data has not yet been received
SOIL_PLACEHOLDER = [6.5, 15.0, 20.0]


class ScienceLogger(Node):
    def __init__(self):
        super().__init__("science_logger")

        # Parameters
        default_output = os.path.expanduser("~/science_results.csv")
        self.declare_parameter("output_file", default_output)
        self._output_file: str = os.path.expanduser(
            self.get_parameter("output_file").get_parameter_value().string_value
        )

        # Internal state — latest received values
        self._soil: list[float] | None = None   # [pH, moisture%, temp_C]
        self._gps: NavSatFix | None = None
        self._odom: Odometry | None = None

        # QoS — best-effort for sensor topics
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Subscriptions
        self.create_subscription(
            Float32MultiArray,
            "/science/soil_readings",
            self._soil_cb,
            sensor_qos,
        )
        self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self._gps_cb,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            "/odometry/local",
            self._odom_cb,
            10,
        )

        # Service
        self._srv = self.create_service(Trigger, "/science/sample", self._sample_cb)

        # Ensure CSV file has a header row if it doesn't exist yet
        self._ensure_csv_header()

        self.get_logger().info(
            f"science_logger ready — writing to {self._output_file}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _soil_cb(self, msg: Float32MultiArray) -> None:
        self._soil = list(msg.data)

    def _gps_cb(self, msg: NavSatFix) -> None:
        self._gps = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _sample_cb(self, request, response: Trigger.Response) -> Trigger.Response:
        """Service handler — record current sensor snapshot to CSV."""
        timestamp = datetime.utcnow().isoformat(timespec="milliseconds") + "Z"

        # GPS
        if self._gps is not None:
            lat = self._gps.latitude
            lon = self._gps.longitude
        else:
            lat = float("nan")
            lon = float("nan")
            self.get_logger().warning("No GPS fix received yet — recording NaN for lat/lon")

        # Odometry
        if self._odom is not None:
            odom_x = self._odom.pose.pose.position.x
            odom_y = self._odom.pose.pose.position.y
        else:
            odom_x = float("nan")
            odom_y = float("nan")
            self.get_logger().warning("No odometry received yet — recording NaN for odom_x/y")

        # Soil readings
        if self._soil is not None:
            soil = self._soil
            notes = ""
        else:
            soil = SOIL_PLACEHOLDER
            notes = "soil_placeholder"
            self.get_logger().warning(
                "/science/soil_readings not received — using placeholder values "
                f"{SOIL_PLACEHOLDER}"
            )

        ph = soil[0] if len(soil) > 0 else float("nan")
        moisture = soil[1] if len(soil) > 1 else float("nan")
        temp = soil[2] if len(soil) > 2 else float("nan")

        row = {
            "timestamp": timestamp,
            "lat": lat,
            "lon": lon,
            "odom_x": odom_x,
            "odom_y": odom_y,
            "pH": ph,
            "moisture_pct": moisture,
            "temp_C": temp,
            "notes": notes,
        }

        try:
            with open(self._output_file, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
                writer.writerow(row)
        except OSError as exc:
            self.get_logger().error(f"Failed to write CSV: {exc}")
            response.success = False
            response.message = f"CSV write error: {exc}"
            return response

        msg_str = (
            f"Recorded — pH={ph:.2f} moisture={moisture:.1f}% "
            f"temp={temp:.1f}°C lat={lat:.6f} lon={lon:.6f} "
            f"odom=({odom_x:.3f},{odom_y:.3f})"
        )
        self.get_logger().info(msg_str)
        response.success = True
        response.message = msg_str
        return response

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _ensure_csv_header(self) -> None:
        """Write the CSV header if the file does not yet exist."""
        output_dir = os.path.dirname(self._output_file)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        if not os.path.exists(self._output_file):
            with open(self._output_file, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
                writer.writeheader()
            self.get_logger().info(f"Created new CSV file: {self._output_file}")


def main(args=None):
    rclpy.init(args=args)
    node = ScienceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
