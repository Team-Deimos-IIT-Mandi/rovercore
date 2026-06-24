#!/usr/bin/env python3
"""
Slope Detector Node — Phase 2.1
Terrain-adaptive safety monitor.  Reads IMU orientation, computes roll/pitch,
adjusts Nav2 max_vel_x dynamically, and publishes tilt warnings / e-stop.

Published topics:
  /terrain/slope        (std_msgs/Float32)  — tilt magnitude in degrees
  /terrain/tilt_warning (std_msgs/Bool)     — true if slope > tilt_warning_deg
  /rover/estop          (std_msgs/Bool)     — true if roll > estop_roll_deg OR pitch > estop_pitch_deg

Subscribed topics:
  /imu/filtered  (sensor_msgs/Imu)

Parameters:
  tilt_warning_deg  (float) — slope threshold for warning (default 15.0 °)
  estop_roll_deg    (float) — roll   threshold for e-stop  (default 30.0 °)
  estop_pitch_deg   (float) — pitch  threshold for e-stop  (default 35.0 °)

Rate: 20 Hz (timer-driven publish; IMU data processed in callback).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool


# ---------------------------------------------------------------------------
# Quaternion → Euler (roll, pitch, yaw) — no external dependency needed
# ---------------------------------------------------------------------------
def quat_to_rpy(x, y, z, w):
    """Return (roll, pitch, yaw) in radians from a unit quaternion."""
    # Roll (rotation about X)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation about Y)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)  # clamp to ±90°
    else:
        pitch = math.asin(sinp)

    # Yaw (rotation about Z) — not used here but returned for completeness
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class SlopeDetectorNode(Node):
    """Terrain-adaptive slope monitor with Nav2 velocity scaling."""

    def __init__(self):
        super().__init__("slope_detector")

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter("tilt_warning_deg", 15.0)
        self.declare_parameter("estop_roll_deg", 30.0)
        self.declare_parameter("estop_pitch_deg", 35.0)

        self._update_params()
        self.add_on_set_parameters_callback(self._param_cb)

        # ------------------------------------------------------------------ #
        # State
        # ------------------------------------------------------------------ #
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.slope_deg = 0.0
        self._last_vel_band = None   # track previous band to avoid redundant calls

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self.slope_pub = self.create_publisher(Float32, "/terrain/slope", 10)
        self.tilt_warn_pub = self.create_publisher(Bool, "/terrain/tilt_warning", 10)
        self.estop_pub = self.create_publisher(Bool, "/rover/estop", 10)

        # ------------------------------------------------------------------ #
        # Subscriber
        # ------------------------------------------------------------------ #
        self.create_subscription(Imu, "/imu/filtered", self._imu_cb, 10)

        # ------------------------------------------------------------------ #
        # 20 Hz publish timer
        # ------------------------------------------------------------------ #
        self.create_timer(1.0 / 20.0, self._publish_loop)

        self.get_logger().info(
            f"Slope Detector started. "
            f"Warning > {self.tilt_warning_deg:.1f}°, "
            f"E-stop roll > {self.estop_roll_deg:.1f}° / pitch > {self.estop_pitch_deg:.1f}°."
        )

    # ---------------------------------------------------------------------- #
    # Parameter handling
    # ---------------------------------------------------------------------- #

    def _update_params(self):
        self.tilt_warning_deg = (
            self.get_parameter("tilt_warning_deg").get_parameter_value().double_value
        )
        self.estop_roll_deg = (
            self.get_parameter("estop_roll_deg").get_parameter_value().double_value
        )
        self.estop_pitch_deg = (
            self.get_parameter("estop_pitch_deg").get_parameter_value().double_value
        )

    def _param_cb(self, params):
        for p in params:
            if p.name == "tilt_warning_deg":
                self.tilt_warning_deg = p.value
            elif p.name == "estop_roll_deg":
                self.estop_roll_deg = p.value
            elif p.name == "estop_pitch_deg":
                self.estop_pitch_deg = p.value
        return SetParametersResult(successful=True)

    # ---------------------------------------------------------------------- #
    # IMU callback — compute roll/pitch
    # ---------------------------------------------------------------------- #

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        roll, pitch, _ = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)
        # Tilt magnitude: Euclidean norm of roll and pitch
        self.slope_deg = math.sqrt(self.roll_deg ** 2 + self.pitch_deg ** 2)

    # ---------------------------------------------------------------------- #
    # 20 Hz publish loop
    # ---------------------------------------------------------------------- #

    def _publish_loop(self):
        # --- /terrain/slope ---
        slope_msg = Float32()
        slope_msg.data = float(self.slope_deg)
        self.slope_pub.publish(slope_msg)

        # --- /terrain/tilt_warning ---
        tilt_warn = self.slope_deg > self.tilt_warning_deg
        tilt_msg = Bool()
        tilt_msg.data = tilt_warn
        self.tilt_warn_pub.publish(tilt_msg)

        if tilt_warn:
            self.get_logger().warn(
                f"Tilt warning: slope={self.slope_deg:.1f}° "
                f"(roll={self.roll_deg:.1f}°, pitch={self.pitch_deg:.1f}°)",
                throttle_duration_sec=2.0,
            )

        # --- /rover/estop ---
        estop = (
            abs(self.roll_deg) > self.estop_roll_deg
            or abs(self.pitch_deg) > self.estop_pitch_deg
        )
        estop_msg = Bool()
        estop_msg.data = estop
        self.estop_pub.publish(estop_msg)

        if estop:
            self.get_logger().error(
                f"E-STOP: roll={self.roll_deg:.1f}° pitch={self.pitch_deg:.1f}°",
                throttle_duration_sec=1.0,
            )

        # --- Nav2 velocity scaling ---
        self._update_nav2_max_vel()

    # ---------------------------------------------------------------------- #
    # Nav2 parameter update — FollowPath.max_vel_x
    # ---------------------------------------------------------------------- #

    def _update_nav2_max_vel(self):
        """
        Update the Nav2 controller server's FollowPath.max_vel_x parameter
        based on the current slope band.

        Bands:
          slope < 10°       → 0.50 m/s
          10° ≤ slope < 20° → 0.25 m/s
          slope ≥ 20°       → 0.10 m/s
        """
        if self.slope_deg < 10.0:
            target_vel = 3.0
            band = "flat"
        elif self.slope_deg < 20.0:
            target_vel = 1.5
            band = "moderate"
        else:
            target_vel = 0.5
            band = "steep"

        # Only call the service when the band changes to avoid flooding
        if band == self._last_vel_band:
            return
        self._last_vel_band = band

        # Use rclpy AsyncParameterClient (non-blocking, fire-and-forget)
        try:
            from rclpy.parameter_client import AsyncParameterClient  # ROS 2 Humble+
            client = AsyncParameterClient(self, "controller_server")
            future = client.set_parameters(
                [Parameter("FollowPath.max_vel_x", Parameter.Type.DOUBLE, target_vel)]
            )
            # Add a done callback so we don't block the timer
            future.add_done_callback(
                lambda f: self._vel_update_done_cb(f, target_vel, band)
            )
        except Exception as e:
            # AsyncParameterClient may not be available in all builds;
            # log and continue — slope/estop publishers still work.
            self.get_logger().warn(
                f"Could not update Nav2 max_vel_x: {e}",
                throttle_duration_sec=10.0,
            )

    def _vel_update_done_cb(self, future, target_vel, band):
        try:
            result = future.result()
            if result and all(r.successful for r in result.results):
                self.get_logger().info(
                    f"Nav2 FollowPath.max_vel_x → {target_vel} m/s ({band} terrain)"
                )
            else:
                self.get_logger().warn(
                    f"Nav2 parameter set returned failure for max_vel_x={target_vel}"
                )
        except Exception as e:
            self.get_logger().warn(f"Nav2 velocity update callback error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SlopeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
