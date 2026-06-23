#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

SEQUENCE = [
    "BOOTING",
    "NAVIGATE_TO_SENSOR_ZONE_1", "DOME_RETURN", "DELIVER_RECORDER",
    "NAVIGATE_TO_SENSOR_ZONE_2", "DOME_RETURN","DELIVER_SENSORS",
    "NAVIGATE_TO_PANEL_ZONE",
    "NAVIGATE_TO_PIPE_ZONE",
    # "NAVIGATE_TO_MIRROR_ZONE",
    # "NAVIGATE_TO_ANTENNA_ZONE",
    # "NAVIGATE_TO_LAVA_ZONE",
    "DOME_RETURN",
    "SEQUENCE_COMPLETE",
]

WAYPOINTS = {
    "NAVIGATE_TO_SENSOR_ZONE_1": (6.0, 10.0),
    "NAVIGATE_TO_SENSOR_ZONE_2": (6.0, 10.0),
    "NAVIGATE_TO_PANEL_ZONE": (16.0, 26.0),
    "NAVIGATE_TO_PIPE_ZONE": (10.0, 28.0),
    # "NAVIGATE_TO_MIRROR_ZONE": (15.0, -10.0),
    # "NAVIGATE_TO_ANTENNA_ZONE": (5.0, -10.0),
    # "NAVIGATE_TO_LAVA_ZONE": (0.0, -15.0),
    "DOME_RETURN": (10.0, 0.0),
}

TOLERANCE = 3.0
MAX_LIN = 1.5
MIN_LIN = 0.2
MAX_ANG = 0.8
ANG_KP = 1.5
SLOW_DOWN = 3.0
HEADING_THRESH = 0.35
BOOT_WAIT = 2.0
DELIVER_WAIT = 5.0
HZ = 10.0


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a):
    while a > math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def compute_twist(tx, ty, cx, cy, yaw):
    dx, dy = tx - cx, ty - cy
    dist = math.sqrt(dx * dx + dy * dy)
    err = normalize_angle(math.atan2(dy, dx) - yaw)
    t = Twist()
    t.angular.z = max(-MAX_ANG, min(MAX_ANG, ANG_KP * err))
    if abs(err) < HEADING_THRESH:
        t.linear.x = max(MIN_LIN, MAX_LIN * min(dist / SLOW_DOWN, 1.0))
    return t


class MissionNode(Node):

    def __init__(self):
        super().__init__("mission_state")
        self.idx = 0
        self.cx = self.cy = 0.0
        self.yaw = -1.5708
        self.wait_until = 0.0

        self.state_pub = self.create_publisher(String, "rover/mission_state", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/gps/odom", self._odom, 10)
        self.create_subscription(Imu, "/imu/filtered", self._imu, 10)
        self.create_timer(1.0 / HZ, self._tick)
        self.get_logger().info(f"Started — {len(SEQUENCE)} states")

    def _odom(self, msg):
        self.cx = msg.pose.pose.position.x
        self.cy = msg.pose.pose.position.y

    def _imu(self, msg):
        self.yaw = quaternion_to_yaw(msg.orientation)

    def _tick(self):
        if self.idx >= len(SEQUENCE):
            return
        state = SEQUENCE[self.idx]
        self.state_pub.publish(String(data=state))

        if state == "SEQUENCE_COMPLETE":
            self.cmd_pub.publish(Twist())
            return

        if state == "BOOTING":
            self.cmd_pub.publish(Twist())
            if self.wait_until == 0.0:
                self.wait_until = self.get_clock().now().nanoseconds / 1e9 + BOOT_WAIT
            elif self.get_clock().now().nanoseconds / 1e9 >= self.wait_until:
                self.wait_until = 0.0
                self.idx += 1
            return

        if state.startswith("DELIVER_"):
            self.cmd_pub.publish(Twist())
            if self.wait_until == 0.0:
                self.get_logger().info(f"{state} — waiting {DELIVER_WAIT}s")
                self.wait_until = self.get_clock().now().nanoseconds / 1e9 + DELIVER_WAIT
            elif self.get_clock().now().nanoseconds / 1e9 >= self.wait_until:
                self.get_logger().info(f"{state} — done")
                self.wait_until = 0.0
                self.idx += 1
            return

        wp = WAYPOINTS.get(state)
        if wp is None:
            self.get_logger().error(f"No waypoint for {state}")
            self.idx += 1
            return

        tx, ty = wp
        dist = distance(self.cx, self.cy, tx, ty)
        raw_heading_err = normalize_angle(math.atan2(ty - self.cy, tx - self.cx) - self.yaw)
        if dist <= TOLERANCE:
            self.get_logger().info(f"{state} — arrived ({tx:.1f}, {ty:.1f}) — 0.00m left")
            self.cmd_pub.publish(Twist())
            self.idx += 1
            return

        self.cmd_pub.publish(compute_twist(tx, ty, self.cx, self.cy, self.yaw))
        self.get_logger().info(
            f"{state} — ({tx:.1f}, {ty:.1f}) — {dist:.2f}m left — "
            f"heading error {raw_heading_err:.2f}rad",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
