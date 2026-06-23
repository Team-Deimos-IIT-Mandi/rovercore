#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

SEQUENCE = [
    "BOOT",
    "SENSOR1", "DOME", "DELIVER_REC",
    "SENSOR2", "DOME", "DELIVER_SEN",
    "PANEL",
    "PIPE",
    "DOME",
    "DONE",
]

WAYPOINTS = {
    "SENSOR1": (6.0, 10.0),
    "SENSOR2": (6.0, 10.0),
    "PANEL": (16.0, 26.0),
    "PIPE": (10.0, 28.0),
    "DOME": (10.0, 0.0),
}

BOOT_WAIT = 2.0
DELIVER_WAIT = 5.0
TICK_HZ = 2.0


class MissionNav2Node(Node):

    def __init__(self):
        super().__init__("mission_state_nav2")
        self.idx = 0
        self.nav_in_progress = False
        self.wait_until = 0.0

        self.state_pub = self.create_publisher(String, "rover/mission_state", 10)

        cb_group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=cb_group)

        self.create_timer(1.0 / TICK_HZ, self._tick, callback_group=cb_group)

        self.get_logger().info(
            f"Mission Nav2 state machine started — {len(SEQUENCE)} states")

    def _build_goal(self, map_x: float, map_y: float) -> NavigateToPose.Goal:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = map_x
        pose.pose.position.y = map_y
        pose.pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _tick(self):
        if self.idx >= len(SEQUENCE):
            return

        state = SEQUENCE[self.idx]
        self.state_pub.publish(String(data=state))

        if state == "DONE":
            self.get_logger().info("Mission sequence complete.")
            return

        if state == "BOOT":
            if self.wait_until == 0.0:
                self.wait_until = self.get_clock().now().nanoseconds / 1e9 + BOOT_WAIT
                return
            if self.get_clock().now().nanoseconds / 1e9 >= self.wait_until:
                self.wait_until = 0.0
                self.idx += 1
            return

        if state in ("DELIVER_REC", "DELIVER_SEN"):
            if self.wait_until == 0.0:
                self.get_logger().info(f"{state} — waiting {DELIVER_WAIT}s")
                self.wait_until = self.get_clock().now().nanoseconds / 1e9 + DELIVER_WAIT
                return
            if self.get_clock().now().nanoseconds / 1e9 >= self.wait_until:
                self.get_logger().info(f"{state} — done")
                self.wait_until = 0.0
                self.idx += 1
            return

        if self.nav_in_progress:
            return

        wp = WAYPOINTS.get(state)
        if wp is None:
            self.get_logger().error(f"No waypoint for {state}")
            self.idx += 1
            return

        if not self._nav_client.server_is_ready():
            self.get_logger().warn(
                "NavigateToPose server not ready — waiting…",
                throttle_duration_sec=5.0,
            )
            return

        tx, ty = wp
        self.get_logger().info(
            f"{state} — sending Nav2 goal to ({tx:.1f}, {ty:.1f})")
        self.nav_in_progress = True
        goal = self._build_goal(tx, ty)
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f"Nav2 goal rejected for {SEQUENCE[self.idx]}. Skipping.")
            self.nav_in_progress = False
            self.idx += 1
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self.nav_in_progress = False
        result = future.result()
        if result.status == 4:
            self.get_logger().info(
                f"{SEQUENCE[self.idx]} — Nav2 reached destination")
            self.idx += 1
        else:
            self.get_logger().error(
                f"Nav2 failed for {SEQUENCE[self.idx]} "
                f"(status={result.status}). Skipping.")
            self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = MissionNav2Node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
