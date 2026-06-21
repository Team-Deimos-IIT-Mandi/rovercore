#!/usr/bin/env python3
"""
Gate Detection Node — Phase 1.2
Detects a gate (two vertical posts) using 2D LiDAR data and publishes the
midpoint as a navigation goal.

Algorithm:
  1. Convert LaserScan ranges to Cartesian (x, y) in base_link frame.
  2. Euclidean clustering with configurable distance threshold.
  3. Filter clusters to "pole-like" candidates: small spatial spread (< 0.15 m diameter).
  4. If exactly 2 poles are found within max_detection_range and their separation
     is between min_gate_width and max_gate_width → gate detected.
  5. Compute midpoint in base_link, look up TF to map frame, publish as PoseStamped.

Published topics:
  /gate/detected  (std_msgs/Bool)            — true when a gate is in view
  /gate/goal      (geometry_msgs/PoseStamped) — map-frame goal at gate midpoint

Subscribed topics:
  /scan  (sensor_msgs/LaserScan)

Parameters:
  min_gate_width        (float) — minimum post separation in metres (default 0.5)
  max_gate_width        (float) — maximum post separation in metres (default 4.0)
  max_detection_range   (float) — ignore clusters beyond this range   (default 5.0)
  cluster_eps           (float) — Euclidean clustering threshold       (default 0.3)
  pole_max_diameter     (float) — max cluster spread to count as pole  (default 0.15)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool


class GateDetectionNode(Node):
    """Detects a two-post gate from 2-D LiDAR and publishes the midpoint goal."""

    def __init__(self):
        super().__init__("gate_detection_node")

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter("min_gate_width", 0.5)
        self.declare_parameter("max_gate_width", 4.0)
        self.declare_parameter("max_detection_range", 5.0)
        self.declare_parameter("cluster_eps", 0.3)
        self.declare_parameter("pole_max_diameter", 0.15)

        self.min_gate_width = (
            self.get_parameter("min_gate_width").get_parameter_value().double_value
        )
        self.max_gate_width = (
            self.get_parameter("max_gate_width").get_parameter_value().double_value
        )
        self.max_detection_range = (
            self.get_parameter("max_detection_range").get_parameter_value().double_value
        )
        self.cluster_eps = (
            self.get_parameter("cluster_eps").get_parameter_value().double_value
        )
        self.pole_max_diameter = (
            self.get_parameter("pole_max_diameter").get_parameter_value().double_value
        )

        # ------------------------------------------------------------------ #
        # TF2
        # ------------------------------------------------------------------ #
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self.detected_pub = self.create_publisher(Bool, "/gate/detected", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/gate/goal", 10)

        # ------------------------------------------------------------------ #
        # Subscriber
        # ------------------------------------------------------------------ #
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)

        self.get_logger().info(
            f"Gate Detection Node started. Gate width: "
            f"[{self.min_gate_width}, {self.max_gate_width}] m, "
            f"max range: {self.max_detection_range} m."
        )

    # ---------------------------------------------------------------------- #
    # Scan callback
    # ---------------------------------------------------------------------- #

    def _scan_cb(self, msg: LaserScan):
        points = self._scan_to_cartesian(msg)
        clusters = self._euclidean_cluster(points)
        pole_centroids = self._filter_pole_clusters(clusters)
        gate_detected, midpoint = self._detect_gate(pole_centroids)

        detected_msg = Bool()
        detected_msg.data = gate_detected
        self.detected_pub.publish(detected_msg)

        if gate_detected and midpoint is not None:
            goal = self._midpoint_to_map_goal(midpoint, msg.header.stamp)
            if goal is not None:
                self.goal_pub.publish(goal)
                self.get_logger().info(
                    f"Gate detected! Midpoint in base_link: "
                    f"({midpoint[0]:.2f}, {midpoint[1]:.2f}). Goal published to /gate/goal."
                )

    # ---------------------------------------------------------------------- #
    # Step 1 — LaserScan → Cartesian points
    # ---------------------------------------------------------------------- #

    def _scan_to_cartesian(self, msg: LaserScan):
        """Return list of (x, y) in base_link frame for valid ranges."""
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < min(msg.range_max, self.max_detection_range):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
            angle += msg.angle_increment
        return points

    # ---------------------------------------------------------------------- #
    # Step 2 — Simple Euclidean clustering
    # ---------------------------------------------------------------------- #

    def _euclidean_cluster(self, points):
        """
        Greedy single-linkage clustering with distance threshold cluster_eps.
        Returns a list of clusters, each cluster being a list of (x, y) tuples.
        """
        clusters = []
        assigned = [False] * len(points)

        for i, pt in enumerate(points):
            if assigned[i]:
                continue
            cluster = [pt]
            assigned[i] = True
            # Grow the cluster by finding all unassigned points within eps
            queue = [i]
            while queue:
                idx = queue.pop()
                xi, yi = points[idx]
                for j, (xj, yj) in enumerate(points):
                    if assigned[j]:
                        continue
                    if math.hypot(xi - xj, yi - yj) <= self.cluster_eps:
                        assigned[j] = True
                        cluster.append(points[j])
                        queue.append(j)
            clusters.append(cluster)

        return clusters

    # ---------------------------------------------------------------------- #
    # Step 3 — Filter to pole-like clusters
    # ---------------------------------------------------------------------- #

    def _cluster_spread(self, cluster):
        """Return the diameter (max pairwise distance) of a cluster."""
        if len(cluster) < 2:
            return 0.0
        max_d = 0.0
        for i in range(len(cluster)):
            for j in range(i + 1, len(cluster)):
                d = math.hypot(
                    cluster[i][0] - cluster[j][0],
                    cluster[i][1] - cluster[j][1],
                )
                if d > max_d:
                    max_d = d
        return max_d

    def _centroid(self, cluster):
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        return (cx, cy)

    def _filter_pole_clusters(self, clusters):
        """Return centroids of clusters that look like vertical poles."""
        pole_centroids = []
        for cluster in clusters:
            spread = self._cluster_spread(cluster)
            if spread < self.pole_max_diameter:
                c = self._centroid(cluster)
                dist = math.hypot(c[0], c[1])
                if dist <= self.max_detection_range:
                    pole_centroids.append(c)
        return pole_centroids

    # ---------------------------------------------------------------------- #
    # Step 4 — Gate detection from pole pairs
    # ---------------------------------------------------------------------- #

    def _detect_gate(self, pole_centroids):
        """
        Find exactly 2 poles whose separation falls within gate width bounds.
        Returns (detected: bool, midpoint: (x, y) or None).
        """
        if len(pole_centroids) < 2:
            return False, None

        # Check all pairs; use the first valid one
        for i in range(len(pole_centroids)):
            for j in range(i + 1, len(pole_centroids)):
                p1 = pole_centroids[i]
                p2 = pole_centroids[j]
                separation = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                if self.min_gate_width <= separation <= self.max_gate_width:
                    midpoint = (
                        (p1[0] + p2[0]) / 2.0,
                        (p1[1] + p2[1]) / 2.0,
                    )
                    return True, midpoint

        return False, None

    # ---------------------------------------------------------------------- #
    # Step 5 — Transform midpoint to map frame and build PoseStamped goal
    # ---------------------------------------------------------------------- #

    def _midpoint_to_map_goal(self, midpoint, stamp):
        """
        Look up the base_link → map transform and convert the midpoint.
        Returns a PoseStamped in the map frame, or None on TF failure.
        """
        pose_in_base = PoseStamped()
        pose_in_base.header.stamp = stamp
        pose_in_base.header.frame_id = "base_link"
        pose_in_base.pose.position.x = midpoint[0]
        pose_in_base.pose.position.y = midpoint[1]
        pose_in_base.pose.position.z = 0.0
        pose_in_base.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            pose_in_map = do_transform_pose(pose_in_base, transform)
            # Stamp with current time for freshness
            pose_in_map.header.stamp = self.get_clock().now().to_msg()
            return pose_in_map
        except TransformException as e:
            self.get_logger().warn(
                f"TF lookup base_link→map failed: {e}",
                throttle_duration_sec=2.0,
            )
            return None


def main(args=None):
    rclpy.init(args=args)
    node = GateDetectionNode()
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
