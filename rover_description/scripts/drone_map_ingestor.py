#!/usr/bin/env python3
"""
Drone Map Ingestor — receives aerial occupancy map tiles from the companion drone
and republishes them as a Nav2-compatible costmap layer.

Subscribes to raw aerial map (from drone sim or companion RPi Zero 2W),
optionally downsamples/upsamples to target resolution, and republishes on
/drone/map_layer for consumption by the Nav2 global costmap custom layer.

Published topics:
  /drone/map_layer  (nav_msgs/OccupancyGrid) — processed map ready for Nav2 costmap

Subscribed topics:
  /drone/map_sim    (nav_msgs/OccupancyGrid) — raw aerial map from drone

Parameters:
  target_resolution  (float) — costmap cell size in meters (default 1.0)
  map_frame          (str)   — frame id for published map (default "map")
  passthrough        (bool)  — if True, publish raw map without processing (default True)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class DroneMapIngestor(Node):
    def __init__(self):
        super().__init__('drone_map_ingestor')

        self.declare_parameter('target_resolution', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('passthrough', True)

        self._target_res = self.get_parameter('target_resolution').value
        self._map_frame = self.get_parameter('map_frame').value
        self._passthrough = self.get_parameter('passthrough').value

        self._map_pub = self.create_publisher(
            OccupancyGrid, '/drone/map_layer', 10)

        self.create_subscription(
            OccupancyGrid, '/drone/map_sim', self._map_cb, 10)

        self.get_logger().info(
            f'Drone map ingestor started: resolution={self._target_res}m, '
            f'frame={self._map_frame}, passthrough={self._passthrough}')

    def _map_cb(self, msg: OccupancyGrid):
        if self._passthrough:
            # Stamp with current time and force correct frame, pass through
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._map_frame
            self._map_pub.publish(msg)
            return

        # Resample to target resolution
        out = self._resample(msg)
        self._map_pub.publish(out)

    def _resample(self, src: OccupancyGrid) -> OccupancyGrid:
        src_res = src.info.resolution
        scale = src_res / self._target_res

        if abs(scale - 1.0) < 0.01:
            src.header.stamp = self.get_clock().now().to_msg()
            src.header.frame_id = self._map_frame
            return src

        src_w = src.info.width
        src_h = src.info.height
        dst_w = max(1, int(src_w * scale))
        dst_h = max(1, int(src_h * scale))

        dst_data = []
        for row in range(dst_h):
            for col in range(dst_w):
                src_col = min(int(col / scale), src_w - 1)
                src_row = min(int(row / scale), src_h - 1)
                dst_data.append(src.data[src_row * src_w + src_col])

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._map_frame
        out.info.resolution = self._target_res
        out.info.width = dst_w
        out.info.height = dst_h
        out.info.origin = src.info.origin
        out.data = dst_data
        self.get_logger().debug(
            f'Map resampled {src_w}×{src_h}@{src_res:.2f}m → {dst_w}×{dst_h}@{self._target_res:.2f}m')
        return out


def main(args=None):
    rclpy.init(args=args)
    node = DroneMapIngestor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
