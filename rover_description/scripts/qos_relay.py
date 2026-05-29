#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSRelay(Node):
    def __init__(self):
        super().__init__('qos_relay')
        
        # Subscribe with BEST_EFFORT
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.sub = self.create_subscription(
            PointCloud2,
            '/rgbd_camera/points',
            self.listener_callback,
            best_effort_qos
        )
        
        # Publish with RELIABLE
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub = self.create_publisher(PointCloud2, '/rgbd_camera/points_reliable', reliable_qos)
        self.get_logger().info("QoS Relay started: /rgbd_camera/points (Best Effort) -> /rgbd_camera/points_reliable (Reliable)")

    def listener_callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
