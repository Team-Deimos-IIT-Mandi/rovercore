#!/usr/bin/env python3
"""
Subscribes to /joint_states_raw (Gazebo bridge, Gazebo sim timestamps),
republishes to /joint_states with the current ROS sim clock timestamp.

Fixes RSP dropping wheel joint states due to Gazebo bridge clock-sync jitter.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateRestamper(Node):
    def __init__(self):
        super().__init__('joint_state_restamper')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states_raw', self._cb, 10)

    def _cb(self, msg: JointState):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointStateRestamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
