#!/usr/bin/env python3
"""
Subscribes to /joint_states_raw (Gazebo bridge, Gazebo sim timestamps),
republishes to /joint_states with the current ROS sim clock timestamp.

Fixes RSP dropping wheel joint states due to Gazebo bridge clock-sync jitter.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class JointStateRestamper(Node):
    def __init__(self):
        super().__init__('joint_state_restamper')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(
            JointState, '/joint_states_raw', self._cb, 10)
            
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

    def _cb(self, msg: JointState):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = JointStateRestamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
