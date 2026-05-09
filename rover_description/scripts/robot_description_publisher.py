#!/usr/bin/env python3
"""
Publishes robot_description as a std_msgs/String topic with transient_local
(latched) QoS so that ros_gz_sim create -topic robot_description can receive it.

ROS2 robot_state_publisher only stores robot_description as a parameter, not
as a topic.  This node bridges that gap.

Usage (from launch file):
    Node(
        package='rover_description',
        executable='robot_description_publisher',
        parameters=[{'robot_description': xml_string}],
    )
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        self.declare_parameter('robot_description', '')
        urdf = self.get_parameter('robot_description').get_parameter_value().string_value

        qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(String, 'robot_description', qos)
        msg = String()
        msg.data = urdf
        # Publish once immediately; transient_local QoS means late subscribers
        # receive it automatically without re-publishing.
        self._pub.publish(msg)
        self.get_logger().info('robot_description published (%d chars)' % len(urdf))


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
