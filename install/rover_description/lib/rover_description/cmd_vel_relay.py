#!/usr/bin/env python3
"""
CMD_VEL Relay Node
Converts Twist messages from Nav2 (/cmd_vel) to TwistStamped for diff_drive_controller
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscribe to Nav2's cmd_vel (Twist)
        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Relay to diff_drive_controller (use_stamped_vel: false → unstamped topic)
        self.pub = self.create_publisher(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped',
            10
        )

        self.get_logger().info('CMD_VEL relay: /cmd_vel -> /diff_drive_controller/cmd_vel_unstamped')

    def cmd_vel_callback(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
