#!/usr/bin/env python3
"""Joystick teleop — sensor_msgs/Joy → geometry_msgs/Twist on /cmd_vel.

Default axis/button mapping (PS4 / Xbox-style, adjust via parameters):
  Left stick vertical  (axis 1) → linear.x
  Left stick horiz     (axis 0) → angular.z
  R1 / RB              (button 5) → deadman (must hold to move)

Run alongside the joy node:
  ros2 run joy joy_node
  ros2 run rover_description joy_teleop.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        self.declare_parameter('axis_linear',    1)     # left stick up/down
        self.declare_parameter('axis_angular',   0)     # left stick left/right
        self.declare_parameter('deadman_button', 5)     # R1 / RB
        self.declare_parameter('scale_linear',   1.5)   # m/s at full deflection
        self.declare_parameter('scale_angular',  1.5)   # rad/s at full deflection

        self.axis_lin  = self.get_parameter('axis_linear').value
        self.axis_ang  = self.get_parameter('axis_angular').value
        self.deadman   = self.get_parameter('deadman_button').value
        self.scale_lin = self.get_parameter('scale_linear').value
        self.scale_ang = self.get_parameter('scale_angular').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Joy, '/joy', self._cb, 10)

        self.get_logger().info(
            f'Joy teleop ready — hold RB (button {self.deadman}) to drive')

    def _cb(self, msg: Joy):
        twist = Twist()

        if len(msg.buttons) > self.deadman and msg.buttons[self.deadman]:
            if len(msg.axes) > max(self.axis_lin, self.axis_ang):
                twist.linear.x  = self.scale_lin * msg.axes[self.axis_lin]
                twist.angular.z = self.scale_ang * msg.axes[self.axis_ang]

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
