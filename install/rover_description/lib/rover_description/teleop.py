#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Mars Rover!
---------------------------
Moving around:
   w
a  s  d

w/s : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, k : force stop
anything else : stop

CTRL-C to quit
"""

class StampedTeleop(Node):
    def __init__(self):
        super().__init__('stamped_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.linear_vel = 60.0#1.5
        self.angular_vel = 1.0#0.5
        
        self.get_logger().info(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                ts = Twist()

                if key == 'w':
                    ts.linear.x = self.linear_vel
                elif key == 's':
                    ts.linear.x = -self.linear_vel
                elif key == 'a':
                    ts.angular.z = self.angular_vel
                elif key == 'd':
                    ts.angular.z = -self.angular_vel
                elif key in [' ', 'k']:
                    ts.linear.x = 0.0
                    ts.angular.z = 0.0
                elif key == '\x03': #CTRL -C
                    break

                if key != '':
                    self.publisher_.publish(ts)

        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StampedTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()