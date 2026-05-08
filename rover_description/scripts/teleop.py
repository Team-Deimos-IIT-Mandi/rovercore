#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
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
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.linear_vel = 1.5
        self.angular_vel = 0.5
        
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
                ts = TwistStamped()
                ts.header.stamp = self.get_clock().now().to_msg()
                ts.header.frame_id = 'base_link'

                if key == 'w':
                    ts.twist.linear.x = self.linear_vel
                elif key == 's':
                    ts.twist.linear.x = -self.linear_vel
                elif key == 'a':
                    ts.twist.angular.z = self.angular_vel
                elif key == 'd':
                    ts.twist.angular.z = -self.angular_vel
                elif key in [' ', 'k']:
                    ts.twist.linear.x = 0.0
                    ts.twist.angular.z = 0.0
                elif key == '\x03': #CTRL -C 
                    break
                
                if key != '':
                    self.publisher_.publish(ts)

        except Exception as e:
            print(e)
        finally:
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StampedTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()