#!/usr/bin/env python3
"""Teleport the `arc_repair_tool_flat` model near the rover using ros_gz_sim service.

Usage:
  python3 teleport_repair_tool.py [--x X] [--y Y] [--z Z] [--yaw YAW]

Defaults place the tool near the rover within typical camera/arm reach.
"""
import math
import sys
import argparse
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose

DEFAULT_NAME = 'Arc_emergency_kit'
DEFAULT_X = 0.8
DEFAULT_Y = 0.0
DEFAULT_Z = 0.35
DEFAULT_YAW = 0.0


def yaw_to_quaternion(yaw: float):
    # Return dict with x,y,z,w for yaw-only rotation
    s = math.sin(yaw / 2.0)
    c = math.cos(yaw / 2.0)
    return {'x': 0.0, 'y': 0.0, 'z': s, 'w': c}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default=DEFAULT_NAME)
    parser.add_argument('--x', type=float, default=DEFAULT_X)
    parser.add_argument('--y', type=float, default=DEFAULT_Y)
    parser.add_argument('--z', type=float, default=DEFAULT_Z)
    parser.add_argument('--yaw', type=float, default=DEFAULT_YAW)
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('teleport_repair_tool')
    cli = node.create_client(SetEntityPose, '/world/mars_world/set_pose')
    if not cli.wait_for_service(timeout_sec=15.0):
        node.get_logger().error('Service /world/mars_world/set_pose not available')
        return 1

    quat = yaw_to_quaternion(args.yaw)
    req = SetEntityPose.Request()
    req.name = args.name
    req.pose.position.x = float(args.x)
    req.pose.position.y = float(args.y)
    req.pose.position.z = float(args.z)
    req.pose.orientation.x = float(quat['x'])
    req.pose.orientation.y = float(quat['y'])
    req.pose.orientation.z = float(quat['z'])
    req.pose.orientation.w = float(quat['w'])

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.done() and not future.exception():
        node.get_logger().info(f'Teleported {args.name} to x={args.x} y={args.y} z={args.z} yaw={args.yaw}')
        rclpy.shutdown()
        return 0
    else:
        node.get_logger().error('Teleport call failed')
        rclpy.shutdown()
        return 2


if __name__ == '__main__':
    sys.exit(main())
