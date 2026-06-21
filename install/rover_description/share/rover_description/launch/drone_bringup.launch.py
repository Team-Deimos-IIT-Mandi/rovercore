#!/usr/bin/env python3
"""
Drone Bringup — launches MAVROS bridge + drone_aruco_relay + drone_map_ingestor.

Args:
  fcu_url      (str)  — MAVLink UDP URL (default: udp://:14540@localhost:14557)
  use_sim      (bool) — if True, skip MAVROS (no FC connected), relay/ingestor only
  use_sim_time (bool) — ROS sim time
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('rover_description')

    fcu_url     = LaunchConfiguration('fcu_url')
    use_sim     = LaunchConfiguration('use_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_fcu  = DeclareLaunchArgument(
        'fcu_url', default_value='udp://:14540@localhost:14557',
        description='MAVLink FCU URL')
    declare_sim  = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='Skip MAVROS (no FC), run relay+ingestor only')
    declare_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    mavros_params = PathJoinSubstitution([pkg, 'config', 'mavros_params.yaml'])

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[
            mavros_params,
            {'fcu_url': fcu_url, 'use_sim_time': use_sim_time},
        ],
        output='screen',
        condition=UnlessCondition(use_sim),
    )

    drone_aruco_relay = Node(
        package='rover_description',
        executable='drone_aruco_relay.py',
        name='drone_aruco_relay',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    drone_map_ingestor = Node(
        package='rover_description',
        executable='drone_map_ingestor.py',
        name='drone_map_ingestor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_fcu,
        declare_sim,
        declare_time,
        mavros_node,
        drone_aruco_relay,
        drone_map_ingestor,
    ])
