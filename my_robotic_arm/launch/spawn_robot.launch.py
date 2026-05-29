#!/usr/bin/env python3
"""
Spawn robot in Gazebo
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Spawn the robot entity in Gazebo"""
    
    # Node: Spawn Entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robotic_arm',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.05', 
            '-R', '0.0',  # Roll (X-axis rotation) 
            '-P', '0.0',  # Pitch (Y-axis rotation) 
            '-Y', '0.0'   # Yaw (Z-axis rotation)
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_entity,
    ])
