#!/usr/bin/env python3
"""
Launch RViz with configuration
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch RViz2 with the robot configuration"""
    
    # Create the path to the rviz file
    rviz_config_file = os.path.join(
        get_package_share_directory('my_robotic_arm'),
        'rviz',
        'view_robot.rviz'  
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rviz_node,
    ])
