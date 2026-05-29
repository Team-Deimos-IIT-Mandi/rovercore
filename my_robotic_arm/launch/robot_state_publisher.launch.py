#!/usr/bin/env python3
"""
Launch robot state publisher - publishes robot transforms
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Robot State Publisher"""
    
    pkg_name = 'my_robotic_arm' 
    
    # Finding the path for URDF file
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'Arm_Urdf.urdf')
    config_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

    # Read the URDF file content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Replace $(find my_robotic_arm) with absolute path
    # This handles both mesh paths: file://$(find ...) and controller config: ${find ...}
    robot_desc = robot_desc.replace('$(find my_robotic_arm)', pkg_share)
    robot_desc = robot_desc.replace('${find my_robotic_arm}', pkg_share)

    # Node: Robot State Publisher (Publishes TF for RViz)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        
        parameters=[{'robot_description': robot_desc}, {'use_sim_time': True}],
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
