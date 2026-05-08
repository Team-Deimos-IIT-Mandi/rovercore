#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    ros2_control_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[ros2_control_path],
            output='both',
        ),
        
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller', '--param-file', ros2_control_path],
                    remappings=[('/diff_drive_controller/cmd_vel', '/diff_drive_controller/cmd_vel_stamped'),
                                ],
                    output='screen'
                )
            ]
        ),
    ])