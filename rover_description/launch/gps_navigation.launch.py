#!/usr/bin/env python3
# Launch file for Nav2 with GPS localization (no AMCL)
# Refer this: https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('rover_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Paths to config files
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_gps_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    # Remap cmd_vel to diff_drive_controller topic
    remappings = [
        ('/cmd_vel', '/diff_drive_controller/cmd_vel'),
    ]
    
    # Nav2 bringup with remappings
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    
    # Create a group action to apply remappings to all Nav2 nodes
    nav2_with_remappings = GroupAction([
        PushRosNamespace(''),
        nav2_bringup,
    ])
    
    # CMD_VEL relay to convert Twist to TwistStamped for diff_drive_controller
    cmd_vel_relay = Node(
        package='rover_description',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        cmd_vel_relay,
        nav2_with_remappings,
    ])
