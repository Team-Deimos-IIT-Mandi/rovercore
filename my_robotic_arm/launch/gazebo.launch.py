#!/usr/bin/env python3
"""
Launch Gazebo simulation only
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch Gazebo Harmonic with empty world"""
    
    # Include: Gazebo Harmonic (Empty World) 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    return LaunchDescription([
        gz_sim,
    ])
