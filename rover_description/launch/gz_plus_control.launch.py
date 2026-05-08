#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_share = get_package_share_directory('rover_description')
    
    gazebo_launch_path = os.path.join(pkg_share, 'launch', 'gazebo_sim.launch.py')
    controllers_launch_path = os.path.join(pkg_share, 'launch', 'controllers.launch.py')
    ekf_launch_path = os.path.join(pkg_share, 'launch', 'dual_ekf_navsat.launch.py')
    
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controllers_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        gazebo_sim,
        controllers,
        ekf,
    ])