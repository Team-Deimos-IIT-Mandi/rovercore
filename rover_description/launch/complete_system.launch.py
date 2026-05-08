#!/usr/bin/env python3
"""
Complete Mars Rover System Launch File - SEQUENTIAL STARTUP
Launches in order:
  1. Gazebo + Bridges
  2. Robot Controllers + State Publishers
  3. EKF + GPS localization
  4. Cartographer Mapping (optional, after controllers are ready)
  5. Nav2 Navigation (optional)
  6. Rviz Visualization
"""

import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_cartographer_delayed(context, *args, **kwargs):
    """Launch Cartographer after a delay to ensure controllers are ready"""
    enable_mapping = context.launch_configurations.get('enable_mapping', 'true')
    
    if enable_mapping.lower() == 'true':
        pkg_share = get_package_share_directory('rover_description')
        mapping_launch = os.path.join(pkg_share, 'launch', 'mapping.launch.py')
        use_sim_time = context.launch_configurations.get('use_sim_time', 'true')
        
        # Give controllers time to initialize
        print("\n[LAUNCH INFO] Waiting for controllers to initialize before starting Cartographer...")
        time.sleep(3)
        
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    return []


def launch_navigation(context, *args, **kwargs):
    """Launch Nav2 navigation if enabled"""
    enable_navigation = context.launch_configurations.get('enable_navigation', 'false')
    
    if enable_navigation.lower() == 'true':
        pkg_share = get_package_share_directory('rover_description')
        nav_launch = os.path.join(pkg_share, 'launch', 'gps_navigation.launch.py')
        use_sim_time = context.launch_configurations.get('use_sim_time', 'true')
        
        print("\n[LAUNCH INFO] Starting Nav2 GPS Navigation...")
        
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true'
                }.items()
            )
        ]
    return []


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    
    # Launch file paths
    gazebo_sim_launch = os.path.join(pkg_share, 'launch', 'gazebo_sim.launch.py')
    controllers_launch = os.path.join(pkg_share, 'launch', 'controllers.launch.py')
    ekf_navsat_launch = os.path.join(pkg_share, 'launch', 'dual_ekf_navsat.launch.py')
    
    # Rviz config
    rviz_config_file = os.path.join(pkg_share, 'config', 'rover.rviz')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_mapping = LaunchConfiguration('enable_mapping', default='true')
    enable_navigation = LaunchConfiguration('enable_navigation', default='false')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_enable_mapping = DeclareLaunchArgument(
        'enable_mapping',
        default_value='true',
        description='Enable Cartographer mapping (true/false)'
    )
    
    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='false',
        description='Enable Nav2 autonomous navigation (true/false)'
    )
    
    # ========== SEQUENCE 1: Gazebo + Bridges + Robot State Publisher ==========
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_sim_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # ========== SEQUENCE 2: Controllers (waits for Gazebo) ==========
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controllers_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # ========== SEQUENCE 3: EKF + NavSat Transform ==========
    ekf_navsat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_navsat_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # ========== SEQUENCE 4: Cartographer (delayed to let controllers load) ==========
    cartographer_delayed = OpaqueFunction(function=launch_cartographer_delayed)
    
    # ========== SEQUENCE 5: Nav2 Navigation (optional) ==========
    navigation = OpaqueFunction(function=launch_navigation)
    
    # ========== SEQUENCE 6: Rviz Visualization ==========
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ========== Foxglove Bridge for Web Visualization ==========
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_mapping,
        declare_enable_navigation,
        gazebo_sim,           # 1. Gazebo + bridges
        controllers,          # 2. Controllers (depends on Gazebo)
        ekf_navsat,           # 3. Localization
        cartographer_delayed, # 4. Mapping (delayed start, optional)
        navigation,           # 5. Navigation (optional)
        rviz_node,            # 6. Visualization
        foxglove_bridge,      # 7. Foxglove (ws://localhost:8765)
    ])
