#!/usr/bin/env python3
"""
Main launch file for robotic arm with Gazebo simulation and RViz visualization
This is a modular launch file that coordinates all components with proper timing
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    TimerAction,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    """
    Main launch file that brings up the complete robotic arm system
    
    Launch sequence:
    1. Robot State Publisher (publishes robot description)
    2. Gazebo Sim (simulation environment)
    3. Spawn Robot (after 3 second delay)
    4. Controllers (after 2 more seconds)
    5. RViz (visualization)
    """
    
    pkg_name = 'my_robotic_arm'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Get launch file paths
    robot_state_publisher_launch = os.path.join(pkg_share, 'launch', 'robot_state_publisher.launch.py')
    gazebo_launch = os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
    spawn_robot_launch = os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
    controllers_launch = os.path.join(pkg_share, 'launch', 'controllers.launch.py')
    rviz_launch = os.path.join(pkg_share, 'launch', 'rviz.launch.py')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
    
    # 1. Robot State Publisher (starts immediately)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_state_publisher_launch)
    )
    
    # 2. Gazebo (starts immediately)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )
    
    # 3. Spawn Robot (delayed to let Gazebo start)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_robot_launch)
            )
        ]
    )
    
    # 4. Controllers (delayed to let robot spawn)
    controllers = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controllers_launch)
            )
        ]
    )
    
    # 5. RViz (starts immediately for visualization)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch)
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        controllers,
        rviz,
        bridge
    ])