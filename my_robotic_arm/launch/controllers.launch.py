#!/usr/bin/env python3
"""
Launch ros2_control controllers
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    """Launch the controllers with delays to ensure proper startup"""
    
    # Node to start the Joint State Broadcaster (Reads joint angles)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Node to start the Arm Controller (Moves the robot)
    # Delayed to allow joint_state_broadcaster to start first
    arm_controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller"],
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
