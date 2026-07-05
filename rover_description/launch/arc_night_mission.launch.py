from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_description',
            executable='mission_manager.py',
            name='mission_manager',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rover_description',
            executable='dome_exit.py',
            name='airlock_exit_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rover_description',
            executable='astronaut_searcher.py',
            name='astronaut_searcher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rover_description',
            executable='fuel_trail_follower.py',
            name='fuel_trail_follower',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rover_description',
            executable='oxygen_tank_navigator.py',
            name='oxygen_tank_navigator',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rover_description',
            executable='dome_return_navigator.py',
            name='dome_return_navigator',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rover_description',
            executable='aruco_detection.py',
            name='aruco_detection',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rover_description',
            executable='dome_insider.py',
            name='dome_insider_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
