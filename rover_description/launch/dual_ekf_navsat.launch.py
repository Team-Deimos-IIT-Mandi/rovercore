import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('rover_description')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}],
            remappings=[
                ('/odometry/filtered', '/odometry/local')
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}],
            remappings=[
                ('/odometry/filtered', '/odometry/global')
            ]
        ),

        # NavSat Transform Node: Integrates GPS and IMU
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}],
            remappings=[
                ('/gps/fix', '/gps/fix'),
                ('/imu', '/imu'),
                ('/odometry/filtered', '/odometry/global')
            ]
        ),

    ])