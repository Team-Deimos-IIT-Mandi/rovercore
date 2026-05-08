import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([

    Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='mapping_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[ '-configuration_directory', config_dir,
                    '-configuration_basename','rover.lua'],
        remappings=[
                ('/imu', '/imu'), 
                ('/scan', '/scan'),
                ('/odom', '/odometry/global'),
                ('/fix', '/gps/fix')
            ]
        ),
    Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        ),
    ])