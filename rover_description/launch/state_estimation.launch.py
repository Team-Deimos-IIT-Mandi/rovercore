import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')

    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat.yaml')

    # --- EKF Local: odom frame (IMU + wheels + optical flow + constraints) ---
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[ekf_local_config],
        remappings=[('odometry/filtered', '/odometry/local')]
    )

    # --- EKF Global: map frame (IMU + local odom + GPS) ---
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config],
        remappings=[('odometry/filtered', '/odometry/global')]
    )

    # --- NavSat Transform: GPS lat/lon → odom-frame XY ---
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config],
        remappings=[
            ('imu', '/imu/filtered'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/local'),
            ('odometry/gps', '/gps/odom'),
        ]
    )

    # --- Optical Flow Node ---
    optical_flow = Node(
        package='rover_description',
        executable='optical_flow_node.py',
        name='optical_flow_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    imu_filter = Node(
        package='rover_description',
        executable='imu_filter_node.py',
        name='imu_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'imu_calibration.yaml'),
            {'use_sim_time': True}
        ]
    )

    flow_derotation = Node(
        package='rover_description',
        executable='flow_derotation_node.py',
        name='flow_derotation_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    nonholonomic = Node(
        package='rover_description',
        executable='nonholonomic_node.py',
        name='nonholonomic_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    slip_detector = Node(
        package='rover_description',
        executable='slip_detector_node.py',
        name='slip_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    gps_gating = Node(
        package='rover_description',
        executable='gps_gating_node.py',
        name='gps_gating_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    zupt = Node(
        package='rover_description',
        executable='zupt_node.py',
        name='zupt_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        ekf_local,
        ekf_global,
        navsat_transform,
        optical_flow,
        imu_filter,
        flow_derotation,
        nonholonomic,
        slip_detector,
        gps_gating,
        zupt,
    ])
