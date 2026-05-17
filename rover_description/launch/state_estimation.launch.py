import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')

    ekf_local_config  = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    navsat_config     = os.path.join(pkg_share, 'config', 'navsat.yaml')

    # ---- Launch arguments ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='true = Gazebo sim, false = real hardware')

    use_sim_time = LaunchConfiguration('use_sim_time')

    gps_port_arg = DeclareLaunchArgument(
        'gps_port', default_value='/dev/rover_gps')
    flow_port_arg = DeclareLaunchArgument(
        'flow_port', default_value='/dev/rover_flow')
    imu_bus_arg = DeclareLaunchArgument(
        'imu_bus', default_value='1')
    imu_addr_arg = DeclareLaunchArgument(
        'imu_addr', default_value='105')
    imu_rate_arg = DeclareLaunchArgument(
        'imu_rate_hz', default_value='100.0')

    # ---- Hardware sensor bringup (real hardware only) ----
    sensors_hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensors_hw.launch.py')),
        launch_arguments={
            'gps_port':    LaunchConfiguration('gps_port'),
            'flow_port':   LaunchConfiguration('flow_port'),
            'imu_bus':     LaunchConfiguration('imu_bus'),
            'imu_addr':    LaunchConfiguration('imu_addr'),
            'imu_rate_hz': LaunchConfiguration('imu_rate_hz'),
        }.items(),
        condition=UnlessCondition(use_sim_time),
    )

    # ---- EKF Local: odom frame (IMU + wheels + optical flow + constraints) ---
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[ekf_local_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/local')]
    )

    # ---- EKF Global: map frame (IMU + local odom + GPS) ----
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/global')]
    )

    # ---- NavSat Transform: GPS lat/lon → odom-frame XY ----
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu', '/imu/filtered'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/local'),
            ('odometry/gps', '/gps/odom'),
        ]
    )

    # ---- Sim-only optical flow (camera-based Farneback) ----
    # On real hardware, mtf01_driver_node (inside sensors_hw) publishes the same topics.
    optical_flow_sim = Node(
        package='rover_description',
        executable='optical_flow_node.py',
        name='optical_flow_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_time),
    )

    # ---- IMU filter — sim only; sensors_hw starts it for real hardware ----
    imu_filter_sim = Node(
        package='rover_description',
        executable='imu_filter_node.py',
        name='imu_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'imu_calibration.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_sim_time),
    )

    # ---- Flow derotation — sim only; sensors_hw starts it for real hardware ----
    flow_derotation_sim = Node(
        package='rover_description',
        executable='flow_derotation_node.py',
        name='flow_derotation_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_time),
    )

    # ---- Always-on processing nodes ----
    nonholonomic = Node(
        package='rover_description',
        executable='nonholonomic_node.py',
        name='nonholonomic_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    slip_detector = Node(
        package='rover_description',
        executable='slip_detector_node.py',
        name='slip_detector_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    gps_gating = Node(
        package='rover_description',
        executable='gps_gating_node.py',
        name='gps_gating_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    zupt = Node(
        package='rover_description',
        executable='zupt_node.py',
        name='zupt_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gps_port_arg,
        flow_port_arg,
        imu_bus_arg,
        imu_addr_arg,
        imu_rate_arg,
        sensors_hw,
        ekf_local,
        ekf_global,
        navsat_transform,
        optical_flow_sim,
        imu_filter_sim,
        flow_derotation_sim,
        nonholonomic,
        slip_detector,
        gps_gating,
        zupt,
    ])
