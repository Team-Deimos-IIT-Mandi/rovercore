import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='depot',
        description='Simulation world (e.g., depot, mars)'
    )

    arm_arg = DeclareLaunchArgument(
        'arm_enabled', default_value='false',
        description='Include arm controllers and MoveIt move_group in sim'
    )

    target_lat_arg = DeclareLaunchArgument(
        'target_lat', default_value='39.8940850',
        description='Target GPS Latitude (defaults to 2m ahead of astronaut in mars world)'
    )

    target_lon_arg = DeclareLaunchArgument(
        'target_lon', default_value='32.7846688',
        description='Target GPS Longitude (defaults to 2m ahead of astronaut in mars world)'
    )

    world = LaunchConfiguration('world')
    arm_enabled = LaunchConfiguration('arm_enabled')

    # ---------------------------------------------------------------------------
    # 1. Simulation (Gazebo + RSP + bridges)
    # ---------------------------------------------------------------------------
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    # ---------------------------------------------------------------------------
    # 2. State estimation (dual EKF + NavSat + sensor pre-processing nodes)
    # ---------------------------------------------------------------------------
    state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'state_estimation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items(),
    )

    # ---------------------------------------------------------------------------
    # 3. Nav2 full stack + depth→pointcloud
    # ---------------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items(),
    )

    # ---------------------------------------------------------------------------
    # 4. Safety nodes
    # ---------------------------------------------------------------------------
    watchdog = Node(
        package='rover_description',
        executable='watchdog_node.py',
        name='watchdog_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'timeout_sec': 2.0,
            'gpio_pin': 12,
        }],
    )

    slope_detector = Node(
        package='rover_description',
        executable='slope_detector.py',
        name='slope_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ---------------------------------------------------------------------------
    # 5. Mission nodes
    # ---------------------------------------------------------------------------
    mission_initializer = Node(
        package='rover_description',
        executable='mission_initializer.py',
        name='mission_initializer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_lat': LaunchConfiguration('target_lat'),
            'target_lon': LaunchConfiguration('target_lon'),
        }],
    )

    spiral_search = Node(
        package='rover_description',
        executable='spiral_search.py',
        name='spiral_search',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    aruco_detection = Node(
        package='rover_description',
        executable='aruco_detection.py',
        name='aruco_detection',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    cmd_vel_relay = Node(
        package='rover_description',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    #---------------------------------------------
    #ARC NIGHT MISSION
    #------------------------------------------------

    #-

    return LaunchDescription([
        world_arg,
        arm_arg,
        # target_lat_arg,
        # target_lon_arg,
        sim_launch,
        state_estimation_launch,
        nav2_launch,
        # watchdog,
        # slope_detector,
        # mission_initializer,
        # spiral_search,
        aruco_detection,
        cmd_vel_relay,
    ])
