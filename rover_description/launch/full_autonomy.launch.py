import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')

    # ---------------------------------------------------------------------------
    # Resource Path Configuration (Gazebo Harmonic / Gz Sim 8)
    # ---------------------------------------------------------------------------
    # Locate the space_robotics_gz_envs asset cache
    srb_assets_cache = os.path.normpath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..',
                     'src', 'space_robotics_gz_envs', 'assets', 'cache')
    )
    
    # Combine the workspace install directories and local model folders
    gz_resource_path = os.pathsep.join(filter(os.path.isdir, [
        os.path.dirname(pkg_share),        # rover meshes / standard package models
        os.path.join(pkg_share, 'models'), # rover_description custom models
        srb_assets_cache,                  # procgen martian/lunar rocks
    ]))

    # Export variables immediately so all child launch files and nodes inherit them
    set_gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)
    set_ign_resource_path = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_resource_path)

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='depot',
<<<<<<< HEAD
        description='Simulation world key (choices: depot, mars, moon, mars_array, moon_array)'
=======
        description='Simulation world (e.g., depot, mars)'
>>>>>>> 2807e26 (Temporary save)
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
    # The Spinal Cord: Relays standard cmd_vel to Gazebo's hardware topic
    # ---------------------------------------------------------------------------
<<<<<<< HEAD
=======
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

>>>>>>> 2807e26 (Temporary save)
    cmd_vel_relay = Node(
        package='rover_description',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ---------------------------------------------------------------------------
    # 5. Mission nodes
    # ---------------------------------------------------------------------------
    # ---------------------------------------------------------------------------
    # TF Tape: Connects Gazebo's proprietary camera frame to the ROS 2 base_link
    # ---------------------------------------------------------------------------
    # 1. Places the camera on the rover and pitches it down 0.5 rad
    camera_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf',
        arguments=['0.3', '0', '0.4', '0', '0.0', '0', 'base_link', 'camera_mount_link'],
        parameters=[{'use_sim_time': True}],
    )

    # 2. Rotates the mount link to the standard Optical Frame (Z forward, X right, Y down)
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tf',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_mount_link', 'Rover/base_link/rgbd_camera'],
        parameters=[{'use_sim_time': True}],
    )
    # THE BRAIN: The Action Client talking to Nav2
    mission_coordinator = Node(
        package='rover_description',
        executable='mission_coordinator.py',
        name='mission_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wp_astronaut': [16.0, -12.0],
            'wp_oxygen': [23.8, -6.0],
            'wp_base': [-2.0, 3.5]
        }],
    )

    # THE EYES: Your OpenCV script
    vision_worker = Node(
        package='rover_description',
        executable='task_fuel_trail.py', 
        name='vision_worker',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'linear_speed': 0.2,
            'kp_line': 0.005
        }],
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        world_arg,
        arm_arg,
        target_lat_arg,
        target_lon_arg,
        sim_launch,
        state_estimation_launch,
        nav2_launch,
        # watchdog,
        # slope_detector,
        cmd_vel_relay,
        mission_coordinator,
        vision_worker,
        camera_mount_tf,
        camera_optical_tf,
    ])
