import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Define package name and paths
    pkg_name = 'rover_description'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'Assem10.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')

    # 2. Use xacro/Command to process the URDF/Xacro file
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': robot_description_content}

    # 3. Nodes and Launches

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Include Gazebo (Ignition) Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg_share, 'worlds', 'empty.world')}.items(),
    )

    # Spawn the robot in Gazebo (delayed to let Gazebo initialize)
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'Assem10',
                   '-z', '0.5',
                   '-P', '-1.5708'],
        output='screen'
    )

    # RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # Bridge Ignition topics <-> ROS 2 topics
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # Diff drive: cmd_vel (ROS->IGN) and odometry (IGN->ROS)
            '/model/Assem10/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/Assem10/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # Joint states
            '/world/depot/model/Assem10/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # IMU
            '/world/depot/model/Assem10/link/IMU/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # GPS
            '/world/depot/model/Assem10/link/GPS/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            # Camera image
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            # Lidar
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # Downward flow camera (world-scoped Ignition topic)
            '/world/depot/model/Assem10/link/FLOW_CAM/sensor/flow_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            # Downward rangefinder (world-scoped, single-beam lidar bridged as LaserScan)
            '/world/depot/model/Assem10/link/FLOW_CAM/sensor/range_sensor/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        remappings=[
            ('/model/Assem10/cmd_vel', '/cmd_vel'),
            ('/model/Assem10/odometry', '/odom'),
            ('/world/depot/model/Assem10/joint_state', '/joint_states'),
            ('/world/depot/model/Assem10/link/IMU/sensor/imu_sensor/imu', '/imu/data'),
            ('/world/depot/model/Assem10/link/GPS/sensor/navsat_sensor/navsat', '/gps/fix'),
            ('/world/depot/model/Assem10/link/FLOW_CAM/sensor/flow_camera/image', '/flow_cam/image'),
            ('/world/depot/model/Assem10/link/FLOW_CAM/sensor/range_sensor/scan', '/range/height'),
        ],
        output='screen'
    )

    # Static transforms for Ignition sensor frames → URDF link frames
    # Ignition uses model-scoped frame names (e.g. Assem10/base_link/lidar)
    node_lidar_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'Assem10/base_link/lidar',
                   '--x', '0.1', '--y', '0', '--z', '0.15'],
        parameters=[{'use_sim_time': True}]
    )

    node_flow_cam_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'FLOW_CAM', '--child-frame-id', 'Assem10/FLOW_CAM/flow_camera'],
        parameters=[{'use_sim_time': True}]
    )

    # Ignition needs to find meshes via model:// URIs
    gz_resource_path = os.path.dirname(pkg_share)

    # 4. Return the LaunchDescription
    return LaunchDescription([
        # Fix Ignition transport discovery on machines where multicast is restricted
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        # Allow Ignition to resolve package:// (model://) mesh URIs
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_resource_path),
        node_robot_state_publisher,
        gazebo_launch,
        node_ros_gz_bridge,
        node_lidar_frame,
        node_flow_cam_frame,
        # Delay spawn to allow Gazebo to fully initialize
        TimerAction(period=5.0, actions=[node_spawn_entity]),
        TimerAction(period=6.0, actions=[node_rviz]),
    ])
