import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. Define package name and paths
    pkg_name = 'rover_description'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'Assem10.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')

    # 2. Process URDF/Xacro using modern Command and FindExecutable
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]), 
        value_type=str
    )
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

    # Spawn the robot in Gazebo
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'Rover',
                   '-z', '0.02'],
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
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # Bridge Ignition topics <-> ROS 2 topics
    # Bridge Ignition topics <-> ROS 2 topics
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # Odometry & Control (Using 'Rover')
            '/model/Rover/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/Rover/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            
            # THE MISSING PIECE: Bridge Gazebo's TF to ROS 2 TF so RViz gets 'odom'
            '/model/Rover/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

            # Joint States (wheels) - Using 'empty' world and 'Rover' model
            '/world/empty/model/Rover/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            
            # Sensors
            '/world/empty/model/Rover/link/IMU/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/world/empty/model/Rover/link/GPS/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/world/empty/model/Rover/link/FLOW_CAM/sensor/flow_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/world/empty/model/Rover/link/FLOW_CAM/sensor/range_sensor/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        remappings=[
            ('/model/Rover/cmd_vel', '/cmd_vel'),
            ('/model/Rover/odometry', '/odom'),
            ('/model/Rover/tf', '/tf'), # Remap Gazebo TF to standard ROS /tf
            ('/world/empty/model/Rover/joint_state', '/joint_states'),
            ('/world/empty/model/Rover/link/IMU/sensor/imu_sensor/imu', '/imu/data'),
            ('/world/empty/model/Rover/link/GPS/sensor/navsat_sensor/navsat', '/gps/fix'),
            ('/world/empty/model/Rover/link/FLOW_CAM/sensor/flow_camera/image', '/flow_cam/image'),
            ('/world/empty/model/Rover/link/FLOW_CAM/sensor/range_sensor/scan', '/range/height'),
        ],
        output='screen'
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
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_resource_path),
        node_robot_state_publisher,
        node_joint_state_publisher,
        gazebo_launch,
        node_ros_gz_bridge,
        node_flow_cam_frame,
        
        # Delay spawning to give Gazebo time to initialize the world
        TimerAction(period=3.0, actions=[node_spawn_entity]),
        TimerAction(period=5.0, actions=[node_rviz]),
    ])