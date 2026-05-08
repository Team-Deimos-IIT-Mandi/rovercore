import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def process_xacro_with_file_paths(xacro_file, pkg_share, model_file='arucoMarker0.dae'):
    """Process xacro and convert package:// to file:// for Gazebo"""
    result = subprocess.run(
        ['xacro', xacro_file, f'model_file:={model_file}'],
        capture_output=True, text=True
    )
    urdf_content = result.stdout
    
    # Replace package:// with file:// for meshes
    mesh_path = os.path.join(pkg_share, 'meshes')
    urdf_content = urdf_content.replace(
        'package://rover_description/meshes',
        f'file://{mesh_path}'
    )
    return urdf_content


def generate_launch_description():
    # 1. Setup paths
    pkg_share = get_package_share_directory('rover_description')
    gz_plus_controller_path = os.path.join(pkg_share, 'launch', 'gz_plus_control.launch.py')
    xacro_file = os.path.join(pkg_share, 'urdf', 'aruco_marker.xacro')
    aruco_namespace = 'aruco_marker_1'

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_plus_controller_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    args = [
        DeclareLaunchArgument('x', default_value='29.16'),
        DeclareLaunchArgument('y', default_value='-15.56'),
        DeclareLaunchArgument('z', default_value='0.35'), 
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='1.0'),
        DeclareLaunchArgument('model_name', default_value='arucoMarker1'),
        DeclareLaunchArgument('model_file', default_value='arucoMarker0.dae')
    ]

    # Process xacro with file:// paths for Gazebo
    robot_description_content = process_xacro_with_file_paths(xacro_file, pkg_share)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=aruco_namespace,
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True, 'frame_prefix': f'{aruco_namespace}/'}]
    )

    spawn_aruco = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', LaunchConfiguration('model_name'),
            '-topic', f'/{aruco_namespace}/robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '-0.35',
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', '0.0',
        ],
        output='screen'
    )

    # Foxglove bridge for visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription(args + [
        gazebo_sim,
        robot_state_publisher_node,
        spawn_aruco,
        foxglove_bridge,
    ])




   

    