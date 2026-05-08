from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

# Note: Foxglove bridge is started by aruco.launch.py -> gz_plus_control.launch.py -> gazebo_sim.launch.py
# Connect to ws://localhost:8765 for visualization

def generate_launch_description():

    pkg_share = get_package_share_directory('rover_description')
    aruco_gazebo_launch_path = os.path.join(pkg_share, 'launch', 'aruco.launch.py')
    navigation_path = os.path.join(pkg_share,'launch','gps_navigation.launch.py')

    # Default GPS target - offset from world origin (49.9, 8.9) 
    # These defaults create a target ~30m away from spawn point
    lat_arg = DeclareLaunchArgument('lat', default_value='49.90027')  # ~30m north
    lon_arg = DeclareLaunchArgument('lon', default_value='8.90036')   # ~30m east
    target_lat_val = LaunchConfiguration('lat')
    target_lon_val = LaunchConfiguration('lon')

    aruco_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_gazebo_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    initializer_node = Node(
        package='rover_description',
        executable='mission_initializer.py',
        name='mission_initializer',
        parameters=[{
            'use_sim_time': True,
            'target_lat': target_lat_val,
            'target_lon': target_lon_val,
        }],
        output='screen',
        emulate_tty=True 
    )

    spiral_node = Node(
        package='rover_description',
        executable='spiral_search.py', 
        name='spiral_waypoint_generator',
        parameters=[{
            'use_sim_time': True,
        }],
        output='screen'
    )

    # ArUco Nodes
    camera_1 = Node(package='rover_description', 
                    executable='aruco_detection.py', 
                    name='aruco_front', 
                    parameters=[{
                        'use_sim_time': True,
                    }],output='screen')
    camera_2 = Node(package='rover_description', 
                    executable='aruco_detection1.py', 
                    name='aruco_left', 
                    parameters=[{
                        'use_sim_time': True,
                    }],output='screen')
    camera_3 = Node(package='rover_description', 
                    executable='aruco_detection2.py', 
                    name='aruco_right', 
                    parameters=[{
                        'use_sim_time': True,
                    }],output='screen')

    return LaunchDescription([
        lat_arg,
        lon_arg,
        aruco_gz_sim,
        navigation_launch,
        initializer_node,
        spiral_node,
        camera_1,
        camera_2,
        camera_3
    ])