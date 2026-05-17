import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Adjusted Paths based on your tree
    description_pkg = get_package_share_directory('rover_description')
    hardware_pkg = get_package_share_directory('rover_hardware')
    
    # 1. Get URDF from rover_description
    urdf_file = os.path.join(description_pkg, 'urdf', 'Assem10_real.urdf.xacro')
    robot_description_content = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    robot_description = {'robot_description': robot_description_content}

    # 2. Get Controllers from rover_hardware
    controller_config = os.path.join(hardware_pkg, 'config', 'controllers.yaml')

    # Main ros2_control_node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        remappings=[
            ('/diff_drive_controller/odom', '/odom'),
        ],
        output="screen",
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Relay /cmd_vel → controller's actual subscription topic
    cmd_vel_relay = Node(
        package='rover_description',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
    )

    # Controller Spawners
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        cmd_vel_relay,
        diff_drive_spawner,
        joint_broadcaster_spawner
    ])