import os
from launch import LaunchDescription
import rclpy
from rclpy.node import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # <-- THIS IS THE CORRECT IMPORT!
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('armmoveit')
    sim_launch = os.path.join(pkg_dir, 'launch', 'sim.launch.py')

    # 1. Start the simulation
    include_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch))

    # 2. Define the Tracker Node
    tracker_node = Node(
        package='armmoveit',
        executable='red_valve_tracker.py', 
        name='red_valve_tracker_node',
        output='screen'
    )

    # 4. Wrap them in timers so Gazebo has time to boot up the cameras first!
    delay_tracker = TimerAction(period=5.0, actions=[tracker_node])

    return LaunchDescription([
        include_sim,
        delay_tracker,
    ])