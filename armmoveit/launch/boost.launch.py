import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('armmoveit')
    sim_launch = os.path.join(pkg_dir, 'launch', 'sim.launch.py')
    scripts_dir = os.path.join(pkg_dir, 'scripts')

    depth_sanitizer = os.path.join(scripts_dir, 'depth_sanitize.py')
    hsv_viewer = os.path.join(scripts_dir, 'display_hsv.py')
    teleport = os.path.join(scripts_dir, 'teleport_repair_tool.py')

    include_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch))

    # Start the sanitizer and HSV viewer shortly after sim to let topics appear
    start_sanitizer = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(cmd=['python3', depth_sanitizer], output='screen')]
    )

    start_hsv = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(cmd=['python3', hsv_viewer], output='screen')]
    )

    # Teleport once after sim and spawn complete
    teleport_action = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(cmd=['python3', teleport], output='screen')]
    )

    return LaunchDescription([
        include_sim,
        start_sanitizer,
        start_hsv,
        teleport_action,
    ])
