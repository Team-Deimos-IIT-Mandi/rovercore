import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description="SLAM mode: 'mapping' (build a new map) or 'localization' "
                    "(localize against an existing map)"
    )

    map_file_path_arg = DeclareLaunchArgument(
        'map_file_path',
        default_value='',
        description='Absolute path to the map YAML file (required in localization mode)'
    )

    mode = LaunchConfiguration('mode')
    map_file_path = LaunchConfiguration('map_file_path')

    # Cartographer Lua config file lives in the installed share directory.
    # Create the path as a plain string (resolved at launch-description build
    # time) because cartographer_node takes --configuration_directory and
    # --configuration_basename as plain CLI arguments.
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    cartographer_config_basename = 'cartographer.lua'

    # ---------------------------------------------------------------------------
    # Cartographer node (runs in both modes; the Lua config itself can branch on
    # TRAJECTORY_BUILDER_nD.pure_localization_trimmer if needed, but mode is
    # controlled here at the argument level so the operator can choose)
    # ---------------------------------------------------------------------------

    # Mapping mode cartographer node
    cartographer_mapping = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        condition=UnlessCondition(
            PythonExpression(["'", mode, "' == 'localization'"])
        ),
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/local'),
        ],
    )

    # Localization mode cartographer node (loads a pre-built .pbstream map)
    cartographer_localization = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'localization'"])
        ),
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename,
            # Load the serialized state (pbstream) for pure localization.
            # Operators should convert their map with:
            #   cartographer_pbstream_to_ros_map  (if they have a .pbstream)
            # or pass the pbstream path via an additional launch arg.
            '-load_state_filename', map_file_path,
            '-start_trajectory_with_default_topics=false',
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/local'),
        ],
    )

    # ---------------------------------------------------------------------------
    # Cartographer occupancy grid node (runs in both modes)
    # ---------------------------------------------------------------------------
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ],
    )

    # ---------------------------------------------------------------------------
    # map_server — only in localization mode, to serve the static map to Nav2
    # (nav2_map_server provides the map_server executable in ROS 2 Humble)
    # ---------------------------------------------------------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'localization'"])
        ),
        parameters=[
            {'use_sim_time': True},
            {'yaml_filename': map_file_path},
        ],
    )

    # Lifecycle manager to bring map_server into the active state automatically
    # (required in ROS 2 Humble — map_server is a lifecycle node)
    map_server_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'localization'"])
        ),
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    return LaunchDescription([
        mode_arg,
        map_file_path_arg,
        cartographer_mapping,
        cartographer_localization,
        occupancy_grid_node,
        map_server,
        map_server_lifecycle_manager,
    ])
