import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bt_xml_path = os.path.join(pkg_share, 'config', 'navigate_and_search_bt.xml')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if True'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # ---------------------------------------------------------------------------
    # Nav2 bringup (lifecycle-managed stack)
    # ---------------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
            'default_bt_xml_filename': bt_xml_path,
        }.items(),
    )

    # ---------------------------------------------------------------------------
    # depth_image_proc: convert /rgbd_camera/depth_image → /rgbd_camera/points
    #
    # NOTE: This node also requires a CameraInfo message on the topic
    #       /rgbd_camera/camera_info.  Make sure the Gazebo bridge publishes that
    #       topic (add the bridge argument for
    #       /rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
    #       in sim.launch.py) or that a camera_info_manager node supplies it.
    # ---------------------------------------------------------------------------
    depth_to_pointcloud = Node(
    package='depth_image_proc',
    executable='point_cloud_xyz_node',
    name='depth_image_to_pointcloud',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    remappings=[
        # Point this to the depth topic published by sim.launch.py
        ('image_rect', '/rgbd_camera/depth_image'), 
        
        ('camera_info', '/rgbd_camera/camera_info'), 
        
        ('points', '/rgbd_camera/points'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        nav2_launch,
        depth_to_pointcloud,
    ])
