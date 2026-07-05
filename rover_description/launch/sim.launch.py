import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ---------------------------------------------------------------------------
# Worlds catalogue
#   key        = launch arg value
#   world_name = SDF <world name="..."> (used in bridge topic paths)
#   file       = absolute path to .world / .sdf
#   x/y/yaw     = rover spawn pose
#   z_spawn    = rover spawn height above terrain
# ---------------------------------------------------------------------------
_PKG      = 'rover_description'
_SRB_DIR  = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),  # src/
    '..', '..', '..', '..', 'src', 'space_robotics_gz_envs',
)

def _srb(name):
    """Absolute path to a space_robotics_gz_envs world file."""
    return os.path.normpath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..',
                     'src', 'space_robotics_gz_envs', 'worlds', name)
    )

WORLDS = {
    'depot': {
        'world_name': 'depot', 'x': '0.0', 'y': '0.0', 'yaw': '0.0',
        'z': '0.50', 'file': None
    },   # resolved at runtime from pkg_share
    'mars': {
        'world_name': 'mars_world', 'x': '-5.0', 'y': '3.5', 'yaw': '1.57',
        'z': '0.20', 'file': 'pkg_share'
    }, # Use pkg_share worlds/mars.world.sdf
    'moon': {
        'world_name': 'moon', 'x': '0.0', 'y': '0.0', 'yaw': '0.0',
        'z': '0.50', 'file': _srb('moon.sdf')
    },
    'mars_array': {
        'world_name': 'mars_array', 'x': '0.0', 'y': '0.0', 'yaw': '0.0',
        'z': '0.50', 'file': _srb('mars_array.sdf')
    },
    'moon_array': {
        'world_name': 'moon_array', 'x': '0.0', 'y': '0.0', 'yaw': '0.0',
        'z': '0.50', 'file': _srb('moon_array.sdf')
    },
}


def launch_setup(context, *args, **kwargs):
    world_arg  = LaunchConfiguration('world').perform(context)
    pkg_share  = get_package_share_directory(_PKG)

    world_cfg  = WORLDS.get(world_arg, WORLDS['depot'])
    world_name = world_cfg['world_name']
    if world_arg == 'mars':
        world_file = os.path.join(pkg_share, 'worlds', 'mars.world.sdf')
    else:
        world_file = world_cfg['file'] or os.path.join(pkg_share, 'worlds', 'empty.world')
    x_spawn    = world_cfg['x']
    y_spawn    = world_cfg['y']
    z_spawn    = world_cfg['z']
    yaw_spawn  = world_cfg['yaw']

    urdf_file       = os.path.join(pkg_share, 'urdf', 'Assem10.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')

    # ── Robot description ──────────────────────────────────────────────────
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', urdf_file]),
            value_type=str,
        )
    }

    # ── Resource path (meshes + space assets) ─────────────────────────────
    srb_assets_cache = os.path.normpath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..',
                     'src', 'space_robotics_gz_envs', 'assets', 'cache')
    )
    gz_resource_path = os.pathsep.join(filter(os.path.isdir, [
        os.path.dirname(pkg_share),   # rover meshes / models
        os.path.join(pkg_share, 'models'), # rover_description models (gale_crater_patch2, etc)
        srb_assets_cache,             # martian_surface*, martian_rock*, lunar_rock*
    ]))

    # ── World path helper ──────────────────────────────────────────────────
    def wp(suffix):
        return f'/world/{world_name}/model/Rover/{suffix}'

    # ── Nodes ──────────────────────────────────────────────────────────────
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    node_restamper = Node(
        package='rover_description',
        executable='joint_state_restamper.py',
        name='joint_state_restamper',
        parameters=[{'use_sim_time': True}],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    node_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'Rover',
                   '-x', x_spawn, '-y', y_spawn, '-z', z_spawn,
                   '-Y', yaw_spawn],
        output='screen',
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    node_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Drive
            '/model/Rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/Rover/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Joints / sensors using world-scoped paths
            f'{wp("joint_state")}@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix_raw@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/flow_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/range/height@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # RGBD depth camera
            '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # RGB cameras (publish directly to their <topic> names)
            '/cam/front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cam/front_left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/front_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cam/front_right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/front_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cam/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cam/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cam/rear/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/rear/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cam/science/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/science/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/model/Rover/cmd_vel',   '/cmd_vel'),
            ('/model/Rover/odom',      '/odom'),
            (wp('joint_state'),                              '/joint_states_raw'),
        ],
        output='screen',
    )

    sensor_tf_nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', link,
                       '--child-frame-id', f'Rover/base_link/{sensor}'],
            parameters=[{'use_sim_time': True}],
        )
        for link, sensor in [
            ('OPTICAL',     'rgbd_camera'),
            ('IMU',         'imu_sensor'),
            ('GPS',         'navsat_sensor'),
            ('FLOW_CAM',    'flow_camera'),
            ('FLOW_CAM',    'range_sensor'),
            ('CAM_FRONT',   'cam_front'),
            ('CAM_FL',      'cam_fl'),
            ('CAM_FR',      'cam_fr'),
            ('CAM_LEFT',    'cam_left'),
            ('CAM_RIGHT',   'cam_right'),
            ('CAM_REAR',    'cam_rear'),
            ('CAM_SCIENCE', 'cam_science'),
        ]
    ]

    # GPS covariance injector — Gazebo NavSat bridge outputs zero covariance;
    # this node injects sensible defaults before navsat_transform_node sees it.
    node_gps_cov = Node(
        package='rover_description',
        executable='gps_fix_covariance_node.py',
        name='gps_fix_covariance_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        node_rsp,
        node_restamper,
        gazebo_launch,
        node_bridge,
        node_gps_cov,
        *sensor_tf_nodes,
        TimerAction(period=3.0, actions=[node_spawn]),
        TimerAction(period=5.0, actions=[node_rviz]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='depot',
            choices=['depot', 'mars', 'moon', 'mars_array', 'moon_array'],
            description=(
                'Simulation world.  depot=flat test arena  '
                'mars/moon=terrain with rocks  '
                '*_array=multi-terrain grid.  '
                'Space worlds need procgen assets: '
                'run src/space_robotics_gz_envs/scripts/procgen_assets.bash first.'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
