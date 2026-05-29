import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction, SetEnvironmentVariable,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 🚨 THE NEW MOVEIT IMPORT
from moveit_configs_utils import MoveItConfigsBuilder

# ---------------------------------------------------------------------------
# Worlds catalogue
# ---------------------------------------------------------------------------
_PKG      = 'rover_description'
_SRB_DIR  = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    '..', '..', '..', '..', 'src', 'space_robotics_gz_envs',
)

def _srb(name):
    return os.path.normpath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..',
                     'src', 'space_robotics_gz_envs', 'worlds', name)
    )

WORLDS = {
    'depot':      {'world_name': 'depot',      'z': '0.50', 'file': None},
    'mars':       {'world_name': 'mars',       'z': '0.50', 'file': _srb('mars.sdf')},
    'moon':       {'world_name': 'moon',       'z': '0.50', 'file': _srb('moon.sdf')},
    'mars_array': {'world_name': 'mars_array', 'z': '0.50', 'file': _srb('mars_array.sdf')},
    'moon_array': {'world_name': 'moon_array', 'z': '0.50', 'file': _srb('moon_array.sdf')},
}


def launch_setup(context, *args, **kwargs):
    world_arg  = LaunchConfiguration('world').perform(context)
    pkg_share  = get_package_share_directory(_PKG)
    armmoveit_pkg = get_package_share_directory('armmoveit')

    world_cfg  = WORLDS.get(world_arg, WORLDS['depot'])
    world_name = world_cfg['world_name']
    world_file = world_cfg['file'] or os.path.join(pkg_share, 'worlds', 'empty.world')
    z_spawn    = world_cfg['z']

    urdf_file       = os.path.join(pkg_share, 'urdf', 'rarm.urdf')
    # Use MoveIt's RViz config so the motion planning panels show up
    rviz_config_file = os.path.join(armmoveit_pkg, 'config', 'moveit.rviz') 

    # ── Robot description ──────────────────────────────────────────────────
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', urdf_file]),
            value_type=str,
        )
    }

    # ── MoveIt Configuration Builder ───────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("rover_with_arm", package_name="armmoveit")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

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
                   '-name', 'Rover', '-z', z_spawn],
        output='screen',
    )

    def wp(suffix):
        return f'/world/{world_name}/model/Rover/{suffix}'

    node_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/Rover/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/Rover/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'{wp("joint_state")}@sensor_msgs/msg/JointState[ignition.msgs.Model',
            f'{wp("link/IMU/sensor/imu_sensor/imu")}@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            f'{wp("link/GPS/sensor/navsat_sensor/navsat")}@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'{wp("link/FLOW_CAM/sensor/flow_camera/image")}@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'{wp("link/FLOW_CAM/sensor/range_sensor/scan")}@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cam/front/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam/front/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cam/front_left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam/front_left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cam/front_right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam/front_right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cam/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cam/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cam/rear/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam/rear/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cam/science/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam/science/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        remappings=[
            ('/model/Rover/cmd_vel',   '/cmd_vel'),
            ('/model/Rover/odom',      '/odom'),
            (wp('joint_state'),                              '/joint_states_raw'),
            (wp('link/IMU/sensor/imu_sensor/imu'),           '/imu/data'),
            (wp('link/GPS/sensor/navsat_sensor/navsat'),     '/gps/fix'),
            (wp('link/FLOW_CAM/sensor/flow_camera/image'),   '/flow_cam/image'),
            (wp('link/FLOW_CAM/sensor/range_sensor/scan'),   '/range/height'),
        ],
        output='screen',
    )

    node_flow_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'FLOW_CAM', '--child-frame-id', 'Assem10/FLOW_CAM/flow_camera'],
        parameters=[{'use_sim_time': True}],
    )

    cam_tf_nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', link, '--child-frame-id', f'Assem10/{link}/{sensor}'],
            parameters=[{'use_sim_time': True}],
        )
        for link, sensor in [
            ('CAM_FRONT',   'cam_front'),
            ('CAM_FL',      'cam_fl'),
            ('CAM_FR',      'cam_fr'),
            ('CAM_LEFT',    'cam_left'),
            ('CAM_RIGHT',   'cam_right'),
            ('CAM_REAR',    'cam_rear'),
            ('CAM_SCIENCE', 'cam_science'),
        ]
    ]

    # ── Hardware Controllers Spawners ──────────────────────────────────────
    node_jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    node_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    node_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
    )

    # 🚨 ADDED: The Differential Drive Spawner 
    node_rover_base_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_base_controller", '--controller-manager', '/controller_manager'],
    )

    # ── The NATIVE MoveGroup Node ──────────────────────────────────────────
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),    
            robot_description,     
            {'use_sim_time': True}
        ],
    )

    # ── RViz Event Handler ─────────────────────────────────────────────────
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            robot_description,
            {'use_sim_time': True}
        ],
    )

    launch_rviz_when_ready = RegisterEventHandler(
        OnProcessIO(
            target_action=move_group,
            on_stdout=lambda event: (
                [node_rviz]
                if b'You can start planning now' in event.text
                else []
            )
        )
    )

    # ── Resource path (meshes + space assets) ─────────────────────────────
    srb_assets_cache = os.path.normpath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..',
                     'src', 'space_robotics_gz_envs', 'assets', 'cache')
    )
    
    arm_share = get_package_share_directory('my_robotic_arm') 

    gz_resource_path = os.pathsep.join(filter(os.path.isdir, [
        os.path.dirname(pkg_share),   
        os.path.dirname(arm_share),   
        srb_assets_cache,             
    ]))

    # ── Perfectly Timed Boot Sequence ──────────────────────────────────────
    return [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        node_rsp,
        node_restamper,
        gazebo_launch,
        node_bridge,
        node_flow_cam_tf,
        *cam_tf_nodes,
        
        # 1. Spawn the rover at 3 seconds
        TimerAction(period=3.0, actions=[node_spawn]),
        
        # 2. Spawn the hardware controllers at 8 seconds (Rover + Arm + Gripper)
        TimerAction(period=8.0, actions=[
            node_jsb_spawner, 
            node_arm_spawner, 
            node_gripper_spawner,
            node_rover_base_spawner
        ]),
        
        # 3. Boot the MoveIt IK Brain at 12 seconds
        TimerAction(period=12.0, actions=[move_group]),

        # 4. Boot RViz strictly when MoveIt is finished loading
        launch_rviz_when_ready
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