import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    AppendEnvironmentVariable,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.event_handlers import OnProcessIO, OnProcessExit


def generate_launch_description():
    #set the simulation time

    sim_time = {'use_sim_time': True}

    #load all package directory

    rover_pkg     = get_package_share_directory('rover_description')
    arm_pkg       = get_package_share_directory('my_robotic_arm')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    armmoveit_pkg = get_package_share_directory('armmoveit')

    #
    controllers_file = os.path.join(
    armmoveit_pkg,
    'config',
    'ros2_controllers.yaml'
    )

    armmoveit_models = os.path.join(armmoveit_pkg, 'models')


    combined_resource_paths = (
        f"{os.path.join(rover_pkg, '..')}:"
        f"{os.path.join(arm_pkg, '..')}:"          
        f"{os.path.join(armmoveit_pkg, '..')}:"     
        f"{armmoveit_models}"                      
    )


    set_ign_resource = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', combined_resource_paths
    )
    set_gz_resource = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', combined_resource_paths
    )

    urdf_file = os.path.join(rover_pkg, 'urdf', 'rarm.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc_string = f.read()

    robot_desc_string = robot_desc_string.replace(
        '$(find armmoveit)/config/ros2_controllers.yaml',
        controllers_file
    )
    robot_description_dict = {'robot_description': robot_desc_string}


    urdf_file = os.path.join(rover_pkg, 'urdf', 'rarm.urdf')
    moveit_config = (
        MoveItConfigsBuilder("rover_with_arm", package_name="armmoveit")
        .robot_description(file_path=urdf_file)  # <--- ADD THIS LINE
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_dict, sim_time]
    )
    # Assuming you already have rover_pkg defined via get_package_share_directory
    rover_models_path = os.path.join(rover_pkg, 'models')

    # Add this action to tell Gazebo where to look
    set_gazebo_model_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=rover_models_path
    )

    world_file = os.path.join(rover_pkg, 'worlds', 'mars.world.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. Clock bridge
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            
            # 2. Front Cameras
            '/cam/front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cam/front_left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/front_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cam/front_right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/front_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # 3. Side & Rear Cameras
            '/cam/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cam/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cam/rear/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/rear/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # 4. Specialty Cameras (Science & Optical Flow)
            '/cam/science/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cam/science/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/flow_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/flow_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # 5. RGBD Camera (OPTICAL link)
            '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',

            # 6. Gripper RGBD Camera
            '/arm/gripper/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/arm/gripper/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=[
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'rover_with_arm',
            '-allow_renaming', 'true',
            '-z', '5'
        ],
        output='screen',
    )
    delay_spawn = TimerAction(period=5.0, actions=[spawn])

    # ── PHASE 3: MoveGroup (12s) ─────────────────────────────────────
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),    # <--- This single line passes EVERY hidden MoveIt flag
            robot_description_dict,     # Your direct URDF string
            sim_time                    # {'use_sim_time': True}
        ],
    )

    delay_move_group = TimerAction(period=10.0, actions=[move_group])

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "100"]
    )
    arm_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager-timeout", "100"]
    )
    grip_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager-timeout", "100"]
    )
    rover_base_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_base_controller", "--controller-manager-timeout", "100"],
    )
    
    # ADDED TIMERACTION HERE: Wait 5 seconds AFTER spawn request finishes
    # to let Gazebo finish generating the newly added camera sensors.
    delay_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[
                TimerAction(
                    period=5.0, 
                    actions=[jsb, arm_ctrl, grip_ctrl, rover_base_spawner]
                )
            ] 
        )
    )
    
    return LaunchDescription([
        set_ign_resource,
        set_gazebo_model_path,
        set_gz_resource,
        rsp,
        gazebo,
        gz_bridge,
        delay_spawn,
        delay_move_group, 
        delay_spawners,
        imu_bridge,
    ])
